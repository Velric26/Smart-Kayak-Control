// =====================================================================
//  Smart Kayak Thruster Control System  -  firmware entry point
//
//  Architecture: the fast control loop runs in loop() (already pinned to
//  core 1 by the Arduino-ESP32 runtime); a low-rate telemetry task runs
//  on core 0 so serial/WiFi jitter never disturbs control timing (§2.2).
//  As IMU/GPS come online (Phases 2/4) they become their own tasks.
//
//  Build:  pio run -e mule -t upload   (L298N rover)
//          pio run -e kayak            (dual-ESC kayak)
// =====================================================================
#include <Arduino.h>
#include <string.h>
#include "config.h"

#include "hal/MotorDriver.h"
#include "hal/RCReceiver.h"
#include "hal/BatteryMonitor.h"
#include "hal/BypassSense.h"
#include "control/Mixer.h"
#include "control/PID.h"
#include "statemachine/StateMachine.h"
#include "estimation/IMU.h"
#include "estimation/Heading.h"

// ---- HAL selection: the ONLY platform-specific include (§0, §1.2a) ---
#if defined(DRIVER_ESC)
  #include "hal/ESC_Driver.h"
  ESC_Driver   driver;
#else
  #include "hal/L298N_Driver.h"
  L298N_Driver driver;
#endif

RCReceiver     rc;
BatteryMonitor battery;
BypassSense    bypass;
StateMachine   sm;

// Controllers (used from Phase 3+). Declared now so the structure is set.
PID headingPID, distancePID;

// Heading estimation (Phase 2/3).
IMU         imu;
HeadingAHRS ahrs;
bool        imuOk = false;
float       headingSetpoint = 0;   // captured on entering HEADING_HOLD

// Runtime-tunable heading params (adjust live over serial, no reflash):
//   "kp 0.01"  "kd 0.004"  "db 5"   then Enter.
float hdgKp = HDG_KP, hdgKd = HDG_KD, hdgDeadband = HEADING_DEADBAND_DEG;

// Shared snapshot for the telemetry task (core 0). Guarded by a spinlock.
struct Snapshot {
  Mode  mode = Mode::BOOT;
  float left = 0, right = 0;
  float battV = 0;
  bool  linkOk = false, bypassManual = false;
  bool  armReq = false;
  int   modeSel = 0;
  float heading = 0, setpoint = 0;
  float appliedL = 0, appliedR = 0;   // post min-drive (what the motor gets)
} snap;
portMUX_TYPE snapMux = portMUX_INITIALIZER_UNLOCKED;

float cmdL = 0, cmdR = 0;   // slew-limited outputs (persist across ticks)

// ---------------------------------------------------------------------
//  Telemetry task (core 0) — serial for now; WiFi/BLE later (Phase 7).
// ---------------------------------------------------------------------
void telemetryTask(void*) {
  const TickType_t period = pdMS_TO_TICKS(1000 / TELEMETRY_HZ);
  for (;;) {
    Snapshot s;
    portENTER_CRITICAL(&snapMux); s = snap; portEXIT_CRITICAL(&snapMux);
    Serial.printf("[%-14s] L=%+.2f>%+.2f R=%+.2f>%+.2f  hdg=%5.1f sp=%5.1f  arm=%d modeSel=%d  batt=%.2fV  link=%s%s\n",
                  modeName(s.mode), s.left, s.appliedL, s.right, s.appliedR, s.heading, s.setpoint,
                  (int)s.armReq, s.modeSel, s.battV,
                  s.linkOk ? "OK" : "LOST",
                  s.bypassManual ? "  BYPASS=MANUAL" : "");
    vTaskDelay(period);
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // Outputs to neutral BEFORE anything else (§1.3).
  driver.begin();

  rc.begin();
  battery.begin();
  bypass.begin();
  sm.begin();
  pinMode(PIN_STATUS_LED, OUTPUT);

  headingPID.setGains(hdgKp, HDG_KI, hdgKd);
  distancePID.setGains(POS_KP, POS_KI, POS_KD);

  imuOk = imu.begin();
  if (imuOk) {
    Serial.println("IMU OK - calibrating gyro bias, hold still...");
    ahrs.begin(&imu);
    Serial.println("Heading estimator ready.");
  } else {
    Serial.println("IMU not found - HEADING_HOLD will be inert.");
  }

  xTaskCreatePinnedToCore(telemetryTask, "telemetry", 4096, nullptr, 1, nullptr, 0);

  Serial.printf("\nSmart Kayak Control online. Driver=%s\n", driver.name());
}

// Live heading tuning over serial: "kp <v>", "kd <v>", "db <v>" + Enter.
void handleSerialTuning() {
  static char buf[40];
  static uint8_t bi = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      buf[bi] = 0;
      if (bi > 0) {
        char cmd[8]; float val;
        if (sscanf(buf, "%7s %f", cmd, &val) == 2) {
          if      (!strcmp(cmd, "kp")) hdgKp = val;
          else if (!strcmp(cmd, "kd")) hdgKd = val;
          else if (!strcmp(cmd, "db")) hdgDeadband = val;
          headingPID.setGains(hdgKp, HDG_KI, hdgKd);
          Serial.printf(">> kp=%.4f  kd=%.4f  db=%.1f\n", hdgKp, hdgKd, hdgDeadband);
        }
      }
      bi = 0;
    } else if (bi < sizeof(buf) - 1) {
      buf[bi++] = c;
    }
  }
}

void loop() {
  // ---- Fixed-rate scheduling (core 1) ----
  static TickType_t wake = xTaskGetTickCount();
  static uint32_t   lastUs = micros();
  vTaskDelayUntil(&wake, pdMS_TO_TICKS(1000 / CONTROL_HZ));

  uint32_t nowUs = micros();
  float dt = (nowUs - lastUs) * 1e-6f;
  lastUs = nowUs;

  handleSerialTuning();

  // ---- Sense ----
  static uint8_t battDiv = 0;
  if (++battDiv >= CONTROL_HZ / 10) { battery.update(); battDiv = 0; } // ~10 Hz

  bool linkOk       = rc.linkOk();
  bool bypassManual = bypass.isManual();

  SMInputs in;
  in.rcLinkOk        = linkOk;
  in.batteryCritical = battery.critical();
  in.bypassManual    = bypassManual;
  in.armRequest      = rc.norm(RC_ARM) > 0.5f;   // CH5 latching: held = armed
  in.sticksNeutral   = (fabsf(rc.norm(RC_LEFT)) < 0.1f) &&
                       (fabsf(rc.norm(RC_RIGHT)) < 0.1f);
  // MODE: CH6 momentary button -> advance one mode per press (rising edge).
  // If a press doesn't register, this button's active polarity is reversed:
  // change ">  0.5f" to "< -0.5f".
  static bool modePrev = false;
  static int  modeIndex = 0;
  bool modePressed = rc.norm(RC_MODE) > 0.5f;
  if (modePressed && !modePrev) modeIndex = (modeIndex + 1) % 4;
  modePrev = modePressed;
  in.modeSelect = modeIndex;

  sm.update(in);
  if (sm.justReengaged()) { headingPID.reset(); distancePID.reset(); }

  // Advance the heading estimate, and capture the setpoint when we first
  // enter HEADING_HOLD (hold whatever direction the boat faces on engage).
  if (imuOk) ahrs.update(dt);
  static Mode prevMode = Mode::BOOT;
  if (sm.mode() == Mode::HEADING_HOLD && prevMode != Mode::HEADING_HOLD) {
    headingSetpoint = ahrs.deg();
    headingPID.reset();
  }
  prevMode = sm.mode();

  // ---- Decide target thrust by mode ----
  float tgtL = 0, tgtR = 0;
  switch (sm.mode()) {
    case Mode::MANUAL:
      // Direct differential from the DS600's already-mixed L/R (§8.1).
      tgtL = rc.norm(RC_LEFT);
      tgtR = rc.norm(RC_RIGHT);
      break;

    case Mode::HEADING_HOLD:
      if (imuOk) {
        float err = headingError(headingSetpoint, ahrs.deg());        // shortest signed error (deg)
        float w;
        if (fabsf(err) < hdgDeadband) { w = 0.0f; headingPID.reset(); }  // close enough: hold quietly
        else { w = HEADING_TURN_SIGN * headingPID.update(err, dt); }     // correct
        float v = 0.5f * (rc.norm(RC_LEFT) + rc.norm(RC_RIGHT));         // optional forward from stick
        mix(v, w, tgtL, tgtR);
      } else {
        tgtL = 0; tgtR = 0;   // no IMU -> can't hold heading
      }
      break;

    case Mode::ANCHOR:
    case Mode::ANCHOR_HEADING:
      // TODO Phases 5-6: GPS position hold. Inert until GPS is mounted.
      tgtL = 0; tgtR = 0;
      break;

    case Mode::BOOT:
    case Mode::DISARMED:
    case Mode::FAILSAFE:
    default:
      tgtL = 0; tgtR = 0;
      break;
  }

  // ---- Output: slew-limit for smoothness, then drive ----
  float step = THRUST_SLEW_PER_S * dt;
  cmdL = slew(tgtL, cmdL, step);
  cmdR = slew(tgtR, cmdR, step);

  bool armed = (sm.mode() == Mode::MANUAL || sm.mode() == Mode::HEADING_HOLD ||
                sm.mode() == Mode::ANCHOR || sm.mode() == Mode::ANCHOR_HEADING);
  if (armed) driver.setThrust(cmdL, cmdR);
  else       { driver.disable(); cmdL = cmdR = 0; }

  digitalWrite(PIN_STATUS_LED, armed ? HIGH : (millis() / 250) & 1); // solid=armed, blink=safe

  // ---- Publish snapshot ----
  portENTER_CRITICAL(&snapMux);
  snap.mode = sm.mode(); snap.left = cmdL; snap.right = cmdR;
  snap.appliedL = driver.lastL; snap.appliedR = driver.lastR;
  snap.battV = battery.volts(); snap.linkOk = linkOk; snap.bypassManual = bypassManual;
  snap.armReq = in.armRequest; snap.modeSel = in.modeSelect;
  snap.heading = imuOk ? ahrs.deg() : 0.0f;
  snap.setpoint = headingSetpoint;
  portEXIT_CRITICAL(&snapMux);
}
