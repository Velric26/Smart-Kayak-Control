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
#include <BluetoothSerial.h>
#include <Preferences.h>
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

BluetoothSerial SerialBT;   // mirrors telemetry + tuning input over BT SPP
Preferences     prefs;      // NVS store for auto-tuned heading gains

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

// Telemetry print rate (Hz), live-adjustable: "log 0" mutes, "log 5" = 5 Hz.
volatile int logHz = TELEMETRY_HZ;

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
  char line[160];
  for (;;) {
    int hz = logHz;
    if (hz <= 0) { vTaskDelay(pdMS_TO_TICKS(200)); continue; }  // muted
    vTaskDelay(pdMS_TO_TICKS(1000 / hz));
    Snapshot s;
    portENTER_CRITICAL(&snapMux); s = snap; portEXIT_CRITICAL(&snapMux);
    snprintf(line, sizeof(line),
             // batt= omitted - divider not wired yet, see hal/BatteryMonitor.cpp.
             "[%-14s] L=%+.2f>%+.2f R=%+.2f>%+.2f  hdg=%5.1f sp=%5.1f  arm=%d modeSel=%d  link=%s%s\r\n",
             modeName(s.mode), s.left, s.appliedL, s.right, s.appliedR, s.heading, s.setpoint,
             (int)s.armReq, s.modeSel,
             s.linkOk ? "OK" : "LOST",
             s.bypassManual ? "  BYPASS=MANUAL" : "");
    Serial.print(line);
    SerialBT.print(line);
  }
}

void setup() {
  Serial.begin(115200);
  delay(200);
  SerialBT.begin(BT_DEVICE_NAME);
  Serial.printf("Bluetooth SPP up as \"%s\" - pair and open a serial terminal.\r\n", BT_DEVICE_NAME);

  // Outputs to neutral BEFORE anything else (§1.3).
  driver.begin();

  rc.begin();
  battery.begin();
  bypass.begin();
  sm.begin();
  pinMode(PIN_STATUS_LED, OUTPUT);

  // Restore auto-tuned heading gains from NVS (fall back to config defaults).
  prefs.begin("kayak", false);
  hdgKp = prefs.getFloat("hdgKp", HDG_KP);
  hdgKd = prefs.getFloat("hdgKd", HDG_KD);
  Serial.printf("Heading gains: kp=%.4f kd=%.4f%s\r\n", hdgKp, hdgKd,
                prefs.isKey("hdgKp") ? " (from auto-tune/NVS)" : " (config default)");

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

  Serial.printf("\r\nSmart Kayak Control online. Driver=%s\r\n", driver.name());
}

// =====================================================================
//  Relay-feedback heading auto-tune (Astrom-Hagglund method).
//  Drives a bang-bang turn command around the engage heading; the rover
//  settles into a limit cycle. From its period (Tu) and amplitude we get
//  the ultimate gain Ku = 4*amp / (pi*amp_deg), then apply a conservative
//  ("no-overshoot") PD set. Runs ONLY in HEADING_HOLD; any mode/RC change
//  aborts it to neutral. Live-tunes hdgKp/hdgKd; bake into config.h if good.
// =====================================================================
struct AutoTune {
  bool   active = false;
  float  amp  = 0.45f;        // relay turn-command magnitude (must rotate the rover)
  float  hyst = 2.0f;         // deg hysteresis around center (rejects mag noise)
  float  center = 0;
  int    sign = 0;
  uint32_t startMs = 0, lastRiseMs = 0;
  int    periods = 0;         // completed full periods seen
  static const int WARMUP = 1;   // discard first period (startup transient)
  static const int TARGET = 5;   // periods to average
  float  periodSum = 0, ampSum = 0;
  float  winMax = -1e9f, winMin = 1e9f;
  int    savedLog = TELEMETRY_HZ;   // telemetry rate to restore when done

  void echo(const char* s) { Serial.print(s); SerialBT.print(s); }

  void start() {
    active = true; center = ahrs.deg(); sign = 0;
    periods = 0; periodSum = 0; ampSum = 0;
    winMax = -1e9f; winMin = 1e9f;
    startMs = lastRiseMs = millis();
    savedLog = logHz; logHz = 0;    // mute telemetry so the result is easy to read
    char m[100];
    snprintf(m, sizeof(m), ">> AUTOTUNE start (amp=%.2f). Keep the area clear - rover will wag in place.\r\n", amp);
    echo(m);
  }

  void abort(const char* why) {
    if (!active) return;
    active = false; logHz = savedLog;
    char m[80];
    snprintf(m, sizeof(m), ">> AUTOTUNE aborted: %s\r\n", why);
    echo(m);
  }

  void finish() {
    active = false; logHz = savedLog;
    float Tu = periodSum / TARGET;
    float a  = ampSum   / TARGET;
    if (a < 0.5f || Tu < 0.05f) {
      echo(">> AUTOTUNE failed: oscillation too small/fast - raise amp and retry.\r\n");
      return;
    }
    float Ku = 4.0f * amp / (PI * a);
    float kp = 0.20f * Ku;          // no-overshoot relay rule (gentle)
    float kd = kp * 0.33f * Tu;
    hdgKp = kp; hdgKd = kd;
    headingPID.setGains(hdgKp, HDG_KI, hdgKd);
    prefs.putFloat("hdgKp", kp);     // persist so the gentle PD survives reboot
    prefs.putFloat("hdgKd", kd);
    char m[240];
    snprintf(m, sizeof(m),
      ">> AUTOTUNE done: Tu=%.2fs amp=%.1fdeg Ku=%.4f\r\n"
      ">> applied + saved gentle PD: kp=%.4f kd=%.4f  (aggressive ZN ref: kp=%.4f kd=%.4f)\r\n"
      ">> survives reboot now; say the word to also bake into config.h.\r\n",
      Tu, a, Ku, kp, kd, 0.6f * Ku, 0.6f * Ku * Tu / 8.0f);
    echo(m);
  }

  // Produces the relay turn command via tgtL/tgtR. Clears active when done.
  void step(float heading, float& tgtL, float& tgtR) {
    float err = headingError(center, heading);
    if (err > winMax) winMax = err;
    if (err < winMin) winMin = err;

    int prev = sign;
    if      (err > +hyst) sign = +1;
    else if (err < -hyst) sign = -1;   // else: hold previous sign

    if (prev == -1 && sign == +1) {    // rising edge = one full period
      uint32_t now = millis();
      if (periods >= WARMUP) {
        periodSum += (now - lastRiseMs) * 1e-3f;
        ampSum    += (winMax - winMin) * 0.5f;
      }
      lastRiseMs = now;
      periods++;
      winMax = -1e9f; winMin = 1e9f;
      if (periods >= WARMUP + TARGET) { finish(); tgtL = tgtR = 0; return; }
    }

    if (millis() - startMs > 30000UL) { abort("timeout (no clean oscillation)"); tgtL = tgtR = 0; return; }

    float w = HEADING_TURN_SIGN * amp * (sign == 0 ? +1.0f : (float)sign);
    mix(0.0f, w, tgtL, tgtR);
  }
};
AutoTune autotune;

// Live heading tuning over serial OR Bluetooth: "kp <v>", "kd <v>", "db <v>" + Enter.
// Word-only commands: "tune" (start auto-tune), "stop" (abort auto-tune).
void applyTuningLine(char* buf, uint8_t len) {
  if (len == 0) return;
  buf[len] = 0;
  if (!strncmp(buf, "tune", 4)) { autotune.start();        return; }
  if (!strncmp(buf, "stop", 4)) { autotune.abort("user stop"); return; }
  if (!strncmp(buf, "clrgains", 8)) {   // forget auto-tuned gains, revert to config
    prefs.remove("hdgKp"); prefs.remove("hdgKd");
    hdgKp = HDG_KP; hdgKd = HDG_KD; headingPID.setGains(hdgKp, HDG_KI, hdgKd);
    char m[80];
    snprintf(m, sizeof(m), ">> gains cleared, reverted to config: kp=%.4f kd=%.4f\r\n", hdgKp, hdgKd);
    Serial.print(m); SerialBT.print(m);
    return;
  }
  char cmd[8]; float val;
  if (sscanf(buf, "%7s %f", cmd, &val) == 2) {
    if      (!strcmp(cmd, "kp")) hdgKp = val;
    else if (!strcmp(cmd, "kd")) hdgKd = val;
    else if (!strcmp(cmd, "db")) hdgDeadband = val;
    else if (!strcmp(cmd, "amp")) autotune.amp = val;        // relay magnitude for auto-tune
    else if (!strcmp(cmd, "log")) logHz = (int)val;          // telemetry rate Hz (0 = mute)
    else if (!strcmp(cmd, "mnl")) driver.minDriveL = val;    // left  motor min drive trim
    else if (!strcmp(cmd, "mnr")) driver.minDriveR = val;    // right motor min drive trim
    headingPID.setGains(hdgKp, HDG_KI, hdgKd);
    char reply[110];
    snprintf(reply, sizeof(reply),
             ">> kp=%.4f kd=%.4f db=%.1f amp=%.2f  mnL=%.2f mnR=%.2f  log=%dHz\r\n",
             hdgKp, hdgKd, hdgDeadband, autotune.amp, driver.minDriveL, driver.minDriveR, logHz);
    Serial.print(reply);
    SerialBT.print(reply);
  }
}

void handleSerialTuning() {
  static char buf[40];
  static uint8_t bi = 0;
  while (Serial.available() || SerialBT.available()) {
    char c = Serial.available() ? Serial.read() : SerialBT.read();
    if (c == '\n' || c == '\r') {
      applyTuningLine(buf, bi);
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

  // Auto-tune is only valid while actively holding heading; bail otherwise.
  if (autotune.active && sm.mode() != Mode::HEADING_HOLD) autotune.abort("left HEADING_HOLD");

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
        if (autotune.active) {
          autotune.step(ahrs.deg(), tgtL, tgtR);    // relay-feedback auto-tune drives the wag
        } else {
          float err = headingError(headingSetpoint, ahrs.deg());        // shortest signed error (deg)
          float w;
          if (fabsf(err) < hdgDeadband) { w = 0.0f; headingPID.reset(); }  // close enough: hold quietly
          else { w = HEADING_TURN_SIGN * headingPID.update(err, dt); }     // correct
          float v = 0.5f * (rc.norm(RC_LEFT) + rc.norm(RC_RIGHT));         // optional forward from stick
          mix(v, w, tgtL, tgtR);
        }
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
