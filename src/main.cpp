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
#include "hal/GPS.h"
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
GPS            gps;

// Controllers (used from Phase 3+). Declared now so the structure is set.
PID headingPID, distancePID;

// Heading estimation (Phase 2/3).
IMU         imu;
HeadingAHRS ahrs;
bool        imuOk = false;
float       headingSetpoint = 0;   // captured on entering HEADING_HOLD

// Smart Anchor: position captured on entering ANCHOR (once a fix is available).
double      anchorLat = 0, anchorLon = 0;
float       anchorHeading = 0;     // heading captured at engage (ANCHOR_HEADING hold)
bool        anchorSet = false;
float       anchorDist = 0, anchorBrg = 0;   // live range (m) + bearing-home (deg), for telemetry
// Live-tunable anchor params:
float       ancDeadband = ANCHOR_DEADBAND_M;
float       ancAccMax = ANCHOR_ACC_MAX_M;    // require GPS accuracy <= this to capture/chase
float       posKp = POS_KP, posKd = POS_KD;  // distance-PID gains (anchor return)

// Runtime-tunable heading params (adjust live over serial, no reflash):
//   "kp 0.01"  "kd 0.004"  "db 5"   then Enter.
float hdgKp = HDG_KP, hdgKd = HDG_KD, hdgDeadband = HEADING_DEADBAND_DEG;

// Asymmetric ramp limits (thrust units/sec), live-tunable. "slew" caps accel
// (ramp up) for less wheelspin; "slewdn" caps decel (ramp down) - keep it high
// so the rover stops promptly even when slew is low.
float thrustSlew  = THRUST_SLEW_PER_S;
float thrustDecel = THRUST_DECEL_PER_S;

// Telemetry print rate (Hz), live-adjustable: "log 0" mutes, "log 5" = 5 Hz.
volatile int logHz = TELEMETRY_HZ;

// Count of brief HEADING_HOLD drop-outs masked by the regrab grace window —
// a nonzero, climbing value means an input (bypass pin / RC) is glitching.
volatile uint32_t hhDropouts = 0;

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
  uint32_t dropouts = 0;              // masked HEADING_HOLD glitch count
  bool   gpsFix = false;
  int    gpsSats = 0;
  float  gpsHdop = 99.9f;
  float  gpsAcc = 99.9f;
  double gpsLat = 0, gpsLon = 0;
  bool   anchorOn = false, anchorSet = false;
  float  anchorDist = 0, anchorBrg = 0;
  float  gpsCog = -1;   // course-over-ground (deg); -1 when stationary/invalid
} snap;
portMUX_TYPE snapMux = portMUX_INITIALIZER_UNLOCKED;

float cmdL = 0, cmdR = 0;   // slew-limited outputs (persist across ticks)

// ---------------------------------------------------------------------
//  Telemetry task (core 0) — serial for now; WiFi/BLE later (Phase 7).
// ---------------------------------------------------------------------
void telemetryTask(void*) {
  char line[220];
  for (;;) {
    int hz = logHz;
    if (hz <= 0) { vTaskDelay(pdMS_TO_TICKS(200)); continue; }  // muted
    vTaskDelay(pdMS_TO_TICKS(1000 / hz));
    Snapshot s;
    portENTER_CRITICAL(&snapMux); s = snap; portEXIT_CRITICAL(&snapMux);
    char ancTok[28] = "";
    if (s.anchorOn)
      snprintf(ancTok, sizeof(ancTok), "  anc=%s%.1fm@%.0f",
               s.anchorSet ? "" : "(wait)", s.anchorDist, s.anchorBrg);
    char cogTok[16] = "";
    if (s.gpsCog >= 0) snprintf(cogTok, sizeof(cogTok), " cog=%.0f", s.gpsCog);
    snprintf(line, sizeof(line),
             // batt= omitted - divider not wired yet, see hal/BatteryMonitor.cpp.
             "[%-14s] L=%+.2f R=%+.2f  hdg=%5.1f%s sp=%5.1f  gps=%ds/%.1f%s acc=%.1f%s  link=%s%s  drop=%lu\r\n",
             modeName(s.mode), s.appliedL, s.appliedR, s.heading, cogTok, s.setpoint,
             s.gpsSats, s.gpsHdop, s.gpsFix ? " FIX" : "", s.gpsAcc, ancTok,
             s.linkOk ? "OK" : "LOST",
             s.bypassManual ? "  BYPASS=MANUAL" : "",
             (unsigned long)s.dropouts);
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
  gps.begin();
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
    // Restore mag hard/soft-iron cal from NVS (from `cal compass`), if present.
    if (prefs.isKey("magOffX")) {
      float off[3]   = { prefs.getFloat("magOffX", MAG_OFF[0]),
                         prefs.getFloat("magOffY", MAG_OFF[1]),
                         prefs.getFloat("magOffZ", MAG_OFF[2]) };
      float scale[3] = { prefs.getFloat("magScX", MAG_SCALE[0]),
                         prefs.getFloat("magScY", MAG_SCALE[1]),
                         prefs.getFloat("magScZ", MAG_SCALE[2]) };
      ahrs.setMagCal(off, scale);
      Serial.println("Mag cal: loaded from NVS (cal compass).");
    }
    if (prefs.isKey("hdgOff")) {
      ahrs.setHeadingOffset(prefs.getFloat("hdgOff", 0.0f));
      Serial.printf("Heading offset: %.0f deg (from NVS).\r\n", ahrs.headingOffset());
    }
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

// =====================================================================
//  Drive-floor calibration. Pushes a motor (or both) at an EXACT duty so
//  the breakaway (KICK) and sustain (RUN) floors can be found by eye:
//    cal kick l|r|both -> hold KICK for 0.7 s (does it start from rest?)
//    cal run  l|r|both -> KICK for kickMs, then RUN (does it keep going?)
//    cal max           -> both motors ramp 0->max over 1 s, hold max 2 s
//  kick/run probe exact floors via setRaw(); max runs the full setThrust path
//  (kick + cap). Auto-stops, "stop" aborts. Motors move regardless of arm.
// =====================================================================
struct DriveCal {
  bool active = false, doL = false, doR = false, runTest = false, maxTest = false;
  uint32_t startMs = 0;
  int  savedLog = TELEMETRY_HZ;
  // cal max ramp state
  float    maxCmd = 0;
  bool     atMax = false;
  uint32_t holdStartMs = 0;

  void echo(const char* s) { Serial.print(s); SerialBT.print(s); }

  void start(bool l, bool r, bool runMode) {
    autotune.abort("cal started");
    doL = l; doR = r; runTest = runMode; maxTest = false; active = true;
    startMs = millis(); savedLog = logHz; logHz = 0;
    char m[150];
    snprintf(m, sizeof(m),
      ">> CAL %s start [%s%s] 0.7s  kickL=%.2f kickR=%.2f runL=%.2f runR=%.2f kickMs=%lu. Keep area clear!\r\n",
      runMode ? "RUN" : "KICK", l ? "L" : "", r ? "R" : "",
      driver.kickL, driver.kickR, driver.runL, driver.runR, (unsigned long)driver.kickMs);
    echo(m);
  }

  void startMax() {
    autotune.abort("cal started");
    doL = doR = true; runTest = false; maxTest = true; active = true;
    maxCmd = 0; atMax = false; holdStartMs = 0;
    startMs = millis(); savedLog = logHz; logHz = 0;
    char m[140];
    snprintf(m, sizeof(m),
      ">> CAL MAX start: ramp to full at slew=%.1f, then hold full 2s  maxL=%.2f maxR=%.2f. Keep area clear!\r\n",
      thrustSlew, driver.maxL, driver.maxR);
    echo(m);
  }

  void stop(const char* why) {
    if (!active) return;
    active = false; logHz = savedLog; driver.disable();
    char m[48];
    snprintf(m, sizeof(m), ">> CAL %s\r\n", why);
    echo(m);
  }

  void step() {   // called each control tick while active; drives motors directly
    if (!active) return;
    uint32_t el = millis() - startMs;
    if (maxTest) {
      if (el >= 8000UL) { stop("safety timeout"); return; }   // guard against a stalled ramp
      const float dt = 1.0f / CONTROL_HZ;
      if (!atMax) {
        maxCmd += thrustSlew * dt;                          // ramp at the tuned acceleration
        if (maxCmd >= 1.0f) { maxCmd = 1.0f; atMax = true; holdStartMs = millis(); }
      } else if (millis() - holdStartMs >= 2000UL) {
        stop("done"); return;                              // held full for 2 s
      }
      driver.capActive = true;                             // exercise the cap regardless of mode
      driver.setThrust(maxCmd, maxCmd);                    // full path: kick + scale into [floor, max]
      return;
    }
    if (el >= 700UL) { stop("done"); return; }
    float dl = 0, dr = 0;
    if (doL) dl = (!runTest || el < driver.kickMs) ? driver.kickL : driver.runL;
    if (doR) dr = (!runTest || el < driver.kickMs) ? driver.kickR : driver.runR;
    driver.setRaw(dl, dr);
  }
};
DriveCal drivecal;

// =====================================================================
//  Compass (magnetometer) calibration. While active, motors are held off
//  and you spin the rover by hand through full turns (tilt too if you can).
//  It tracks per-axis raw min/max, prints them live, and on "stop" derives
//  hard-iron offset = (max+min)/2 and soft-iron scale = avgRange/range,
//  applies them live (ahrs.setMagCal) and saves to NVS. Gyro-Z bias is
//  measured automatically at boot (hold still), so it's not part of this.
// =====================================================================
struct CompassCal {
  bool active = false;
  int16_t mn[3], mx[3];
  uint32_t startMs = 0, lastPrint = 0;
  int  savedLog = TELEMETRY_HZ;

  void echo(const char* s) { Serial.print(s); SerialBT.print(s); }

  void start() {
    if (!imuOk) { echo(">> compass cal: no IMU, cannot calibrate.\r\n"); return; }
    autotune.abort("cal started"); drivecal.stop("cal started");
    active = true; startMs = lastPrint = millis(); savedLog = logHz; logHz = 0;
    imu.readMag();
    for (int i = 0; i < 3; ++i) { mn[i] = mx[i] = imu.mRaw[i]; }
    echo(">> COMPASS CAL: motors OFF. Spin the rover SLOWLY through several full\r\n"
         ">> turns (tilt/roll too if you can). Watch the ranges grow; type 'stop'\r\n"
         ">> when they quit changing.\r\n");
  }

  void step() {
    if (!active) return;
    imu.readMag();
    for (int i = 0; i < 3; ++i) {
      if (imu.mRaw[i] < mn[i]) mn[i] = imu.mRaw[i];
      if (imu.mRaw[i] > mx[i]) mx[i] = imu.mRaw[i];
    }
    uint32_t now = millis();
    if (now - lastPrint >= 300) {
      lastPrint = now;
      char m[120];
      snprintf(m, sizeof(m), "   X[%6d..%6d] Y[%6d..%6d] Z[%6d..%6d]\r\n",
               mn[0], mx[0], mn[1], mx[1], mn[2], mx[2]);
      echo(m);
    }
    if (now - startMs > 120000UL) finish("timeout");   // 2 min safety
  }

  void finish(const char* why) {
    if (!active) return;
    active = false; logHz = savedLog;
    float off[3], scale[3], range[3];
    for (int i = 0; i < 3; ++i) { off[i] = 0.5f * (mx[i] + mn[i]); range[i] = 0.5f * (mx[i] - mn[i]); }
    float avg = (range[0] + range[1] + range[2]) / 3.0f;
    for (int i = 0; i < 3; ++i) scale[i] = (range[i] > 1.0f) ? (avg / range[i]) : 1.0f;
    ahrs.setMagCal(off, scale);
    prefs.putFloat("magOffX", off[0]);   prefs.putFloat("magOffY", off[1]);   prefs.putFloat("magOffZ", off[2]);
    prefs.putFloat("magScX", scale[0]);  prefs.putFloat("magScY", scale[1]);  prefs.putFloat("magScZ", scale[2]);
    char m[280];
    snprintf(m, sizeof(m),
      ">> COMPASS CAL %s. Applied + saved to NVS. Paste into config.h to bake:\r\n"
      ">> constexpr float MAG_OFF[3]   = {%.1f, %.1f, %.1f};\r\n"
      ">> constexpr float MAG_SCALE[3] = {%.3f, %.3f, %.3f};\r\n",
      why, off[0], off[1], off[2], scale[0], scale[1], scale[2]);
    echo(m);
  }
};
CompassCal compasscal;

// Live tuning over serial OR Bluetooth (type + Enter). Word commands:
//   tune / stop / clrgains, cal kick|run l|r|both. Value commands: see below.
void applyTuningLine(char* buf, uint8_t len) {
  if (len == 0) return;
  buf[len] = 0;
  if (!strncmp(buf, "tune", 4)) { autotune.start();        return; }
  if (!strncmp(buf, "stop", 4)) { autotune.abort("user stop"); drivecal.stop("aborted"); compasscal.finish("stopped"); return; }
  if (!strncmp(buf, "cal", 3)) {
    char a[8], b[8], c[8] = "";
    int n = sscanf(buf, "%7s %7s %7s", a, b, c);   // "cal" "kick"|"run"|"max"|"compass" ["l"|"r"|"both"]
    if (n >= 2 && !strcmp(b, "compass")) { compasscal.start(); return; }
    if (n >= 2 && !strcmp(b, "max")) { drivecal.startMax(); return; }
    bool runMode = (n >= 2 && !strcmp(b, "run"));
    bool kickMode = (n >= 2 && !strcmp(b, "kick"));
    if (!runMode && !kickMode) {
      const char* u = ">> usage: cal kick|run [l|r|both]  |  cal max  |  cal compass\r\n";
      Serial.print(u); SerialBT.print(u); return;
    }
    bool l = true, r = true;                        // no side given -> both
    if (n >= 3) { l = (!strcmp(c, "l") || !strcmp(c, "both")); r = (!strcmp(c, "r") || !strcmp(c, "both")); }
    drivecal.start(l, r, runMode);
    return;
  }
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
    else if (!strcmp(cmd, "kickl")) driver.kickL = val;      // left  breakaway floor
    else if (!strcmp(cmd, "kickr")) driver.kickR = val;      // right breakaway floor
    else if (!strcmp(cmd, "runl") || !strcmp(cmd, "mnl")) driver.runL = val;  // left  sustain (mnl alias)
    else if (!strcmp(cmd, "runr") || !strcmp(cmd, "mnr")) driver.runR = val;  // right sustain (mnr alias)
    else if (!strcmp(cmd, "kickms")) driver.kickMs = (uint32_t)val;           // kick duration
    else if (!strcmp(cmd, "mxl")) driver.maxL = val;        // left  output cap
    else if (!strcmp(cmd, "mxr")) driver.maxR = val;        // right output cap
    else if (!strcmp(cmd, "slew")) thrustSlew = val;        // accel limit (ramp up)
    else if (!strcmp(cmd, "slewdn")) thrustDecel = val;     // decel limit (ramp down to stop)
    else if (!strcmp(cmd, "ancdb")) ancDeadband = val;      // anchor hold radius (m)
    else if (!strcmp(cmd, "ancacc")) ancAccMax = val;       // max GPS accuracy (m) to capture/chase
    else if (!strcmp(cmd, "pkp")) { posKp = val; distancePID.setGains(posKp, POS_KI, posKd); } // dist P
    else if (!strcmp(cmd, "pkd")) { posKd = val; distancePID.setGains(posKp, POS_KI, posKd); } // dist D
    else if (!strcmp(cmd, "hoff")) { ahrs.setHeadingOffset(val); prefs.putFloat("hdgOff", val); } // compass->true-N trim, saved
    headingPID.setGains(hdgKp, HDG_KI, hdgKd);
    char reply[280];
    snprintf(reply, sizeof(reply),
             ">> kp=%.4f kd=%.4f db=%.1f amp=%.2f  kickL=%.2f kickR=%.2f runL=%.2f runR=%.2f kickMs=%lu  maxL=%.2f maxR=%.2f slew=%.1f slewdn=%.1f"
             "  hoff=%.0f ancdb=%.1f ancacc=%.1f pkp=%.3f pkd=%.3f  log=%dHz\r\n",
             hdgKp, hdgKd, hdgDeadband, autotune.amp,
             driver.kickL, driver.kickR, driver.runL, driver.runR, (unsigned long)driver.kickMs,
             driver.maxL, driver.maxR, thrustSlew, thrustDecel,
             ahrs.headingOffset(), ancDeadband, ancAccMax, posKp, posKd, logHz);
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
  gps.update();   // drain GPS UART each tick
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

  // Advance the heading estimate, and capture the setpoint when we ENGAGE
  // HEADING_HOLD. Re-grab only on a *fresh* engage: if HEADING_HOLD was
  // interrupted only briefly (a glitch on the bypass/RC inputs bouncing the
  // state machine), keep the existing setpoint so the lock doesn't wander.
  if (imuOk) ahrs.update(dt);
  static uint32_t lastHHms = 0;
  uint32_t nowMs = millis();
  if (sm.mode() == Mode::HEADING_HOLD) {
    uint32_t gap = nowMs - lastHHms;
    if (gap > HEADING_REGRAB_MS) {                // absent longer than the grace window => real engage
      headingSetpoint = ahrs.deg();
      headingPID.reset();
    } else if (gap > 25) {                        // back within grace => a brief glitch we just masked
      hhDropouts++;
    }
    lastHHms = nowMs;
  }

  // Anchor capture: on entering an anchor mode, arm capture; then latch the
  // anchor at the first available GPS fix. Leaving anchor clears it.
  static Mode prevAnchorMode = Mode::BOOT;
  bool inAnchor  = (sm.mode() == Mode::ANCHOR || sm.mode() == Mode::ANCHOR_HEADING);
  bool wasAnchor = (prevAnchorMode == Mode::ANCHOR || prevAnchorMode == Mode::ANCHOR_HEADING);
  if (inAnchor && !wasAnchor) {                    // fresh engage
    anchorSet = false;
    anchorHeading = imuOk ? ahrs.deg() : 0.0f;
    headingPID.reset(); distancePID.reset();
  }
  if (inAnchor && !anchorSet && gps.hasFix() && gps.accM() <= ancAccMax) {  // latch at first GOOD fix
    anchorLat = gps.lat(); anchorLon = gps.lon(); anchorSet = true;
  }
  if (!inAnchor) anchorSet = false;
  prevAnchorMode = sm.mode();

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
      if (anchorSet && gps.hasFix() && imuOk && gps.accM() <= ancAccMax) {
        double d   = TinyGPSPlus::distanceBetween(gps.lat(), gps.lon(), anchorLat, anchorLon);
        double brg = TinyGPSPlus::courseTo(gps.lat(), gps.lon(), anchorLat, anchorLon); // bearing home
        anchorDist = (float)d; anchorBrg = (float)brg;
        if (d < ancDeadband) {
          // At anchor. ANCHOR_HEADING also holds the heading captured on engage.
          if (sm.mode() == Mode::ANCHOR_HEADING) {
            float e = headingError(anchorHeading, ahrs.deg());
            float w = (fabsf(e) < hdgDeadband) ? 0.0f : HEADING_TURN_SIGN * headingPID.update(e, dt);
            mix(0.0f, w, tgtL, tgtR);
          } else { tgtL = 0; tgtR = 0; headingPID.reset(); distancePID.reset(); }
        } else {
          // Turn toward bearing-home; drive forward scaled by alignment (cos),
          // so it arcs in and never powers away. HAL caps (mxl/mxr) limit speed.
          float herr  = headingError((float)brg, ahrs.deg());
          float w     = HEADING_TURN_SIGN * headingPID.update(herr, dt);
          float fwd   = constrain(distancePID.update((float)d, dt), 0.0f, 1.0f);
          float align = cosf(herr * 0.01745329f);
          float v     = (align > 0.0f) ? fwd * align : 0.0f;  // no forward when facing away
          mix(v, w, tgtL, tgtR);
        }
      } else {
        tgtL = 0; tgtR = 0;   // no fix / no anchor / no IMU -> inert (safe)
      }
      break;

    case Mode::BOOT:
    case Mode::DISARMED:
    case Mode::FAILSAFE:
    default:
      tgtL = 0; tgtR = 0;
      break;
  }

  // ---- Output ----
  bool armed = (sm.mode() == Mode::MANUAL || sm.mode() == Mode::HEADING_HOLD ||
                sm.mode() == Mode::ANCHOR || sm.mode() == Mode::ANCHOR_HEADING);
  if (compasscal.active) {
    compasscal.step();           // manual-spin mag calibration: motors OFF for safety
    driver.disable(); cmdL = cmdR = 0;
  } else if (drivecal.active) {
    drivecal.step();             // bench calibration drives motors directly via setRaw()
    cmdL = cmdR = 0;             // hold slew state neutral so normal control resumes cleanly
  } else {
    // asymmetric slew: gentle accel, quick stop
    float accStep = thrustSlew * dt, decStep = thrustDecel * dt;
    cmdL = slewAsym(tgtL, cmdL, accStep, decStep);
    cmdR = slewAsym(tgtR, cmdR, accStep, decStep);
    if (armed) {
      driver.capActive = (sm.mode() != Mode::MANUAL);   // cap autonomous modes only; MANUAL = full stick
      driver.setThrust(cmdL, cmdR);
    } else { driver.disable(); cmdL = cmdR = 0; }
  }

  digitalWrite(PIN_STATUS_LED, armed ? HIGH : (millis() / 250) & 1); // solid=armed, blink=safe

  // ---- Publish snapshot ----
  portENTER_CRITICAL(&snapMux);
  snap.mode = sm.mode(); snap.left = cmdL; snap.right = cmdR;
  snap.appliedL = driver.lastL; snap.appliedR = driver.lastR;
  snap.battV = battery.volts(); snap.linkOk = linkOk; snap.bypassManual = bypassManual;
  snap.armReq = in.armRequest; snap.modeSel = in.modeSelect;
  snap.heading = imuOk ? ahrs.deg() : 0.0f;
  snap.setpoint = headingSetpoint;
  snap.dropouts = hhDropouts;
  snap.gpsFix = gps.hasFix(); snap.gpsSats = gps.sats(); snap.gpsHdop = gps.hdop();
  snap.gpsAcc = gps.accM();
  snap.gpsLat = gps.lat(); snap.gpsLon = gps.lon();
  snap.anchorOn = inAnchor; snap.anchorSet = anchorSet;
  snap.anchorDist = anchorDist; snap.anchorBrg = anchorBrg;
  snap.gpsCog = gps.course();
  portEXIT_CRITICAL(&snapMux);
}
