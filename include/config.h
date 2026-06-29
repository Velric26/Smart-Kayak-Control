#pragma once
// =====================================================================
//  config.h  -  Single source of truth for pins and tunables.
//  Pin map matches the optimized layout in the architecture doc (§1.1).
//  ESC signals live on GPIO25/26 (clean, non-strapping) — the same pins the
//  rover used for the L298N enables before it migrated to dual ESCs.
// =====================================================================
//
// =====================================================================
//  CONSOLE REFERENCE  (USB or Bluetooth SmartKayak-*; type + Enter, CRLF)
// ---------------------------------------------------------------------
//  TUNABLE VALUES  ("<cmd> <value>"; each entry echoes the full set):
//    kp <v>      heading P gain
//    kd <v>      heading D gain
//    db <v>      heading deadband (deg) - the "calming" knob
//    amp <v>     relay amplitude used by `cal`/auto-tune
//    slew <v>    accel limit (ramp up, units/sec) - lower = gentler launch
//    slewdn <v>  decel limit (ramp down to stop, units/sec) - keep high
//    kickl <v>   left  breakaway floor      kickr <v>  right breakaway floor
//    runl  <v>   left  sustain  floor       runr  <v>  right sustain  floor
//                 (mnl/mnr are aliases for runl/runr)
//    kickms <v>  breakaway kick duration (ms)
//    mxl <v>     left  output cap           mxr <v>    right output cap
//                 (caps apply in autonomous modes only; MANUAL = full)
//    ancdb <m>   anchor hold radius (m)   ancacc <m>  max GPS accuracy to chase
//    pkp <v>     distance-PID P gain (anchor return)   pkd <v>  D gain
//    hoff <deg>  compass->true-north heading trim (for ANCHOR; saved to NVS)
//    fuse <v>    heading filter gyro-trust (0..0.999); lower = snappier, noisier
//    log <v>     telemetry rate Hz (0 = mute)
//
//  WORD COMMANDS:
//    tune        start relay heading auto-tune (HEADING_HOLD + IMU only)
//    stop        abort tune / drive-cal; FINISH+save compass cal
//    clrgains    forget NVS heading gains, revert to config defaults
//
//  CALIBRATION ROUTINES:
//    cal kick l|r|both   push at KICK from standstill (0.7 s)
//    cal run  l|r|both   kick to start, then hold RUN (0.7 s)
//    cal max             both ramp to full at slew, hold full 2 s (find spin)
//    cal compass         motors off; spin by hand -> saves mag hard/soft iron
//
//  PERSISTED IN NVS: heading gains (from `tune`) and mag cal (from
//  `cal compass`). Everything else is RAM-only until baked into this file.
// =====================================================================

// ---------------------------------------------------------------------
//  Platform sanity check (set by platformio.ini build flags)
// ---------------------------------------------------------------------
#if !defined(PLATFORM_MULE) && !defined(PLATFORM_KAYAK)
#  error "Build with -e mule or -e kayak (see platformio.ini)"
#endif

// ---------------------------------------------------------------------
//  Pin map  (ESP32 DevKit V1 / WROOM)
// ---------------------------------------------------------------------
// ESC signal pins (servo PWM, 1000-2000 us). One wire per ESC; direction is
// encoded in the pulse width. DO NOT move these (clean, non-strapping).
constexpr int PIN_MOTOR_PWM_L = 25;
constexpr int PIN_MOTOR_PWM_R = 26;

// GPIO 27/14/18/19 are now FREE (former L298N direction pins, retired with the
// H-bridge). Available for future use (e.g. lights, aux, 3PDT wiring).

// I2C bus -- 9-DOF IMU (GY-801-type) shares this bus:
//   L3G4200D gyro 0x69 | LSM303DLHC accel 0x19, mag 0x1E  (no barometer on this board)
constexpr int PIN_I2C_SDA = 21;
constexpr int PIN_I2C_SCL = 22;

// GPS (NEO-8M) on UART2
constexpr int PIN_GPS_RX = 16;   // ESP32 RX  <- GPS TX
constexpr int PIN_GPS_TX = 17;   // ESP32 TX  -> GPS RX

// RC inputs (parallel high-impedance tap on the DS600 channels).
// Input-only pins (34-39) used where possible to preserve output pins.
// DS600 buttons: CH3/4/5 LATCH (hold state), CH6 is MOMENTARY.
constexpr int PIN_RC_LEFT = 35;  // DS600 CH1 mixed-Left  thrust (stick)
constexpr int PIN_RC_RIGHT= 36;  // DS600 CH2 mixed-Right thrust (stick)
constexpr int PIN_RC_ARM  = 39;  // ARM  : CH5 LATCHING button (held = armed)
constexpr int PIN_RC_MODE = 4;   // MODE : CH6 MOMENTARY button (press = cycle)

// Battery sense (ADC1, input-only)
constexpr int PIN_BATT_SENSE = 34;

// 3PDT bypass position sense (third pole). RESERVED — not read in firmware
// yet (the bypass logic was removed; a floating pin here was glitching the
// state machine). Re-add BypassSense + the StateMachine level-0 branch when
// the 3PDT is installed. See docs/architecture.md §1.6.
constexpr int PIN_BYPASS_SENSE = 13;

// Status LED (onboard)
constexpr int PIN_STATUS_LED = 2;

// ---------------------------------------------------------------------
//  RC signal characteristics (standard servo PWM)
// ---------------------------------------------------------------------
constexpr int   RC_US_MIN     = 1000;
constexpr int   RC_US_NEUTRAL = 1500;
constexpr int   RC_US_MAX     = 2000;
constexpr int   RC_US_VALID_LO= 800;   // reject pulses outside [lo,hi] as noise
constexpr int   RC_US_VALID_HI= 2200;
constexpr float RC_DEADBAND   = 0.06f; // normalized stick deadband
constexpr uint32_t RC_TIMEOUT_US = 400000UL; // 400 ms with no fresh pulse = signal lost

// ---------------------------------------------------------------------
//  Loop timing
// ---------------------------------------------------------------------
constexpr int CONTROL_HZ   = 100;  // fast control loop
constexpr int TELEMETRY_HZ = 5;    // serial/telemetry print rate

// ---------------------------------------------------------------------
//  Bluetooth Classic (SPP) telemetry mirror. Lets you watch logs and use
//  live tuning ("kp"/"kd"/"db") off-USB while motors run on battery power.
//  Pair from a phone/PC with a serial-Bluetooth terminal app.
// ---------------------------------------------------------------------
#if defined(PLATFORM_KAYAK)
constexpr const char* BT_DEVICE_NAME = "SmartKayak-Kayak";
#else
constexpr const char* BT_DEVICE_NAME = "SmartKayak-TestMule";
#endif

// ---------------------------------------------------------------------
//  Battery (2S Li-ion). Tune divider ratio to your resistors (§1.4).
//  Vbatt = Vadc * BATT_DIVIDER. Example 100k/56k -> ~2.79.
// ---------------------------------------------------------------------
constexpr float BATT_DIVIDER       = 2.79f;
constexpr float BATT_WARN_V        = 6.6f;  // 3.30 V/cell
// !!! TEMP for bench: battery failsafe DISABLED (no divider wired on GPIO34).
// !!! RESTORE to 6.0f (3.00 V/cell) once the battery sense divider is in place.
constexpr float BATT_CRITICAL_V    = 0.0f;

// ---------------------------------------------------------------------
//  Output shaping
// ---------------------------------------------------------------------
constexpr float THRUST_SLEW_PER_S  = 0.5f;   // MANUAL accel limit, units/sec (ramp UP) - live "slew"
constexpr float THRUST_DECEL_PER_S = 20.0f;  // decel limit, units/sec (ramp DOWN to stop) - live "slewdn"
// Autonomous modes (HEADING_HOLD / ANCHOR) get their own, faster accel limit so
// heading corrections aren't throttled by the gentle MANUAL launch ramp. Live "slewa".
constexpr float THRUST_SLEW_AUTO_PER_S = 4.0f;

// ---------------------------------------------------------------------
//  Motor direction trim. Flip a flag if a POSITIVE command spins that
//  side the wrong way. Applied in the HAL, so it carries to the ESC too,
//  and the console keeps showing the logical command (positive = forward).
//  Symptom guide: forward/reverse swapped but turning correct -> set BOTH.
//  After any change, re-check a forward push AND a turn.
// ---------------------------------------------------------------------
constexpr bool MOTOR_L_INVERT = true;
constexpr bool MOTOR_R_INVERT = true;

// Two-tier minimum drive (per side). Brushed gearmotors have static friction
// HIGHER than kinetic, so a stopped motor needs a brief "breakaway kick" to
// start, then a lower "run" floor sustains it. Commands remap so 0 stays 0 and
// any non-zero command scales into [floor, 1], where floor = KICK for the first
// KICK_MS after breakaway, then RUN. Brushed motors rarely match -> per side.
//
// Calibrate live (no reflash):
//   set values: "kickl/kickr <v>", "runl/runr <v>", "kickms <v>"
//   test:       "cal kick l|r|both"  -> push at KICK from standstill (0.7 s)
//               "cal run  l|r|both"  -> kick to start, then hold RUN (0.7 s)
//               "cal max"            -> both ramp to full at slew, hold full 2 s
// Bake the winners here. (mnl/mnr remain aliases for runl/runr.)
constexpr float MOTOR_KICK_L = 0.40f;   // left  breakaway floor (static friction)
constexpr float MOTOR_KICK_R = 0.42f;   // right breakaway floor
constexpr float MOTOR_RUN_L  = 0.20f;   // left  sustain floor (kinetic)
constexpr float MOTOR_RUN_R  = 0.22f;   // right sustain floor
constexpr uint32_t MOTOR_KICK_MS = 50; // how long the breakaway kick holds

// Per-side output cap: a full command maps to this, not 1.0, so commands scale
// into [floor, max]. Tames over-aggressive correction. Live: "mxl"/"mxr <v>".
constexpr float MOTOR_MAX_L = 0.70f;    // left  max duty
constexpr float MOTOR_MAX_R = 0.70f;    // right max duty

// ---------------------------------------------------------------------
//  Magnetometer calibration (from a diag_calib tumble on this board).
//  Corrected axis = (raw - MAG_OFF) * MAG_SCALE.
//  Re-run diag_calib and update these if the IMU is remounted or the
//  board's nearby metal changes (e.g. final kayak install).
// ---------------------------------------------------------------------
constexpr float MAG_OFF[3]   = {-89.0, -152.0, 0.0};  // hard-iron center
constexpr float MAG_SCALE[3] = {0.891, 0.894, 1.317}; // soft-iron scale

// ---------------------------------------------------------------------
//  Heading fusion (complementary filter: gyro-Z + magnetic compass).
// ---------------------------------------------------------------------
constexpr float GYRO_YAW_SIGN      = +1.0f; // flip to -1 if fused heading runs opposite the compass
constexpr float HEADING_FUSE_ALPHA = 0.95f; // gyro trust per step (higher = smoother, slower mag pull)
constexpr float HEADING_TURN_SIGN  = +1.0f; // flip to -1 if HEADING_HOLD turns AWAY from the setpoint
constexpr float HEADING_DEADBAND_DEG = 4.0f; // within this error, hold (no turn) - stops setpoint hunting
// Grace window: if HEADING_HOLD drops out for less than this, keep the existing
// setpoint instead of re-grabbing the current heading. Masks brief RC-link
// glitches (FAILSAFE flicker) that would otherwise bounce the state machine
// and move the setpoint with no operator input.
constexpr uint32_t HEADING_REGRAB_MS = 750;
// Relay auto-tune tolerates HEADING_HOLD dropping out for up to this long (brief
// RC glitches during the hard wag) before it aborts — lets a tune finish through
// a momentary disarm instead of bailing on a single bad tick.
constexpr uint32_t AUTOTUNE_GRACE_MS = 750;

// ---------------------------------------------------------------------
//  Control gains -- placeholders. Tune on the test mule, RE-TUNE on the water.
//  Expect kayak gains to differ greatly (inertia + disturbance dominated).
// ---------------------------------------------------------------------
#if defined(PLATFORM_KAYAK)
  constexpr float HDG_KP = 0.015f, HDG_KI = 0.0f,   HDG_KD = 0.004f;
  constexpr float POS_KP = 0.30f,  POS_KI = 0.02f,  POS_KD = 0.0f;
#else // PLATFORM_MULE
  constexpr float HDG_KP = 0.0007f, HDG_KI = 0.0f, HDG_KD = 0.0006f;
  // POS gain acts on RANGE IN METERS: at this kp, forward saturates ~1/kp metres
  // out (0.30 => full-forward when >~3 m away). Live-tunable: "pkp"/"pkd".
  constexpr float POS_KP = 0.40f,   POS_KI = 0.0f, POS_KD = 0.0f;
#endif

// ---------------------------------------------------------------------
//  Smart Anchor / position-hold (Phase 5). On engaging ANCHOR we capture
//  the current lat/lon; while outside the deadband the rover turns to the
//  bearing-home (heading loop) and drives forward (distancePID on range),
//  scaled by cos(heading error) so it arcs in and never powers away from
//  home. Forward thrust is capped by the HAL output caps (MOTOR_MAX_L/R) in
//  autonomous modes — no separate anchor cap. Live-tunable: "ancdb <m>".
// ---------------------------------------------------------------------
constexpr float ANCHOR_DEADBAND_M = 1.0f;  // inside this radius => hold (tight; watch vs accM)
// Only capture/chase the anchor while horizontal accuracy (GST) is at least
// this good — keeps us from latching or chasing a noisy fix. Live: "ancacc".
constexpr float ANCHOR_ACC_MAX_M  = 3.0f;  // require accM <= this (metres)
