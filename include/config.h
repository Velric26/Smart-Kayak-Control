#pragma once
// =====================================================================
//  config.h  -  Single source of truth for pins and tunables.
//  Pin map matches the optimized layout in the architecture doc (§1.1).
//  Motor PWM lives on GPIO25/26 (clean, non-strapping) so the same two
//  pins serve the L298N enables now and the ESC signals later.
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
// Motor PWM  -- ENA/ENB now, ESC L/R signal later. DO NOT move these.
constexpr int PIN_MOTOR_PWM_L = 25;
constexpr int PIN_MOTOR_PWM_R = 26;

// L298N direction pins -- freed when the ESC arrives (ESC encodes
// direction in pulse width, so these become spare GPIOs).
constexpr int PIN_DIR_IN1 = 27;  // left  fwd/rev
constexpr int PIN_DIR_IN2 = 14;
constexpr int PIN_DIR_IN3 = 18;  // right fwd/rev
constexpr int PIN_DIR_IN4 = 19;

// I2C bus -- 10-DOF IMU module shares this bus (all addresses distinct):
//   L3GD20(H) gyro 0x6B(/0x6A) | LSM303DLHC accel 0x19, mag 0x1E | BMP180 0x77
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

// 3PDT bypass position sense (third pole). Unconnected until the switch
// is installed; INPUT_PULLUP makes an unwired pin read HIGH == AUTO.
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
//  Battery (2S Li-ion). Tune divider ratio to your resistors (§1.4).
//  Vbatt = Vadc * BATT_DIVIDER. Example 100k/56k -> ~2.79.
// ---------------------------------------------------------------------
constexpr float BATT_DIVIDER       = 2.79f;
constexpr float BATT_WARN_V        = 6.6f;  // 3.30 V/cell
constexpr float BATT_CRITICAL_V    = 6.0f;  // 3.00 V/cell -> failsafe

// ---------------------------------------------------------------------
//  Output shaping
// ---------------------------------------------------------------------
constexpr float THRUST_SLEW_PER_S  = 4.0f;  // max change in thrust units/sec (smoothness)

// ---------------------------------------------------------------------
//  Motor direction trim. Flip a flag if a POSITIVE command spins that
//  side the wrong way. Applied in the HAL, so it carries to the ESC too,
//  and the console keeps showing the logical command (positive = forward).
//  Symptom guide: forward/reverse swapped but turning correct -> set BOTH.
//  After any change, re-check a forward push AND a turn.
// ---------------------------------------------------------------------
constexpr bool MOTOR_L_INVERT = true;
constexpr bool MOTOR_R_INVERT = true;

// ---------------------------------------------------------------------
//  Control gains -- placeholders. Tune on the mule, RE-TUNE on the water.
//  Expect kayak gains to differ greatly (inertia + disturbance dominated).
// ---------------------------------------------------------------------
#if defined(PLATFORM_KAYAK)
  constexpr float HDG_KP = 0.015f, HDG_KI = 0.0f,   HDG_KD = 0.004f;
  constexpr float POS_KP = 0.30f,  POS_KI = 0.02f,  POS_KD = 0.0f;
#else // PLATFORM_MULE
  constexpr float HDG_KP = 0.020f, HDG_KI = 0.0f,   HDG_KD = 0.002f;
  constexpr float POS_KP = 0.50f,  POS_KI = 0.0f,   POS_KD = 0.0f;
#endif
