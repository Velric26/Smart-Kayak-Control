# Smart Kayak Thruster Control

ESP32 differential-thrust control for a fishing kayak (twin underwater thrusters),
developed and validated on a 3D-printed land rover ("the mule") first, then migrated
to the kayak with minimal change. Goal feature set: manual RC drive, Heading Lock,
GPS Smart Anchor / position-hold, waypoints.

Owner: Ricardo. Dev host: Windows + PowerShell. Toolchain: VS Code + PlatformIO (Arduino framework).

Full design rationale lives in `docs/architecture.md` — read it before proposing
architectural changes. This file is the operational quick-reference; `include/config.h`
is the source of truth for pins and tunables.

## Golden rules
- **Validate-first, incremental.** Every change is flashed and confirmed on real hardware
  before moving on. Do not batch up multiple unverified behavioral changes.
- **The HAL is the migration key.** Mule and kayak differ ONLY in the motor driver
  (`L298N_Driver` vs `ESC_Driver`) behind `MotorDriver`. Keep platform-specific code
  out of control/estimation/statemachine layers.
- **Safe state is neutral.** Any fault, disarm, or RC loss => motors to neutral/coast.
- The mule validates *control behavior*, not water dynamics. Tilt comp, drag, and
  inertia are deferred to the kayak phase.

## Hardware (REAL, bench-confirmed — differs from generic assumptions)
- MCU: ESP32 DevKit V1 (WROOM).
- Mule actuation: L298N H-bridge + 2 brushed DC motors + caster. Power: 2S 18650 (7.4–8.4V).
- Kayak actuation (FUTURE, not yet acquired): 2x bidirectional ESC + 3PDT manual-override switch.
- RC: HotRC DS600 / ASPIQUEEN A300 (same unit, rebranded). **TX does the differential
  mixing**; ESP32 receives already-mixed L/R. Buttons: Ch3/4/5 **latching**, Ch6 **momentary**.
- IMU: GY-801-type 9-DOF module:
  - Gyro = **L3G4200D** (I2C 0x69, WHO_AM_I 0xD3) — NOT an L3GD20.
  - Accel = LSM303DLHC (0x19), Mag = LSM303DLHC (0x1E).
  - **No barometer** on this module. Ignore any BMP180 references.
  - Driver quirks handled in `estimation/IMU.cpp`: L3G4200D needs 0x80 auto-increment bit;
    DLHC mag is X-Z-Y order, big-endian, no auto-increment.

## Pin map
Authoritative copy is in `include/config.h`. Key points:
- Motor PWM L=GPIO25, R=GPIO26 — these become the ESC signal pins later (wire stays, only
  firmware reconfigures). Kept off strapping pins so future ESC signal is glitch-free at boot.
- L298N direction: IN1=27, IN2=14, IN3=18, IN4=19 (freed when swapping to ESC).
- I2C: SDA=21, SCL=22. GPS UART2: RX=16, TX=17 (reserved).
- RC in: mixed-L=35, mixed-R=36 (VP), Ch5/ARM=39 (VN, latching), Ch6/MODE=4 (momentary).
- Battery sense=34 (ADC1). 3PDT bypass sense=13 (reserved). Status LED=2.

## Build & flash (PlatformIO)
Run from the project root. `pio` is on PATH (`%USERPROFILE%\.platformio\penv\Scripts`).
- Mule build:   `pio run -e mule -t upload`   then  `pio device monitor`
- Kayak build:  `pio run -e kayak -t upload`   (adds ESP32Servo dep)
- Diagnostics (each is its own env): `diag_i2c`, `diag_imu`, `diag_calib`, `diag_heading`, `diag_fused`.
  e.g. `pio run -e diag_i2c -t upload`
Env selection in `platformio.ini` swaps the driver via `build_src_filter` + `DRIVER_ESC` flag.

## Live tuning (no reflash)
Heading controller is tunable over the serial monitor while running. Type + Enter:
- `kp <v>` `kd <v>` `db <v>`  (proportional, derivative, deadband in deg). It echoes the new values.
Telemetry line shows `L=cmd>applied R=cmd>applied` — left of `>` is the controller command,
right is the post-min-drive value the motor actually receives.

## Current state (as of this handoff)
- Phases 0–2 complete: build/flash, dual-core FreeRTOS (control on core1 @100Hz, telemetry core0),
  state machine, RC link + latching arm + momentary mode-cycle, RC-loss & battery failsafes,
  manual differential drive (both motors inverted), IMU bring-up, mag calibration, fused heading.
- Phase 3 (Heading Lock) closed loop WORKS — correct sign, returns to setpoint.
- **In progress / open:** tuning heading-hold to be less aggressive (added error deadband +
  softened kp + live tuning). Mid-bench-debug: on the rover, HEADING_HOLD pivots-in-place stall
  out (motors whine, no rotation) when powered from **USB** — insufficient voltage/current + the
  L298N's ~2V drop, worst case being a stationary pivot. **Next test: run motors off the 2S
  pack** (USB to ESP32 for monitor, battery to L298N VS, common ground). Expect real rotation
  there, then revisit `MOTOR_MIN_DRIVE`.

## Provisional / "do not forget" config values
- `MOTOR_MIN_DRIVE` currently bumped to ~0.6 while chasing the no-move issue; likely revert
  toward 0.5 (or lower) once on battery, and lean on the deadband for calming.
- `BATT_CRITICAL_V = 0.0f` => **battery failsafe DISABLED** for bench. Restore to ~6.0f once the
  voltage divider is wired. `BATT_DIVIDER = 2.79` (R1=100k/R2=56k -> GPIO34). Treat the on-screen
  `batt=` value as unreliable until the divider is confirmed wired.
- Calibrated/measured constants baked in `config.h`: MAG_OFF, MAG_SCALE (from tumble cal),
  gyro/heading signs, HEADING_FUSE_ALPHA=0.98, motor inverts both true.

## Pending hardware
- 3D-print the second-floor IMU mount (away from motor currents), then **re-run mag calibration**.
- Wire the battery divider; then restore the battery failsafe.
- Acquire 3PDT switch + ESCs for the kayak migration.

## Roadmap (after current tuning)
Tilt-compensated heading (for water) -> GPS (TinyGPSPlus) -> Smart Anchor / position-hold ->
WiFi telemetry & runtime tuning -> ESC swap on the mule -> kayak migration.
See `docs/architecture.md` §10–§11.

## Working in this repo
- This is now a git repo: make a commit after each validated change instead of re-zipping.
- Brace-check new C++ files and keep platform code behind the HAL.
- Don't reintroduce L3GD20 / BMP180 assumptions.
