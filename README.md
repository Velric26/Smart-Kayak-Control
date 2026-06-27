# Smart Kayak Thruster Control System — Firmware

ESP32 differential-thrust control, developed and validated on a 3D-printed land
rover ("the mule") and migrated to a twin-thruster fishing kayak with minimal
change. One codebase builds for both platforms via PlatformIO environments; the
only platform-specific code is the motor driver behind the `MotorDriver` HAL.

Goal feature set: manual RC drive, Heading Lock, GPS Smart Anchor / position-hold,
waypoints. `CLAUDE.md` is the living operational quick-reference; `include/config.h`
is the source of truth for pins/tunables (its header has the full console command
reference); `docs/architecture.md` has the design rationale.

## Build & flash

```bash
# Development rover (dual bidirectional ESC)
pio run -e mule -t upload
pio device monitor                 # 115200 baud

# Final kayak — same source + driver, just the kayak gain set
pio run -e kayak -t upload

# Diagnostics (each its own standalone sketch/env)
pio run -e diag_i2c -t upload      # I2C bus scan
pio run -e diag_imu                # raw IMU
pio run -e diag_calib              # gyro bias + mag hard/soft-iron
pio run -e diag_heading            # flat magnetic heading
pio run -e diag_fused              # compass vs gyro-fused heading
pio run -e diag_gps                # NEO-8M NMEA readout (mirrored over Bluetooth)
pio run -e diag_gps_bridge         # ESP32 as a USB<->GPS bridge for gps_config.py
```

Both targets run the same dual-ESC driver; they differ only in the gain set
selected by `PLATFORM_MULE`/`PLATFORM_KAYAK` in `platformio.ini` (plus BT name
and GPS dynamic model).

## Structure

```
include/config.h            Pin map + tunables + CONSOLE REFERENCE (single source of truth)
src/
  main.cpp                  setup() + control loop (core 1, 100 Hz) + telemetry task (core 0)
  hal/                      Hardware abstraction — the migration boundary
    MotorDriver.h             abstract setThrust(); shared two-tier drive shaping + output cap
    ESC_Driver.*              dual bidirectional ESC (ESP32Servo, 1000-2000 us) — rover + kayak
    RCReceiver.*              parallel interrupt capture of the DS600 channels
    BatteryMonitor.*          2S sense (disabled in firmware until the divider is wired)
    GPS.*                     NEO-8M reader (TinyGPSPlus) + GST horizontal-accuracy estimate
  estimation/
    IMU.*                     register-level L3G4200D gyro + LSM303DLHC accel/mag
    Heading.*                 complementary-filter fused heading + runtime mag cal + hoff trim
  control/
    PID.h                     anti-windup PID + shortest-path heading-error helper
    Mixer.h                   differential mix + asymmetric slew (gentle up, quick stop)
  statemachine/
    StateMachine.*            modes, arming, safety precedence
  diag/                       standalone bring-up sketches (see build envs above)
tools/
  gps_config.py             one-shot UBX configurator for the NEO-8M (5 Hz, SBAS, GST, 38400)
  bt_console.py             live telemetry + tuning console over Bluetooth, with /trim and /align
```

## What works now

- **Dual-core FreeRTOS:** control on core 1 @100 Hz, telemetry on core 0.
- **Manual differential drive** from the DS600 (TX does the mixing), sticks-centered
  arming, RC-loss → FAILSAFE → neutral.
- **Two-tier drive shaping** (in the HAL, carries to the ESC): per-side breakaway
  KICK then sustain RUN floor, per-side output cap (autonomous modes only),
  asymmetric slew. Calibrate by eye with `cal kick|run|max`.
- **Heading Lock** (IMU fused heading + PD loop) with relay auto-tune (`tune`),
  deadband, and a glitch-stable setpoint.
- **In-firmware compass calibration** (`cal compass`) — saved to NVS, auto-loaded.
- **GPS** (NEO-8M, configured to 5 Hz + SBAS + GST via `tools/gps_config.py`) with a
  live horizontal-accuracy estimate.
- **Smart Anchor / position-hold:** capture lat/lon on engage, return home via the
  distance PID (throttle) + heading loop (bearing), gated on GPS accuracy.
- **Bluetooth telemetry + live tuning** (pairs as `SmartKayak-Mule`/`-Kayak`); all
  values tunable at runtime, heading gains and mag cal persisted in NVS.

See `CLAUDE.md` → "Current state" for the authoritative status and open items.

## Live tuning & calibration

Over USB **or** Bluetooth — type a command + Enter (CRLF). The full list lives at
the top of `include/config.h`. Highlights: heading `kp/kd/db`; drive floors
`kickl/kickr/runl/runr/kickms`; output caps `mxl/mxr`; motion `slew/slewdn`;
anchor `ancdb/ancacc/pkp/pkd`; compass `hoff`; `log <hz>`. Routines: `tune`,
`cal kick|run|max`, `cal compass`.

`tools/bt_console.py COM5` gives a console with `/trim` (measure heading drift
across a `cal max` run to balance the motors) and `/align` (capture `hdg` vs GPS
`cog` to derive the `hoff` compass→true-north trim).

## Pending hardware

- Wire the battery divider (`BATT_DIVIDER`, R1=100k/R2=56k → GPIO34), then re-enable
  `BatteryMonitor` and the `batt=` telemetry field.
- Acquire the 3PDT manual-override switch for the kayak. The 3PDT bypass logic
  was **removed from firmware** (a floating GPIO13 glitched the state machine);
  GPIO13 is reserved and the design is kept in `docs/architecture.md` §1.6.
- The rover already runs the dual ESCs — after the swap, re-verify
  `MOTOR_L/R_INVERT` and re-tune the drive floors (`cal kick/run/max`), since the
  ESC deadband differs from the old H-bridge.

## Roadmap

GPS bring-up ✓ → Smart Anchor ✓ → ESC migration on the rover ✓ →
tilt-compensated heading (for water) → waypoints → WiFi telemetry & tuning →
kayak migration.

## Notes / verify before relying on it

- The board must be WROOM (GPIO16/17 free for the GPS); WROVER ties those to PSRAM.
- The DS600 must output per-channel servo PWM (not a single PPM/SBUS stream).
- Power the NEO-8M from the ESP32 **3V3** rail, not the 2S pack (its LDO maxes ~6 V).
- If the GPS is factory-reset it reverts to 9600 baud — re-run `tools/gps_config.py`
  (via `diag_gps_bridge`, no USB-UART needed) and it returns to 38400.
- Bidirectional ESCs usually need a neutral arming hold on power-up — already held
  at neutral in `ESC_Driver::begin()`.
