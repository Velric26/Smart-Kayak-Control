# Smart Kayak Thruster Control System — Firmware

ESP32 firmware for a differential-thrust control system, developed on a land
rover ("mule") and migrated to a twin-thruster fishing kayak with minimal
change. One codebase builds for both platforms via PlatformIO environments.

## Build & flash

```bash
# Development rover (L298N H-bridge)
pio run -e mule
pio run -e mule -t upload
pio device monitor -e mule         # 115200 baud

# Final kayak (dual ESC) — same source, different target
pio run -e kayak -t upload
```

The build target is selected entirely by flags in `platformio.ini`
(`DRIVER_L298N` / `DRIVER_ESC`, `PLATFORM_MULE` / `PLATFORM_KAYAK`). The only
platform-specific line in the codebase is the HAL include in `main.cpp`.

## Structure

```
include/config.h            Pin map + tunables (single source of truth)
src/
  main.cpp                  setup() + control loop (core 1) + telemetry task (core 0)
  hal/                      Hardware abstraction (the migration boundary)
    MotorDriver.h             abstract: setThrust(-1..+1)
    L298N_Driver.*            mule (analogWrite + direction pins)
    ESC_Driver.*              kayak (ESP32Servo, 1000-2000 us)
    RCReceiver.*              parallel interrupt capture of DS600 channels
    BatteryMonitor.*          2S Li-ion sense + thresholds
    BypassSense.h             3PDT position sense (GPIO13, stubbed until wired)
  control/
    PID.h                     anti-windup PID + heading-error helper
    Mixer.h                   differential mix + slew limiter
  statemachine/
    StateMachine.*            modes, arming, safety precedence
```

## Pin map

See `include/config.h`. Motor PWM is on GPIO25/26 (clean, non-strapping) so
the same pins serve the L298N enables now and the ESC signals later.

## What works now (Phase 1)

Manual differential drive from the DS600, arming (sticks-centered guard),
RC-loss → FAILSAFE → neutral, battery-critical failsafe, slew-limited smooth
output, and serial telemetry. The 3PDT bypass-sense and ESC driver are wired
into the structure and compile, but the switch/ESC aren't installed yet —
`BypassSense` reads AUTO on an unconnected pin, and autonomous modes are inert
(no IMU/GPS mounted).

## Roadmap hooks (where later phases plug in)

- **Phase 2 (IMU/AHRS):** add a `src/estimation/AHRS.*` task using DLHC-specific drivers (L3GD20 gyro 0x6B, LSM303DLHC accel 0x19 / mag 0x1E) feeding a Madgwick/Mahony filter; feed heading to `headingPID`. BMP180 (0x77) read at ~1 Hz for temperature/telemetry only — not in the control path.
- **Phase 3 (heading hold):** populate the `HEADING_HOLD` case in `main.cpp` with `mix(0, headingPID.update(...), L, R)`.
- **Phase 4 (GPS):** add `src/estimation/GPSReader.*` on UART2.
- **Phase 5/6 (anchor / combined):** nested loop — `distancePID` (outer, GPS-rate) → `headingPID` (inner).
- **Phase 7 (telemetry):** swap the serial `telemetryTask` for a WiFi/WebSocket stream + live gain tuning.
- **ESC + 3PDT migration (§1.2a):** `pio run -e kayak`; move two signal wires to the ESCs; wire the 3PDT third pole to GPIO13. No source edits beyond gains.

## Notes / verify before relying on it

- Set `BATT_DIVIDER` in `config.h` to your actual resistor ratio.
- Confirm the board is WROOM (GPIO16/17 free for GPS); WROVER ties those to PSRAM.
- Confirm the DS600 outputs per-channel servo PWM (not a single PPM/SBUS stream).
- Bidirectional ESCs usually need a neutral arming hold on power-up — already held at neutral in `ESC_Driver::begin()`.
- This scaffold is structured to compile cleanly; run `pio run -e mule` to verify against your installed core version.
