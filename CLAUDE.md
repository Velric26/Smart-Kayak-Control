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
- GPS: **u-blox NEO-8M** on UART2 (RX=GPIO16<-GPS TX, TX=GPIO17->GPS RX), powered from the ESP32
  **3V3** rail (NOT the 2S pack — the module LDO maxes ~6V). `TinyGPSPlus` parses it (`hal/GPS.*`).
  **Configured + saved to flash** via `tools/gps_config.py` (UBX over the CP210x USB-UART): **38400
  baud, 5 Hz, Pedestrian dynamic model, SBAS/WAAS on, GST enabled** (per-axis position sigma ->
  `GPS::accM()` horizontal accuracy in metres, used to gate anchor capture/chase). `GPS_BAUD` in
  `hal/GPS.cpp` + `diag/gps_raw.cpp` is 38400 to match; if you factory-reset the module it reverts
  to 9600 (re-run the tool). Switch dynModel to **Sea (5)** for the kayak. Guadalajara is ~1500 m,
  so the Sea model still works (it only constrains vertical dynamics).

## Pin map
Authoritative copy is in `include/config.h`. Key points:
- Motor PWM L=GPIO25, R=GPIO26 — these become the ESC signal pins later (wire stays, only
  firmware reconfigures). Kept off strapping pins so future ESC signal is glitch-free at boot.
- L298N direction: IN1=27, IN2=14, IN3=18, IN4=19 (freed when swapping to ESC).
- I2C: SDA=21, SCL=22. GPS UART2: RX=16, TX=17 (NEO-8M wired & live).
- RC in: mixed-L=35, mixed-R=36 (VP), Ch5/ARM=39 (VN, latching), Ch6/MODE=4 (momentary).
- Battery sense=34 (ADC1). 3PDT bypass sense=13 (RESERVED — bypass logic removed
  from firmware; a floating pin glitched the state machine. Re-add when 3PDT wired). Status LED=2.

## Build & flash (PlatformIO)
Run from the project root. `pio` is on PATH (`%USERPROFILE%\.platformio\penv\Scripts`).
- Mule build:   `pio run -e mule -t upload`   then  `pio device monitor`   (adds TinyGPSPlus dep)
- Kayak build:  `pio run -e kayak -t upload`   (adds ESP32Servo + TinyGPSPlus deps)
- Diagnostics (each is its own env): `diag_i2c`, `diag_imu`, `diag_calib`, `diag_heading`,
  `diag_fused`, `diag_gps` (NEO-8M NMEA readout, mirrors over Bluetooth so you can roam for sky view).
  e.g. `pio run -e diag_gps -t upload`
Env selection in `platformio.ini` swaps the driver via `build_src_filter` + `DRIVER_ESC` flag.

## Live tuning + calibration (no reflash) — over USB **or** Bluetooth
The full command reference lives at the **top of `include/config.h`** (CONSOLE REFERENCE block).
Type + Enter; each value command echoes the whole set. Highlights:
- Heading: `kp` `kd` `db` (deg). Deadband is the calming knob.
- Drive floors (two-tier, per side): `kickl/kickr` (breakaway), `runl/runr` (sustain; `mnl/mnr`
  are aliases), `kickms` (kick duration). Output cap: `mxl/mxr` (autonomous modes only).
- Motion: `slew` (accel/ramp-up), `slewdn` (decel/ramp-down to stop), `amp` (relay amplitude).
- `log <hz>` telemetry rate (`log 0` mutes; auto-muted during routines).
- Word cmds: `tune` (relay auto-tune), `stop`, `clrgains` (forget NVS gains).
- Cal routines: `cal kick|run l|r|both`, `cal max` (find wheelspin), `cal compass`
  (manual-spin mag hard/soft-iron — applies + saves to NVS, auto-loaded at boot).

**Persisted in NVS:** heading gains (from `tune`) and mag cal (from `cal compass`). Everything
else is RAM-only until baked into `config.h`.

Telemetry line: `L=`/`R=` are the post-drive-shaping values the motors actually get (mode name at
the line start already conveys arm/mode state). `drop=` masked HEADING_HOLD glitches;
`gps=<sats>s/<hdop> FIX`; `cog=` GPS course (while moving); `anc=<dist>m@<brg>` in anchor modes.
(`batt=` removed until divider wired.)

## Current state (as of this handoff)
- Phases 0–2 complete: build/flash, dual-core FreeRTOS (control on core1 @100Hz, telemetry core0),
  state machine, RC link + latching arm + momentary mode-cycle, RC-loss & battery failsafes,
  manual differential drive (both motors inverted), IMU bring-up, mag calibration, fused heading.
- Phase 3 (Heading Lock) closed loop WORKS — correct sign, returns to setpoint.
- Motors now run off the **2S pack** (USB to ESP32 for monitor only); pivots rotate fine on
  battery, confirming the earlier USB-power stall was a supply/L298N-drop issue.
- **Bluetooth telemetry mirror:** `BluetoothSerial` (SPP) mirrors all telemetry + accepts the
  live-tuning commands, so you can watch logs and tune off-USB while on battery. Pairs as
  `SmartKayak-Mule` / `SmartKayak-Kayak`. Use `\r\n` line endings (PC terminals need the CR).
- **Relay auto-tune** (`tune` over the console) drives a bang-bang wag and derives PD gains
  (Åström–Hägglund). CAVEAT: the drive floor is a hard nonlinearity that makes the rover
  limit-cycle, so auto-tune gains tend to oscillate — the **deadband (`db`) is the real cure**.
  Tune manually: widen `db` until it sits quiet, then set `kp`/`kd` for correction feel.
- **Two-tier drive shaping (in the HAL base, carries to ESC):** per-side breakaway KICK for
  `kickMs`, then a lower sustain RUN floor; per-side output cap (`mxl/mxr`) applied in autonomous
  modes only (MANUAL keeps full stick); asymmetric slew — gentle `slew` up, quick `slewdn` stop.
  `cal kick|run|max` routines help dial these in by eye. `setRaw()` added for the cal probes.
- **Compass cal in-firmware** (`cal compass`): spin by hand, derives + saves mag hard/soft-iron to
  NVS (auto-loaded at boot). `MAG_OFF/MAG_SCALE` are now runtime-overridable in `HeadingAHRS`.
- **Heading-lock setpoint is glitch-stable:** re-grabbed only after a real disengage
  (`HEADING_REGRAB_MS` grace), so brief input glitches don't walk `sp`. `drop=` in telemetry
  counts masked glitches; with bypass logic removed, a climbing `drop` now points at RC-link flicker.
- **Phase 4 GPS bring-up DONE:** NEO-8M validated (`diag_gps`, 7 sats / HDOP 1.4) and integrated
  into the main firmware (`hal/GPS.*`); telemetry shows `gps=`. No motor behavior change yet.
- **In progress / open:** (1) bake final tuned values into `config.h` (drive floors, caps, slew,
  heading gains are live/NVS but not yet committed as defaults); (2) **ANCHOR mode** next — capture
  lat/lon on engage, drive home via `distancePID` (throttle) + the tuned heading loop (bearing),
  using `TinyGPSPlus::distanceBetween`/`courseTo`.

## Provisional / "do not forget" config values
- Drive shaping is now **two-tier per-side** (`MOTOR_KICK_L/R`, `MOTOR_RUN_L/R`, `MOTOR_KICK_MS`)
  plus per-side caps (`MOTOR_MAX_L/R`) and asymmetric slew (`THRUST_SLEW_PER_S` up /
  `THRUST_DECEL_PER_S` down). The old single `MOTOR_MIN_DRIVE` scalar is gone. These are
  live-tunable; **current live/NVS values may differ from the baked defaults until committed** —
  finalize via the `cal`/`slew` commands, then bake the winners into `config.h`.
- **Battery monitor is disabled in firmware** (divider not wired): `BatteryMonitor::update()` is
  commented out and `warn()`/`critical()` return false; the `batt=` field is dropped from
  telemetry. `BATT_CRITICAL_V` is now unused. Re-enable `BatteryMonitor.cpp` and restore the
  `batt=` field once the divider (`BATT_DIVIDER = 2.79`, R1=100k/R2=56k -> GPIO34) is wired.
- Calibrated/measured constants baked in `config.h`: MAG_OFF, MAG_SCALE (NVS cal overrides these
  at boot when present), gyro/heading signs, HEADING_FUSE_ALPHA=0.98, motor inverts both true.

## Pending hardware
- 3D-print the second-floor IMU mount (away from motor currents), then **re-run mag calibration**
  (now in-firmware: `cal compass`).
- Wire the battery divider; then restore the battery failsafe.
- Acquire 3PDT switch + ESCs for the kayak migration.
- (GPS NEO-8M now acquired, wired to UART2, and validated — no longer pending.) USB-UART adapter
  on hand to reconfigure the module in u-center later (update rate / baud) if desired.

## Roadmap
GPS bring-up ✓ (done) -> **Smart Anchor / position-hold (NEXT)** -> tilt-compensated heading
(for water) -> waypoints -> WiFi telemetry & runtime tuning -> ESC swap on the mule -> kayak
migration. See `docs/architecture.md` §10–§11.

## Working in this repo
- This is now a git repo: make a commit after each validated change instead of re-zipping.
- Brace-check new C++ files and keep platform code behind the HAL.
- Don't reintroduce L3GD20 / BMP180 assumptions.
