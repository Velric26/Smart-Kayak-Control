# Smart Kayak Thruster Control System — Architecture & Development Plan

Development on the differential-drive mule, migrating with minimal changes to the twin-thruster fishing kayak.

> **STATUS (current):** The rover has **migrated off the L298N H-bridge to dual bidirectional
> ESCs** (one servo-PWM signal per ESC on GPIO25/26) — the `L298N_Driver` is retired and only
> `ESC_Driver` ships. References to the L298N below describe the original two-phase development
> plan (now executed) and remain as design rationale. The 3PDT hardware-bypass (§1.6) is still
> pending; its firmware logic was removed until the switch is installed (GPIO13 reserved).

---

## 0. Guiding Principles

Three principles drive every decision below:

1. **Hardware Abstraction Layer (HAL) is the migration key.** The control code must never know whether it is driving an L298N brushed motor, a brushed ESC, or a brushless ESC — or whether it is on land or water. Every actuator and sensor sits behind an interface. Swapping the L298N for the dual brushed ESC, and later the mule for the kayak, becomes an implementation swap plus a re-tune, not a rewrite.

2. **The mule validates behavior, not water dynamics.** It will fully validate: RC reading, arming, failsafe, the state machine, heading estimation, the differential mixer, mode switching, and telemetry. It will *not* validate position-hold tuning, because land friction and instant stopping are nothing like a kayak's inertia, drift, and reverse-only braking. Plan to re-tune gains on the water. Make every gain runtime-adjustable over the telemetry link so re-tuning never requires a reflash.

3. **Safe state is "motors neutral."** On water, a runaway is far worse than a drift. Every fault, every uncertainty, every undefined transition collapses to neutral thrust. Arming is explicit and deliberate.

---

## 1. Hardware Architecture

### 1.1 Optimized pin map (final wiring target)

This map is laid out around the *end state* so the L298N → ESC swap is nearly wireless. The two motor-PWM pins sit on clean, non-strapping GPIOs **now** (as the L298N enables) and become the **ESC signal lines later** — same pins, only the firmware timing changes. The four direction pins simply free up (an ESC encodes direction in pulse width).

| Function | GPIO | Phase (now → later) | Notes |
| --- | --- | --- | --- |
| Motor PWM L | **GPIO25** | ENA (~20 kHz) → **ESC Left signal** (50 Hz) | wire stays; firmware reconfigures LEDC |
| Motor PWM R | **GPIO26** | ENB → **ESC Right signal** | same |
| Dir IN1 | GPIO27 | L298N only | freed at ESC swap |
| Dir IN2 | GPIO14 | L298N only | freed |
| Dir IN3 | GPIO18 | L298N only | freed |
| Dir IN4 | GPIO19 | L298N only | freed |
| I2C SDA (IMU) | GPIO21 | both | HW-default I2C |
| I2C SCL (IMU) | GPIO22 | both | HW-default I2C |
| GPS RX (← module TX) | GPIO16 | both | UART2 |
| GPS TX (→ module RX) | GPIO17 | both | UART2 |
| RC: mixed L | GPIO35 | both | input-only, parallel tap |
| RC: mixed R | GPIO36 | both | input-only |
| RC: mode (Ch5) | GPIO39 | both | input-only |
| RC: arm / E-stop (Ch6) | GPIO4 | both | interrupt input |
| Battery sense | GPIO34 | both | ADC1, input-only |
| 3PDT bypass sense | GPIO13 | **reserve, wire later** | regular GPIO + pull |
| Status LED (optional) | GPIO2 | both | onboard LED, arming heartbeat |
| Spare | GPIO23, 32, 33 | — | 32/33 are ADC1; 23 freed from old ENA |

Design rationale:

- **RC channels + battery on the input-only pins (34–39) plus GPIO4.** Input-only pins can't drive anything, so spending them on sensing preserves every output-capable pin for actuators and expansion. No internal pull-ups exist there, which is fine: the DS600 drives push-pull, and a disconnected receiver simply reads as "no valid pulses" → the signal-loss watchdog handles it.
- **Direction pins kept off ADC1 (32/33),** leaving two spare analog-capable inputs available *now* (e.g. for current sensing or a second voltage tap before the ESC arrives).
- **Motor PWM moved off the old GPIO23/GPIO5.** GPIO5 is a strapping pin you never want an ESC signal on at boot (§1.3); retiring it from the motor path is the whole point of this layout.

### 1.2 Current status of hardware

The IMU, GPS, RC receiver, and battery are defined and can be wired now. The **3PDT bypass switch and the dual brushed ESC are not yet in hand** — plan for them, but Phase 0–1 runs on the L298N with the 3PDT-sense pin (GPIO13) left unconnected.

The IMU is a 9-DOF module (GY-801-type) combining two chips on one I2C bus: an **L3G4200D** gyro and an **LSM303DLHC** accelerometer + magnetometer (addressed as two separate devices). This particular board has **no barometer** populated. All addresses are distinct, so they coexist on the single bus (GPIO21/22) with no conflict:

| Device | Provides | I2C addr |
| --- | --- | --- |
| L3G4200D | gyroscope (3 axes) | 0x69 (confirmed via WHO_AM_I = 0xD3) |
| LSM303DLHC | accelerometer (3 axes) | 0x19 |
| LSM303DLHC | magnetometer (3 axes) | 0x1E |

Two naming traps to note, both confirmed on the bench: the gyro is an **L3G4200D, not an L3GD20** — different register map and WHO_AM_I (0xD3 vs 0xD4), and the L3GD20 can't even sit at 0x69. And the accel/mag is an **LSM303DLHC, not an LSM303D** — separate accel/mag addresses and a different register map. Phase 2 uses chip-specific drivers for both (§9); a driver for the wrong variant will silently fail to communicate.

### 1.2a Low-effort migration (when the 3PDT + ESC arrive)

1. Unplug the L298N entirely (6 signal wires + its motor power).
2. Move two wires only: GPIO25 → ESC-Left signal, GPIO26 → ESC-Right signal (the former enable pins).
3. GPIO27 / 14 / 18 / 19 are now free — leave or repurpose.
4. Insert the 3PDT selecting {ESP32 outputs} vs {DS600 mixed outputs} → ESCs; run the third pole to GPIO13.
5. Firmware: swap `L298N_Driver` → `ESC_Driver` (LEDC 20 kHz → 50 Hz servo on 25/26) and enable the GPIO13 bypass-sense logic. Nothing else is touched.

### 1.3 Strapping-pin warning (matters for ESCs)

GPIO0, 2, 5, 12, 15 are **strapping pins** — sampled at boot, and they can emit stray pulses during reset. The §1.1 map deliberately keeps both motor-PWM pins (25/26) off them, so the future ESC signals are glitch-free at boot. Hold to these rules anyway:

- Keep ESC signal lines on non-strapping GPIOs (the map already does).
- Keep ESCs physically unpowered or signal-disconnected until firmware explicitly arms them.
- Initialize all actuator outputs to neutral **before** anything else in `setup()`.

### 1.4 Battery monitoring (2S Li-ion, 7.4 V nom / 8.4 V full)

- Voltage divider into an **ADC1** pin (8.4 V → ~3.0 V at the pin; e.g. 100k/56k divider, verify the top stays under 3.3 V).
- ESP32 ADC is non-linear — use `esp_adc_cal` / the calibrated `analogReadMilliVolts()` and a multi-sample average.
- Li-ion cutoffs: warn at **3.3 V/cell (6.6 V)**, hard failsafe at **3.0 V/cell (6.0 V)**. For cell longevity, prefer returning home around 6.6–6.8 V rather than draining to 6.0 V repeatedly.
- Feed this into the battery-aware return-to-home logic later.

### 1.5 Magnetometer placement (both platforms)

The LSM303DLHC magnetometer is your absolute heading reference and it is sensitive to current and ferrous mass. Brushed motors and especially high-current brushless ESC leads create fields that swamp Earth's. Mount the IMU as far as practical from motors, ESCs, and battery leads, and run hard-iron/soft-iron calibration **with the motors drawing current**, on the actual platform. Recalibrate after migrating to the kayak. The DLHC mag is a modest part — noisier and more temperature-sensitive than newer magnetometers — so disciplined calibration matters even more here for a clean, stable heading.

### 1.6 Manual-override bypass switch (hardware failsafe)

The DS600 performs the differential mix in the TX/RX, so its outputs are the two **already-mixed left/right ESC signals** — not raw throttle + steering. A physical **3PDT switch (9-pin)** sits on the two ESC signal lines and selects their source; its third pole reports the position back to the MCU:

```
                 DS600 RX (mixed L, mixed R)
                        |        |
   ESP32 (L,R) ----+    |        |    +---- ESP32 (L,R)
                   |    |        |    |
                 [ pole 1 ]    [ pole 2 ]    [ pole 3 ] --> GPIO13 (sense)
                   |               |
                 ESC Left       ESC Right

   Position AUTO   : ESCs <- ESP32 outputs ;   sense = AUTO
   Position MANUAL : ESCs <- DS600 RX outputs (MCU removed from loop) ; sense = MANUAL
```

This is the top of the safety stack: a hardware mux that software can never defeat. If the ESP32 hangs, crashes, or bricks, flip to MANUAL and drive directly off the remote. **Confirmed as a 3PDT (9-pin):** two poles carry the L/R signals, the third pole feeds the bypass-sense line (GPIO13) so the MCU always knows which source is live. Design rules:

- **Common ground is mandatory.** ESP32, DS600 RX, and ESCs must share ground or the PWM through the switch is undefined. This is the #1 silent failure of bypass setups.
- **The MCU is a *parallel listener*, never a series pass-through.** It taps the RC lines with high-impedance inputs to read intent and mode, and independently generates its own ESC signals in AUTO. The 3PDT just picks which source reaches the ESCs.
- **Third pole → GPIO13 (sense).** Because the MCU sees the switch state, it can freeze/reset its controllers the instant it's bypassed and re-sync cleanly on return (bumpless transfer, §8.5) — no manual re-arm dance required.
- **Power-on in MANUAL**, or power the ESCs only after the ESP32 is up and holding neutral — otherwise boot-time GPIO glitches (§1.3) can twitch the thrusters while in AUTO.
- Flipping the switch briefly opens the signal mid-transition; ESCs hold-last or go to their own failsafe for that instant — normally harmless, just expected.

> **Not yet in hand:** the 3PDT and the dual ESC are still to be acquired. Phase 0–1 run on the L298N with GPIO13 unconnected; everything here is wired in at the migration step (§1.2a).

### 1.7 Mechanical / chassis design (3D-printed mule)

The mule is a two-floor printed chassis. Layout drives heading accuracy as much as the firmware does, because the magnetometer (§1.5) is the most placement-sensitive part on the vehicle.

```
   [GPS antenna ^ up, clear sky]      [IMU on damped cradle, offset]   <- Floor 2: sensors
            |  ground plane underneath      |  not directly above motors/batt/L298N
   =====================================================  Floor 2 plate
            ||   tall standoffs  (more height = less magnetic coupling)
   =====================================================  Floor 1 plate (top)
   [ ESP32 ]            [ L298N  -> ESC (removable sub-plate) ]         <- Floor 1 top: control
   -----------------------------------------------------------  Floor 1 underside
   (( motor L ))   (( motor R ))   [ 2S 18650 + retention ]   o caster  <- hangs below
```

**Sensor mounting (highest impact).** Three rules for the IMU:
- *Distance from current and motors.* Field from the brushed motors falls off ~1/r³ in the near field, so the floor-to-floor standoff height does real work — make it as tall as practical. Additionally **offset the IMU horizontally** so it is not directly above a motor, the battery, or the L298N; all three sit below it and all three are noise sources.
- *Non-ferrous hardware nearby.* Steel screws/standoffs/nuts within a few cm create a fixed hard-iron offset and distortion. Use **brass or nylon fasteners** at the sensor (brass heat-set inserts are non-magnetic and fine; pair with nylon screws right at the cradle).
- *Rigid, known orientation + vibration isolation.* Print a cradle that locks the module to the vehicle axes (misalignment becomes a permanent heading bias), then **soft-mount it** (TPU cradle or rubber/foam grommets) so motor and drive vibration doesn't corrupt the accelerometer the AHRS uses for tilt compensation. This matters more on the kayak, so design it in now.

**GPS mounting.** Patch antenna faces straight up with nothing metallic above or beside it, a small ground plane underneath (copper tape / thin disc) improves the pattern, and keep it a few cm from the ESP32 (2.4 GHz WiFi/BT can desensitize the 1.5 GHz GPS front-end). Forgiving on the rover since testing is outdoors.

**Battery retention (inverted holder).** Spring-contact 18650 holders lose contact under simultaneous inversion + vibration, producing intermittent ESP32 brownouts that masquerade as firmware bugs. Mitigate on two fronts: a **mechanical retainer** (printed clip/cap or TPU strap), *and* a **bulk capacitor across the ESP32 supply rail** to ride through micro-dropouts. A holder with solder tabs / screw terminals removes the spring-contact failure mode entirely. Keep battery and motor leads short and **twisted as a pair** (shrinks the radiating loop) and routed away from the IMU above — mechanical routing is half of the EMI mitigation in §1.5.

**Modularity (mirror the HAL in hardware).** Mount the motor driver on a **removable sub-plate / standoffs** and **connectorize** motor and signal leads (JST/Dupont) rather than soldering direct, so the L298N → ESC swap is unplug-replace — the physical analogue of the one-line HAL change. Use **brass heat-set inserts** for anything opened repeatedly (self-tapping into plastic strips after a few cycles). Provide a grommeted cable pass-through between floors with strain relief.

**Weight distribution (differential rover).** Keep CG low and centered — the underside battery already helps — and biased slightly toward the drive axle so the wheels keep traction while the front caster stays lightly loaded but free to swivel. Too much on the caster fights turns; too little and the nose wanders.

**Printing / structural.** Orient motor mounts so layer lines are not a shear plane under motor torque; add generous fillets and extra perimeters/infill locally at motor and standoff bosses. Leave clearance at the motor terminals to solder **suppression caps directly across the motor** — where they are most effective for the EMI flagged in §1.5, and easy to forget to make room for.

**Transplantable-module mindset.** Treat the sensor mount (damped IMU cradle + GPS plate) as a self-contained sub-assembly with defined orientation, just as the HAL is a self-contained software layer. If it lifts off as a unit, moving it to the kayak is a known quantity rather than a redesign.

---

## 2. Software Architecture

### 2.1 Layered design

```
            +--------------------------------------------------+
            |                 Telemetry / Config               |  WiFi/BT, runtime gains, logging
            +--------------------------------------------------+
            |                  State Machine                    |  modes, arming, transitions
            +--------------------------------------------------+
            |   Control: Position loop -> Heading + Speed loop  |  PID controllers (cascaded)
            +--------------------------------------------------+
            |        Arbitration / Override / Mixer             |  combine auto + RC, enforce priority, differential mix
            +--------------------------------------------------+
            |   State Estimation / Sensor Fusion (AHRS, pos)    |
            +--------------------------------------------------+
            |    HAL: Motors | RC In | GPS | IMU | Battery      |  <-- swap point for ESC and for kayak
            +--------------------------------------------------+
                              ESP32 hardware
```

Everything above the HAL is platform-agnostic. The HAL is the only layer that changes between L298N → ESC → kayak.

### 2.2 Concurrency: FreeRTOS, dual-core pinned

The ESP32 WiFi stack lives on core 0. Pin your deterministic control to core 1 so telemetry/WiFi jitter never disturbs the loop:

- **Core 1 (real-time):**
  - IMU/AHRS task — 100–200 Hz (gyro wants fast sampling)
  - Control + mixer + RC-decode task — 50–100 Hz
  - Safety/watchdog task — high priority, ~50 Hz
- **Core 0 (best-effort):**
  - GPS parse + position loop — 5–10 Hz (GPS-rate)
  - Telemetry / config server — 1–10 Hz

Share state through small structs guarded by a mutex (or lock-free single-writer atomics). Every loop computes a real `dt` from `micros()` and passes it to the controllers — never assume a fixed timestep.

### 2.3 The HAL motor interface (the migration boundary)

Normalize all thrust to `-1.0 .. +1.0`. The control layer only ever speaks this language.

```cpp
class MotorDriver {
public:
  virtual void begin() = 0;
  virtual void setThrust(float left, float right) = 0; // -1..+1 each
  virtual void disable() = 0;                           // -> neutral, motors off
};

// Mule today: L298N (sign-magnitude)
class L298N_Driver : public MotorDriver {
  void setThrust(float l, float r) override {
    driveSide(IN1, IN2, ENA_ch, l);
    driveSide(IN3, IN4, ENB_ch, r);
  }
  void driveSide(int inA, int inB, int pwmCh, float v) {
    bool fwd = v >= 0;
    digitalWrite(inA, fwd); digitalWrite(inB, !fwd);
    ledcWrite(pwmCh, (int)(fabs(v) * PWM_MAX));
  }
};

// Later: brushed/brushless ESC (servo PWM, 1000-2000us, 1500 neutral)
class ESC_Driver : public MotorDriver {
  void setThrust(float l, float r) override {
    escWriteUs(escL, 1500 + (int)(l * 500));
    escWriteUs(escR, 1500 + (int)(r * 500));
  }
};
```

Migrating to the ESC, and to the kayak, swaps the concrete class. Nothing above the HAL changes.

---

## 3. Sensor Fusion / State Estimation

### 3.1 Heading (the workhorse for both heading-lock and anchor)

Fuse gyro + accelerometer + magnetometer with a **Mahony or Madgwick AHRS** filter:

- Gyro → fast, smooth short-term response (but drifts).
- Accelerometer → gravity vector for tilt compensation (so wave-induced roll/pitch doesn't corrupt the compass).
- Magnetometer → absolute heading reference (corrects gyro drift).

Output a **tilt-compensated heading**. Mahony is lighter and very stable for this; Madgwick is fine too. Run it at 100–200 Hz.

### 3.2 Position

For a fishing kayak holding station, GPS position + a PID is the same basic approach commercial spot-lock systems use. Start simple and add fusion only if needed:

- Use GPS lat/lon directly, lightly low-passed. Convert to a local tangent plane (meters N/E from the anchor point) so your math is in meters, not degrees.
- Configure the NEO-8M from its 1 Hz default up to **5 Hz** (10 Hz is the ragged edge for that module). Run the position loop at the GPS rate.
- IMU dead-reckoning between fixes is tempting but double-integrating accelerometer noise drifts badly — skip it initially. A light EKF fusing GPS velocity with heading can come later if you want smoother behavior.

### 3.3 The course-over-ground gotcha (design-critical)

GPS course-over-ground (COG) is **only valid when moving** above ~0.5 m/s. When the boat is stationary — i.e. exactly during anchor hold — COG is pure noise. Therefore:

- **Heading always comes from the magnetometer/AHRS, never from COG.**
- Use COG only as a sanity check / mag-calibration aid *while moving*.

This single rule prevents a class of bugs where the boat spins in circles at low speed.

### 3.4 Sensor gating

Don't trust sensors blindly: require a GPS fix with adequate satellite count / HDOP before allowing anchor mode; reject AHRS heading if the filter hasn't converged; flag stale data (no GPS update in N ms → degrade gracefully).

> Note: the board on hand is a 9-DOF unit with **no barometer**, so there is no altitude/pressure/temperature source. Heading needs only gyro + accel + mag, so this costs nothing for the planned features.

---

## 4. Control Algorithms

### 4.1 Differential mixer

All higher-level controllers output two abstract commands — forward velocity `v` and turn rate `ω` — which the mixer converts to left/right thrust:

```cpp
void mix(float v, float w, float &left, float &right) {
  left  = v - w;
  right = v + w;
  // preserve turn authority: scale both down if saturated rather than clipping
  float m = max(fabs(left), fabs(right));
  if (m > 1.0f) { left /= m; right /= m; }
}
```

This is identical on mule and kayak — differential kinematics transfer directly.

### 4.2 Heading lock

A single PID on heading error → turn rate `ω`, with correct angle wrapping:

```cpp
float headingError(float setpoint, float actual) {
  float e = setpoint - actual;
  while (e > 180.f) e -= 360.f;
  while (e < -180.f) e += 360.f;   // shortest-path error
  return e;
}
// w = headingPID.update(headingError(hdgSet, hdgActual), dt);
// mix(v_command, w, left, right);
```

Works stationary (rotate in place via differential) or while moving.

### 4.3 Position hold (anchor) — cascaded loops

A kayak is non-holonomic: it cannot strafe sideways, only drive forward/back and turn. So position hold = *point toward target and drive to it*, with a deadband to stop hunting. Structure it as nested loops:

```
GPS pos --> [Outer: position loop @ 5-10Hz] --> desired heading + desired speed
                                                      |
                          +---------------------------+
                          v
AHRS hdg --> [Inner: heading + speed loop @ 50-100Hz] --> v, w --> mixer --> thrust
```

Outer loop (slow, GPS-rate):
1. Vector from current position to anchor point → **distance** and **bearing**.
2. If `distance < holdRadius` (user-adjustable deadband) → command idle / station-keep, *do not chase*. This is what makes it smooth and stops the constant hunting that drains battery.
3. If outside → desired heading = bearing to target; desired speed = P/PI on distance (clamped to a gentle approach speed).

Inner loop (fast):
- Heading PID drives `ω` to point at the target.
- Speed term drives `v` toward the target.
- Mixer converts to thrust.

The "combined position + heading hold" mode is the same outer position loop, but once inside the hold radius the inner loop switches its heading setpoint from "bearing to target" to the user's **selected heading** — so it holds the spot *and* faces the way you want (e.g. into the wind for fishing).

---

## 5. PID Tuning Methodology

**Tune inner loops before outer.** Heading first (on the mule you can twist the rover and watch it recover), then position (push the rover off the anchor point and watch it return).

Recommended manual procedure per loop:
1. P only: raise until it responds briskly and just begins to oscillate, then back off ~30–50%.
2. Add D to damp the overshoot/oscillation.
3. Add a small I to kill steady-state offset — essential on water, where constant wind/current is a persistent bias the integrator must counter.

Three things that matter more than the exact gains:

- **Anti-windup is mandatory**, especially position hold against a steady current — the integrator will wind up while saturated and cause huge overshoot. Use conditional integration (stop integrating when output is saturated) plus an integral clamp:

```cpp
struct PID {
  float kp, ki, kd, iMax, outMin, outMax;
  float integ = 0, prevE = 0;
  float update(float e, float dt) {
    float d = (dt > 0) ? (e - prevE) / dt : 0; prevE = e;
    float out = kp*e + ki*integ + kd*d;
    if (out > outMax) out = outMax;          // clamp
    else if (out < outMin) out = outMin;
    else integ += e * dt;                    // integrate only when NOT saturated
    integ = constrain(integ, -iMax, iMax);   // hard clamp
    return constrain(out, outMin, outMax);
  }
};
```

- **Output slew-rate limiting** gives you the "smooth control behavior" you asked for — ramp thrust changes rather than stepping them, so the boat glides instead of jerking.

- **Runtime-tunable gains.** Expose every gain over telemetry and persist to NVS. This is what makes the on-water re-tune painless — you adjust gains live from your phone/laptop while the kayak is doing its thing, no reflash.

Expect kayak gains to be very different from mule gains (slower, more inertia, disturbance-dominated). The *methodology and code* transfer; the numbers don't.

---

## 6. State Machine

```
        +--------+
        |  BOOT  | self-test, neutral outputs, sensor init
        +---+----+
            v
        +--------+   arm (RC switch + neutral throttle)
        |DISARMED|<-----------------------------+
        +---+----+                              |
            | armed                             | disarm / any fault
            v                                   |
   +--------------------- MANUAL ---------------+----+
   |        (RC direct differential drive)           |
   |   mode switch ->                                |
   |   +-------------+  +--------+  +-------------+   |
   |   |HEADING_HOLD |  | ANCHOR |  |ANCHOR+HDG   |   |
   |   +-------------+  +--------+  +-------------+   |
   +-------------------------------------------------+
            |
            v  (RC loss / low batt / sensor fault / geofence breach)
        +---------+
        |FAILSAFE |  -> motors neutral, alert, attempt safe recovery
        +---------+
```

Key rules:
- **Arming is explicit:** only from DISARMED, only with the RC mode switch in a known position and throttle at neutral. Prevents arm-on-boot surprises.
- **FAILSAFE is reachable from every state** and always wins.
- Undefined transitions → FAILSAFE.

---

## 7. Safety Mechanisms & Fail-Safe Behavior

Priority hierarchy, highest first:

0. **Hardware bypass switch (3PDT, §1.6)** — physically routes the ESCs to the RC receiver, independent of the MCU. The ultimate failsafe; no software fault can override it.
1. **Software safety override** (E-stop, hardware watchdog, battery cutoff) — beats all autonomy *within* AUTO.
2. **RC manual intent** — soft override; the human's sticks make the MCU yield while still in AUTO.
3. **Autonomous control** — only when 0–2 permit.

Note the division of labour: the hardware switch protects against the MCU *failing* (hang/crash/brick); the software override protects against the MCU *misbehaving while still running*. Both depend on a present human noticing — appropriate for a manned fishing kayak.

The arbitration layer enforces 1–3 every cycle. Specific protections:

- **RC signal-loss detection.** Track time since the last valid pulse on each channel; if it exceeds ~250–500 ms, declare RC lost → FAILSAFE. (Confirm whether your DS600 receiver outputs hold-last-value or drops pulses on TX loss; design to detect *absence of fresh, in-range pulses* either way.)
- **On RC loss in an autonomous mode:** configurable — either hold position (if GPS healthy) or cut to neutral and drift. On open water, neutral-and-drift is usually the safer default; you can opt into hold once you trust it.
- **Hardware watchdog timer.** If the control task hangs, the WDT resets the ESP32; outputs come up neutral and disarmed.
- **Battery failsafe** per §1.4 (warn → return-home → neutral).
- **GPS fix gating:** refuse to enter or stay in anchor without an adequate fix.
- **Geofence:** a hard boundary; breach → return-to-home or neutral.
- **Output clamping & arming guard:** the HAL physically cannot command thrust unless armed.
- **Always carry a paddle and have a physical recovery plan.** Validate every failsafe on the mule, then again on the water with a tether/spotter before trusting it.

---

## 8. RC Override Logic (DS600)

### 8.1 Signal topology

The DS600 mixes in the TX/RX (§1.6), so the two channels feeding the ESCs are **already-mixed left/right thrust**, and the hardware 3PDT — not the MCU — is what hands manual control to the human in a failure. The MCU's job with RC is therefore narrower and safer than a series pass-through: it **listens in parallel** to read intent and mode, and it **generates its own ESC signals** for the AUTO side of the switch.

### 8.2 Reading the DS600 on the ESP32

Tap the RC lines with high-impedance inputs (parallel, doesn't disturb the signal). The DS600 outputs standard servo PWM (1000–2000 µs, ~50 Hz). Read it **non-blocking** — never `pulseIn()`:

- **Interrupt capture:** `CHANGE` interrupt per pin, timestamp edges with `micros()`, compute width. Simple for a few channels.
- **ESP32 RMT peripheral:** purpose-built pulse decoding, offloads the CPU; best for many channels.

Validate every pulse (reject < 800 µs or > 2200 µs) and timestamp it for the signal-loss watchdog.

To recover the human's *intent* from the two mixed channels (for soft assist or override detection), inverse-mix:
`v = (left + right) / 2`, `ω = (right − left) / 2`. For pure "is the human commanding anything" you can just test either mixed channel against a neutral deadband.

### 8.3 Channel mapping (suggested)

| Channel | Function |
| ------- | -------- |
| Ch1 / Ch2 | Mixed L / R thrust (to 3PDT → ESCs; also tapped by MCU) |
| Ch5 | Mode select (HEADING_HOLD / ANCHOR / …) |
| Ch6 | Arm + soft E-stop |
| (3PDT 3rd pole) | Hardware bypass position sense → MCU GPIO (recommended) |

### 8.4 Two tiers of override

- **Hardware bypass (ultimate):** the 3PDT. MCU-independent, covers MCU *failure*. See §1.6.
- **Soft override (within AUTO):** while the MCU is in command, any mixed-channel input beyond a deadband immediately hands full control back to the human and bumps the state toward MANUAL. Predictable and intuitive under stress. Blended assist (autonomy biases, human nudges) is a later refinement — full-takeover-on-touch is the right default for early water testing.

### 8.5 Bumpless transfer (returning to AUTO)

Whenever control returns to the MCU — from hardware bypass or from soft override — **reset all integrators, re-capture the current heading/position as the new setpoint, and require a deliberate re-arm.** This prevents the wound-up-integrator slam described in §1.6 and makes re-engagement smooth instead of violent. With the 3PDT's sense line on GPIO13, the MCU detects the AUTO transition automatically and performs the reset the instant it sees AUTO.

---

## 9. ESP32 Implementation Specifics

- **Framework / tooling:** Arduino-ESP32 under **PlatformIO** (proper project structure, libraries, and build config — better than the Arduino IDE for a project this size). ESP-IDF is an option if you want more control, but Arduino gets you moving fast.
- **Motor PWM:** the **LEDC** peripheral. For the L298N enables, pick a frequency above audible (e.g. 20 kHz) at 8-bit resolution. For ESCs, output 50 Hz servo timing (LEDC, the `ESP32Servo` library, or RMT).
- **IMU:** I2C (`Wire`) feeding a Mahony/Madgwick filter. The project uses thin **register-level drivers** (no external library) for the **L3G4200D** gyro (WHO_AM_I 0xD3, 0x69) and the **LSM303DLHC** accel (0x19) / mag (0x1E) — see `src/estimation/IMU.*`. Mind the chip quirks the drivers handle: the L3G4200D needs the 0x80 auto-increment flag on multi-byte reads, and the DLHC magnetometer returns its axes in **X-Z-Y order, big-endian** with no auto-increment flag. No barometer is present on this board.
- **GPS:** UART2 + `TinyGPS++` for NMEA. For higher update rates configure the NEO-8M with UBX commands to 5 Hz.
- **Battery ADC:** ADC1 pin, `analogReadMilliVolts()` with averaging.
- **Config & calibration persistence:** the `Preferences` library (NVS) — store PID gains, mag calibration, hold radius, channel mapping. Survives reflash-less tuning.
- **Telemetry:** WiFi (AP or station) with a WebSocket or simple UDP stream of `setpoint vs actual` for live plotting; this is also your tuning console. BLE later for the phone app.
- **Timing:** fixed-dt loops via `micros()`, real `dt` passed to PIDs; pin tasks to cores as in §2.2.

---

## 10. Development Roadmap

Each phase is independently validated before moving on.

**Phase 0 — Foundation.** PlatformIO scaffold, HAL skeleton, `MotorDriver` abstraction. Drive the mule open-loop from serial commands. *Validates the HAL boundary.*

**Phase 1 — RC + manual + failsafe.** Read the DS600 (non-blocking), differential manual drive, deadbands, arming logic, RC-loss → neutral. No sensors yet. *This is your safety backbone; get it bulletproof.*

**Phase 2 — IMU bring-up.** I2C, raw data, gyro-bias / accel / hard- & soft-iron mag calibration (motors running), AHRS filter → stable tilt-compensated heading.

**Phase 3 — Heading lock.** Heading PID, tune on the mule, validate disturbance recovery (twist the rover, watch it correct).

**Phase 4 — GPS bring-up.** Parse, fix-quality gating, local tangent-plane conversion, 5 Hz config, position logging over telemetry.

**Phase 5 — Position hold (anchor).** Nested loops, hold-radius deadband. Tune by pushing the rover off target. *Note: tuning here is mule-specific and will be redone on water — that's expected.*

**Phase 6 — Combined position + heading.** Add the "hold spot, face chosen heading" mode.

**Phase 7 — State machine + safety + telemetry integration.** Wire all modes, transitions, the full failsafe set, and the live tuning console together.

**Phase 8 — ESC swap on the mule.** Replace `L298N_Driver` with `ESC_Driver`. Everything above the HAL is untouched. Re-validate. *This rehearses the kayak migration with zero water risk.*

**Phase 9 — Kayak migration.** Same firmware, kayak `MotorDriver` config, recalibrate mag, **re-tune gains on the water** via telemetry. First water tests on a tether with a spotter and a paddle. Validate every failsafe before trusting autonomy.

**Phase 10 — Advanced features.** Waypoints, return-to-launch, geofence, battery-aware RTH, cruise control, drift compensation, fishing patterns, and the phone app — each layered on the proven core.

---

## 11. Mule → Kayak Migration Checklist

- [ ] Swap `MotorDriver` implementation (ESC config: neutral, range, reverse).
- [ ] Move ESC signal lines off strapping pins; verify no boot-time twitch.
- [ ] Transplant the IMU/GPS sensor sub-assembly as a unit; re-verify orientation and standoff offset from the new motors/wiring (§1.7).
- [ ] Recalibrate magnetometer on the kayak, motors running.
- [ ] Re-confirm GPS mounting / sky view.
- [ ] Re-tune heading and position PIDs on the water via telemetry (gains *will* differ).
- [ ] Re-verify RC-loss, low-battery, geofence, and E-stop failsafes on the water.
- [ ] **Test the hardware bypass switch on the water first** — confirm clean AUTO↔MANUAL handoff and that flipping to MANUAL gives full RC control with the MCU intentionally halted.
- [ ] Verify common ground across ESP32 / DS600 RX / ESCs end to end.
- [ ] First runs tethered, with a paddle and a spotter.

Everything else — state machine, mixer, fusion, RC logic, telemetry — carries over unchanged.
