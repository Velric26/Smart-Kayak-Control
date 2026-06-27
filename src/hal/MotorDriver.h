#pragma once
#include <Arduino.h>
#include "config.h"
// =====================================================================
//  MotorDriver  -  abstract actuator interface (the migration boundary).
//  Everything above the HAL speaks only normalized thrust [-1.0, +1.0].
//  Swapping the actuator (now dual ESC) or mule -> kayak is an implementation swap.
// =====================================================================
class MotorDriver {
public:
  virtual ~MotorDriver() {}

  // Configure pins/peripherals. Must leave outputs at NEUTRAL.
  virtual void begin() = 0;

  // Command thrust per side. Inputs clamped to [-1, +1] by the impl.
  //   +1 = full forward, 0 = neutral/stop, -1 = full reverse.
  virtual void setThrust(float left, float right) = 0;

  // Raw per-side actuation [-1,1]: applies direction invert but NO min-drive
  // shaping, so the exact duty reaches the motor. Used by calibration routines
  // to probe breakaway/run floors. Impl updates lastL/lastR.
  virtual void setRaw(float left, float right) = 0;

  // Force immediate neutral / safe state.
  virtual void disable() = 0;

  // Human-readable name for telemetry.
  virtual const char* name() const = 0;

  // Final applied output per side [-1,1], after invert + min-drive.
  // Updated by setThrust()/disable() so telemetry can show real motor drive.
  float lastL = 0, lastR = 0;

  // Two-tier per-side drive floors (live-tunable). KICK breaks static friction
  // for the first kickMs after a motor starts; RUN sustains it thereafter.
  float kickL = MOTOR_KICK_L, kickR = MOTOR_KICK_R;
  float runL  = MOTOR_RUN_L,  runR  = MOTOR_RUN_R;
  uint32_t kickMs = MOTOR_KICK_MS;
  // Per-side output cap: a full command maps here, not 1.0 (tames aggression).
  // Only honored when capActive is set — control layer enables it for the
  // autonomous modes and disables it in MANUAL (full stick authority).
  float maxL = MOTOR_MAX_L, maxR = MOTOR_MAX_R;
  bool  capActive = true;

protected:
  // Map a logical command [-1,1] to actuator duty using the breakaway-kick then
  // sustain-run floors. side: 0=left, 1=right. Tracks per-side motion + kick
  // timing across calls (setThrust runs every control tick).
  float shape(int side, float v) {
    bool&     moving = side ? movingR_ : movingL_;
    uint32_t& t0     = side ? kickT0R_ : kickT0L_;
    float     kick   = side ? kickR    : kickL;
    float     run    = side ? runR     : runL;
    float     maxv   = capActive ? (side ? maxR : maxL) : 1.0f;
    if (fabsf(v) < 0.02f) { moving = false; return 0.0f; }   // stopped
    uint32_t now = millis();
    if (!moving) { moving = true; t0 = now; }                // breakaway edge
    float floorv = (now - t0 < kickMs) ? kick : run;
    if (maxv < floorv) maxv = floorv;                        // guard against an inverted range
    float m = floorv + (maxv - floorv) * fabsf(v);           // scale into [floor, max]
    return (v < 0.0f) ? -m : m;
  }

private:
  bool     movingL_ = false, movingR_ = false;
  uint32_t kickT0L_ = 0, kickT0R_ = 0;
};
