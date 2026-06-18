#pragma once
// =====================================================================
//  MotorDriver  -  abstract actuator interface (the migration boundary).
//  Everything above the HAL speaks only normalized thrust [-1.0, +1.0].
//  Swapping L298N -> ESC -> kayak is purely an implementation swap.
// =====================================================================
class MotorDriver {
public:
  virtual ~MotorDriver() {}

  // Configure pins/peripherals. Must leave outputs at NEUTRAL.
  virtual void begin() = 0;

  // Command thrust per side. Inputs clamped to [-1, +1] by the impl.
  //   +1 = full forward, 0 = neutral/stop, -1 = full reverse.
  virtual void setThrust(float left, float right) = 0;

  // Force immediate neutral / safe state.
  virtual void disable() = 0;

  // Human-readable name for telemetry.
  virtual const char* name() const = 0;

  // Final applied output per side [-1,1], after invert + min-drive.
  // Updated by setThrust()/disable() so telemetry can show real motor drive.
  float lastL = 0, lastR = 0;
};
