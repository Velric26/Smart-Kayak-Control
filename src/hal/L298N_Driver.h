#pragma once
#include "hal/MotorDriver.h"

// Sign-magnitude brushed driver for the development rover.
// Uses analogWrite() for the enable-pin PWM (version-robust across the
// Arduino-ESP32 2.x/3.x LEDC API changes) and digitalWrite for direction.
class L298N_Driver : public MotorDriver {
public:
  void begin() override;
  void setThrust(float left, float right) override;
  void disable() override;
  const char* name() const override { return "L298N"; }
private:
  void driveSide(int pwmPin, int inA, int inB, float v);
};
