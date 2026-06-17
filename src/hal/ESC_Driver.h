#pragma once
#include "hal/MotorDriver.h"
#include <ESP32Servo.h>

// Dual bidirectional brushed/brushless ESC driver (kayak / future mule).
// Outputs standard servo PWM: 1000 us full reverse, 1500 us neutral,
// 2000 us full forward. Uses the SAME GPIOs as the L298N enables, so the
// hardware migration is just moving two signal wires (§1.2a).
class ESC_Driver : public MotorDriver {
public:
  void begin() override;
  void setThrust(float left, float right) override;
  void disable() override;
  const char* name() const override { return "ESC"; }
private:
  Servo escL_, escR_;
  static int toMicros(float v); // [-1,1] -> [1000,2000]
};
