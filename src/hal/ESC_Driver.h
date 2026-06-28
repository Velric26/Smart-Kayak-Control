#pragma once
#include "hal/MotorDriver.h"
#include <ESP32Servo.h>

// Dual bidirectional brushed/brushless ESC driver (mule rover + kayak).
// Outputs 50 Hz "analog servo mode" PWM: 1000 us full reverse, 1500 us neutral,
// 2000 us full forward on GPIO25/26 (one signal per ESC). The ESC auto-adapts
// its endpoints and arms when it sees a stable 1500 us neutral at power-on.
class ESC_Driver : public MotorDriver {
public:
  void begin() override;
  void setThrust(float left, float right) override;
  void setRaw(float left, float right) override;
  void disable() override;
  const char* name() const override { return "ESC"; }
private:
  Servo escL_, escR_;
  static int toMicros(float v); // [-1,1] -> [1000,2000]
};
