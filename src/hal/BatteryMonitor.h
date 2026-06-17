#pragma once
#include <Arduino.h>

// 2S Li-ion pack monitor on an ADC1 pin. Averaged, divider-corrected.
class BatteryMonitor {
public:
  void  begin();
  void  update();              // call periodically (e.g. 5-10 Hz)
  float volts() const { return v_; }
  bool  warn() const;          // <= BATT_WARN_V
  bool  critical() const;      // <= BATT_CRITICAL_V -> failsafe
private:
  float v_ = 8.4f;             // optimistic init so we don't trip on boot
};
