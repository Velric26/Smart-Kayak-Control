#pragma once
#include <Arduino.h>
#include "config.h"

// 3PDT bypass-switch position sense (third pole -> GPIO13).
// Until the switch is installed the pin is unconnected; INPUT_PULLUP
// makes it read HIGH, which we define as AUTO (MCU in command) — the
// current reality on the mule. Wire the 3PDT so MANUAL pulls it LOW.
//
// When isManual() goes false->... (i.e. control RETURNS to the MCU),
// main resets the controllers for bumpless transfer (§8.5).
class BypassSense {
public:
  void begin() { pinMode(PIN_BYPASS_SENSE, INPUT_PULLUP); }

  // Debounced read. LOW == MANUAL (RC drives ESCs directly).
  bool isManual() {
    bool raw = (digitalRead(PIN_BYPASS_SENSE) == LOW);
    uint32_t now = millis();
    if (raw != lastRaw_) { lastRaw_ = raw; changedMs_ = now; }
    if (now - changedMs_ > 30) state_ = raw;   // 30 ms debounce
    return state_;
  }
  bool isAuto() { return !isManual(); }

private:
  bool lastRaw_ = false, state_ = false;
  uint32_t changedMs_ = 0;
};
