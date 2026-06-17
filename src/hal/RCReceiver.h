#pragma once
#include <Arduino.h>

// Parallel (high-impedance) RC capture for the DS600 channels.
// Interrupt-per-pin pulse-width measurement. The MCU only *listens* —
// it never sits in series on the ESC path (§8.1).
//
// NOTE: for production hardening you can move to the ESP32 RMT peripheral
// or direct GPIO-register reads in the ISR; digitalRead-in-ISR is used
// here for clarity and is fine for a handful of channels.

enum RCChannel { RC_LEFT = 0, RC_RIGHT, RC_MODE, RC_ARM, RC_COUNT };

class RCReceiver {
public:
  void begin();

  // Raw pulse width in microseconds (last valid capture), or 0 if none.
  uint16_t pulseUs(RCChannel ch) const;

  // Normalized [-1, +1] about neutral, with deadband applied.
  float norm(RCChannel ch) const;

  // True if this channel has produced a valid pulse within RC_TIMEOUT_US.
  bool fresh(RCChannel ch) const;

  // True if ALL control channels are fresh (i.e. link is alive).
  bool linkOk() const;
};
