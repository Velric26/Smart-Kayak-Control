#pragma once
#include <Arduino.h>
#include <TinyGPSPlus.h>

// NEO-8M GPS reader on UART2. Wraps TinyGPSPlus: call update() often (each
// control tick) to pump bytes, then query fix/position. distanceBetween() and
// courseTo() (static, on TinyGPSPlus) give range + bearing for ANCHOR mode.
class GPS {
public:
  void begin();
  void update();   // drain UART into the parser; call frequently

  // A fix is "fresh" if we've decoded a valid location in the last 3 s.
  // (TinyGPSPlus accessors are non-const, so these can't be const either.)
  bool   hasFix() { return gps_.location.isValid() && gps_.location.age() < 3000; }
  double lat()    { return gps_.location.lat(); }
  double lon()    { return gps_.location.lng(); }
  int    sats()   { return gps_.satellites.isValid() ? gps_.satellites.value() : 0; }
  float  hdop()   { return gps_.hdop.isValid() ? (float)gps_.hdop.hdop() : 99.9f; }

  TinyGPSPlus& raw() { return gps_; }   // for distanceBetween()/courseTo()

private:
  TinyGPSPlus gps_;
};
