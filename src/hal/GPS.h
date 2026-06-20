#pragma once
#include <Arduino.h>
#include <stdlib.h>
#include <math.h>
#include <TinyGPSPlus.h>

// NEO-8M GPS reader on UART2. Wraps TinyGPSPlus: call update() often (each
// control tick) to pump bytes, then query fix/position. distanceBetween() and
// courseTo() (static, on TinyGPSPlus) give range + bearing for ANCHOR mode.
// The module is configured (tools/gps_config.py) for 5 Hz + GST, which gives a
// real per-axis position sigma we turn into a horizontal accuracy estimate.
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
  // Course-over-ground (deg, true north) and speed — for compass alignment
  // checks: drive straight, COG should match the compass heading. -1 if N/A.
  float  course() { return gps_.course.isValid() ? (float)gps_.course.deg() : -1.0f; }
  float  speedMs() { return gps_.speed.isValid() ? (float)gps_.speed.mps() : 0.0f; }

  // Horizontal position accuracy (m, 1-sigma) from the NMEA GST stdLat/stdLon.
  // This is a real error estimate (unlike HDOP, which is only geometry). Big
  // (99.9) until GST carries a valid fix. Gate ANCHOR capture on this.
  float  accM() {
    // Stale GST (module stopped sending) must not read as "good accuracy".
    if (!gstLat_.isValid() || !gstLon_.isValid()) return 99.9f;
    if (gstLat_.age() > 3000 || gstLon_.age() > 3000) return 99.9f;
    float a = atof(gstLat_.value()), b = atof(gstLon_.value());
    if (a <= 0.0f && b <= 0.0f) return 99.9f;
    return sqrtf(a * a + b * b);
  }

  TinyGPSPlus& raw() { return gps_; }   // for distanceBetween()/courseTo()

private:
  TinyGPSPlus  gps_;
  TinyGPSCustom gstLat_, gstLon_;   // GNGST term 6 (stdLat), term 7 (stdLon)
};
