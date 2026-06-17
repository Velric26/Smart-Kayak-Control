#pragma once
#include <Arduino.h>

// Differential mix: linear velocity v + turn rate w -> left/right thrust.
// Scales both down on saturation to preserve turn authority (§4.1).
// Identical on mule and kayak.
inline void mix(float v, float w, float& left, float& right) {
  left  = v - w;
  right = v + w;
  float m = max(fabsf(left), fabsf(right));
  if (m > 1.0f) { left /= m; right /= m; }
}

// Slew-rate limiter for smooth thrust changes (§5). Call each tick.
inline float slew(float target, float current, float maxStep) {
  float d = target - current;
  if (d >  maxStep) d =  maxStep;
  if (d < -maxStep) d = -maxStep;
  return current + d;
}
