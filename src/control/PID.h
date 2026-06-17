#pragma once
#include <Arduino.h>

// PID with conditional-integration anti-windup and output clamping (§5).
// Gains are runtime-adjustable (expose over telemetry to tune live).
class PID {
public:
  float kp = 0, ki = 0, kd = 0;
  float iMax = 1.0f;            // integrator clamp
  float outMin = -1.0f, outMax = 1.0f;

  void setGains(float p, float i, float d) { kp = p; ki = i; kd = d; }
  void reset() { integ_ = 0; prevE_ = 0; havePrev_ = false; }

  float update(float error, float dt) {
    if (dt <= 0) return lastOut_;
    float deriv = havePrev_ ? (error - prevE_) / dt : 0.0f;
    prevE_ = error; havePrev_ = true;

    float out = kp * error + ki * integ_ + kd * deriv;

    // Anti-windup: only integrate when the output is NOT saturated.
    if (out > outMax)      out = outMax;
    else if (out < outMin) out = outMin;
    else                   integ_ += error * dt;
    integ_ = constrain(integ_, -iMax, iMax);

    lastOut_ = constrain(out, outMin, outMax);
    return lastOut_;
  }

private:
  float integ_ = 0, prevE_ = 0, lastOut_ = 0;
  bool  havePrev_ = false;
};

// Shortest-path heading error in degrees, result in (-180, 180].
inline float headingError(float setpointDeg, float actualDeg) {
  float e = setpointDeg - actualDeg;
  while (e > 180.0f)  e -= 360.0f;
  while (e <= -180.0f) e += 360.0f;
  return e;
}
