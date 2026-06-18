#include <Arduino.h>
#include "hal/ESC_Driver.h"
#include "config.h"

int ESC_Driver::toMicros(float v) {
  v = constrain(v, -1.0f, 1.0f);
  return RC_US_NEUTRAL + (int)(v * (RC_US_MAX - RC_US_NEUTRAL));
}

void ESC_Driver::begin() {
  // ESP32Servo timer allocation (one per servo group).
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  escL_.setPeriodHertz(50);
  escR_.setPeriodHertz(50);
  escL_.attach(PIN_MOTOR_PWM_L, RC_US_MIN, RC_US_MAX);
  escR_.attach(PIN_MOTOR_PWM_R, RC_US_MIN, RC_US_MAX);
  disable();
  // NOTE: most bidirectional ESCs require an arming sequence at neutral
  // for ~1-2 s on power-up before they respond. Hold neutral here and
  // gate motion behind the arming state in the state machine.
}

// Same min-drive remap as the L298N path (skips the thruster's dead zone).
static float applyMinDrive(float v, float minDrive) {
  if (fabsf(v) < 0.02f) return 0.0f;
  float m = minDrive + (1.0f - minDrive) * fabsf(v);
  return (v < 0.0f) ? -m : m;
}

void ESC_Driver::setThrust(float left, float right) {
  if (MOTOR_L_INVERT) left  = -left;
  if (MOTOR_R_INVERT) right = -right;
  left  = applyMinDrive(left,  minDriveL);
  right = applyMinDrive(right, minDriveR);
  lastL = left; lastR = right;
  escL_.writeMicroseconds(toMicros(left));
  escR_.writeMicroseconds(toMicros(right));
}

void ESC_Driver::disable() {
  escL_.writeMicroseconds(RC_US_NEUTRAL);
  escR_.writeMicroseconds(RC_US_NEUTRAL);
  lastL = lastR = 0;
}
