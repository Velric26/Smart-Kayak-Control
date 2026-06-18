#include <Arduino.h>
#include "hal/L298N_Driver.h"
#include "config.h"

void L298N_Driver::begin() {
  pinMode(PIN_DIR_IN1, OUTPUT);
  pinMode(PIN_DIR_IN2, OUTPUT);
  pinMode(PIN_DIR_IN3, OUTPUT);
  pinMode(PIN_DIR_IN4, OUTPUT);
  pinMode(PIN_MOTOR_PWM_L, OUTPUT);
  pinMode(PIN_MOTOR_PWM_R, OUTPUT);
  // Optional: raise PWM frequency above audible range. Safe to omit;
  // default analogWrite frequency drives a brushed motor fine.
  // analogWriteFrequency(PIN_MOTOR_PWM_L, 20000);
  // analogWriteFrequency(PIN_MOTOR_PWM_R, 20000);
  disable();
}

void L298N_Driver::driveSide(int pwmPin, int inA, int inB, float v) {
  v = constrain(v, -1.0f, 1.0f);
  bool forward = (v >= 0.0f);
  digitalWrite(inA, forward ? HIGH : LOW);
  digitalWrite(inB, forward ? LOW  : HIGH);
  analogWrite(pwmPin, (int)(fabsf(v) * 255.0f)); // 8-bit duty
}

// Remap so a zero command stays off and any non-zero command produces at
// least MOTOR_MIN_DRIVE (skips the no-torque whine zone).
static float applyMinDrive(float v) {
  if (fabsf(v) < 0.02f) return 0.0f;
  float m = MOTOR_MIN_DRIVE + (1.0f - MOTOR_MIN_DRIVE) * fabsf(v);
  return (v < 0.0f) ? -m : m;
}

void L298N_Driver::setThrust(float left, float right) {
  if (MOTOR_L_INVERT) left  = -left;
  if (MOTOR_R_INVERT) right = -right;
  left  = applyMinDrive(left);
  right = applyMinDrive(right);
  lastL = left; lastR = right;
  driveSide(PIN_MOTOR_PWM_L, PIN_DIR_IN1, PIN_DIR_IN2, left);
  driveSide(PIN_MOTOR_PWM_R, PIN_DIR_IN3, PIN_DIR_IN4, right);
}

void L298N_Driver::disable() {
  // Coast: both inputs low, zero PWM.
  digitalWrite(PIN_DIR_IN1, LOW); digitalWrite(PIN_DIR_IN2, LOW);
  digitalWrite(PIN_DIR_IN3, LOW); digitalWrite(PIN_DIR_IN4, LOW);
  analogWrite(PIN_MOTOR_PWM_L, 0);
  analogWrite(PIN_MOTOR_PWM_R, 0);
  lastL = lastR = 0;
}
