#include "hal/RCReceiver.h"
#include "config.h"

namespace {
  struct Cap {
    volatile uint32_t riseUs   = 0;
    volatile uint16_t widthUs  = 0;
    volatile uint32_t updateUs = 0;
    int pin = -1;
  };
  Cap g_cap[RC_COUNT];

  // Single argumented ISR shared by all channels.
  void IRAM_ATTR isr(void* arg) {
    Cap* c = static_cast<Cap*>(arg);
    uint32_t now = micros();
    if (digitalRead(c->pin)) {           // rising edge
      c->riseUs = now;
    } else {                             // falling edge -> pulse complete
      uint32_t w = now - c->riseUs;
      if (w >= (uint32_t)RC_US_VALID_LO && w <= (uint32_t)RC_US_VALID_HI) {
        c->widthUs  = (uint16_t)w;
        c->updateUs = now;
      }
    }
  }
}

void RCReceiver::begin() {
  const int pins[RC_COUNT] = { PIN_RC_LEFT, PIN_RC_RIGHT, PIN_RC_MODE, PIN_RC_ARM };
  for (int i = 0; i < RC_COUNT; ++i) {
    g_cap[i].pin = pins[i];
    pinMode(pins[i], INPUT);             // DS600 drives push-pull; no pull needed
    attachInterruptArg(digitalPinToInterrupt(pins[i]), isr, &g_cap[i], CHANGE);
  }
}

uint16_t RCReceiver::pulseUs(RCChannel ch) const {
  noInterrupts();
  uint16_t w = g_cap[ch].widthUs;
  interrupts();
  return w;
}

bool RCReceiver::fresh(RCChannel ch) const {
  noInterrupts();
  uint32_t last = g_cap[ch].updateUs;
  uint16_t w    = g_cap[ch].widthUs;
  interrupts();
  if (w == 0) return false;
  return (micros() - last) < RC_TIMEOUT_US;
}

float RCReceiver::norm(RCChannel ch) const {
  uint16_t us = pulseUs(ch);
  if (us == 0) return 0.0f;
  float n = (float)((int)us - RC_US_NEUTRAL) /
            (float)(RC_US_MAX - RC_US_NEUTRAL);
  n = constrain(n, -1.0f, 1.0f);
  if (fabsf(n) < RC_DEADBAND) return 0.0f;
  return n;
}

bool RCReceiver::linkOk() const {
  // Treat the two thrust channels + arm as the critical set.
  return fresh(RC_LEFT) && fresh(RC_RIGHT) && fresh(RC_ARM);
}
