#pragma once
#include <Arduino.h>

// Operating modes (§6). Phase 1 implements BOOT/DISARMED/MANUAL/FAILSAFE;
// the autonomous modes are present as targets but gated until their
// sensors/controllers come online (Phases 2-6).
enum class Mode : uint8_t {
  BOOT,
  DISARMED,
  MANUAL,
  HEADING_HOLD,
  ANCHOR,
  ANCHOR_HEADING,
  FAILSAFE
};

const char* modeName(Mode m);

struct SMInputs {
  bool rcLinkOk      = false;  // RC fresh on critical channels
  bool batteryCritical = false;
  bool armRequest    = false;  // arm channel asserted
  bool sticksNeutral = true;   // throttle/steer near center
  int  modeSelect    = 0;      // 0=MANUAL,1=HEADING,2=ANCHOR,3=ANCHOR+HDG
};

// NOTE: the 3PDT hardware-bypass input — and the bumpless re-engage it drove
// (justReengaged) — are deferred until the switch is installed. GPIO13
// (PIN_BYPASS_SENSE) is reserved but NOT read in firmware. Re-add both here
// and in StateMachine.cpp when the 3PDT arrives (see docs/architecture.md §1.6).

class StateMachine {
public:
  void begin() { mode_ = Mode::BOOT; }
  void update(const SMInputs& in);
  Mode mode() const { return mode_; }

private:
  Mode mode_ = Mode::BOOT;
};
