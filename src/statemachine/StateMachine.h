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
  bool bypassManual  = false;  // 3PDT in MANUAL (MCU removed from loop)
  bool armRequest    = false;  // arm channel asserted
  bool sticksNeutral = true;   // throttle/steer near center
  int  modeSelect    = 0;      // 0=MANUAL,1=HEADING,2=ANCHOR,3=ANCHOR+HDG
};

class StateMachine {
public:
  void begin() { mode_ = Mode::BOOT; }
  void update(const SMInputs& in);
  Mode mode() const { return mode_; }

  // True on the tick where control just returned to the MCU
  // (bypass MANUAL->AUTO, or FAILSAFE->armed). Consumers reset
  // controllers / re-capture setpoints for bumpless transfer (§8.5).
  bool justReengaged() const { return reengaged_; }

private:
  Mode mode_ = Mode::BOOT;
  bool reengaged_ = false;
  bool wasBypassed_ = false;
};
