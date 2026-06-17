#include "statemachine/StateMachine.h"

const char* modeName(Mode m) {
  switch (m) {
    case Mode::BOOT:           return "BOOT";
    case Mode::DISARMED:       return "DISARMED";
    case Mode::MANUAL:         return "MANUAL";
    case Mode::HEADING_HOLD:   return "HEADING_HOLD";
    case Mode::ANCHOR:         return "ANCHOR";
    case Mode::ANCHOR_HEADING: return "ANCHOR_HEADING";
    case Mode::FAILSAFE:       return "FAILSAFE";
  }
  return "?";
}

void StateMachine::update(const SMInputs& in) {
  reengaged_ = false;

  // ---- Highest precedence: hardware bypass (§7 level 0) -------------
  // If the 3PDT hands control to the RC directly, the MCU stands down.
  // Coming back from bypass requires a fresh arm (handled below) and
  // flags a re-engage so controllers reset.
  if (in.bypassManual) {
    wasBypassed_ = true;
    mode_ = Mode::DISARMED;
    return;
  }

  // ---- Safety override (§7 level 1) ---------------------------------
  if (!in.rcLinkOk || in.batteryCritical) {
    mode_ = Mode::FAILSAFE;
    return;
  }

  switch (mode_) {
    case Mode::BOOT:
      mode_ = Mode::DISARMED;
      break;

    case Mode::FAILSAFE:
      // Recover only when conditions clear AND operator re-arms.
      if (in.rcLinkOk && !in.batteryCritical && !in.armRequest)
        mode_ = Mode::DISARMED;
      break;

    case Mode::DISARMED:
      // Arm only with sticks centered (prevents arm-lurch).
      if (in.armRequest && in.sticksNeutral) {
        mode_ = Mode::MANUAL;
        if (wasBypassed_) { reengaged_ = true; wasBypassed_ = false; }
      }
      break;

    case Mode::MANUAL:
    case Mode::HEADING_HOLD:
    case Mode::ANCHOR:
    case Mode::ANCHOR_HEADING:
      if (!in.armRequest) { mode_ = Mode::DISARMED; break; }
      // Mode selection via RC. Autonomous modes are accepted here but
      // remain inert until their controllers are enabled (Phases 2-6).
      switch (in.modeSelect) {
        case 1:  mode_ = Mode::HEADING_HOLD;   break;
        case 2:  mode_ = Mode::ANCHOR;         break;
        case 3:  mode_ = Mode::ANCHOR_HEADING; break;
        default: mode_ = Mode::MANUAL;         break;
      }
      break;
  }
}
