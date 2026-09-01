// Texas Torque 1477
// Overflow 2026

#pragma once

#include "turbolib/state/TurboState.hpp"

enum class IntakeStateEnum { Intake, Stow, Outtake, SlowPullup, PullIn, Agitation };

class IntakeState : public TurboState<IntakeStateEnum> {
 public:
  using TurboState<IntakeStateEnum>::TurboState;
};
