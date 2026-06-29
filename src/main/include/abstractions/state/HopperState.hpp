// Texas Torque 1477
// Overflow 2026

#pragma once

#include "turbolib/state/TurboState.hpp"

enum class HopperStateEnum { Intake, Outtake, Off };

class HopperState : public TurboState<HopperStateEnum> {
 public:
  using TurboState<HopperStateEnum>::TurboState;
};
