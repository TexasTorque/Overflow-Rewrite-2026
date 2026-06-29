// Texas Torque 1477
// Overflow 2026

#pragma once

#include "turbolib/state/TurboState.hpp"

enum class ShooterStateEnum { Off, Idle, Regression, Layup, Climb, Trench, Laser };

class ShooterState : public TurboState<ShooterStateEnum> {
 public:
  using TurboState<ShooterStateEnum>::TurboState;
};
