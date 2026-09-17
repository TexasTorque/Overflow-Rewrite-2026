// Texas Torque 1477
// Overflow 2026

#pragma once

#include "turbolib/state/TurboState.hpp"

enum class ShooterStateEnum { Off, Idle, Prespin, Regression, Layup, Climb, Trench, Laser, Debug };

class ShooterState : public TurboState<ShooterStateEnum> {
 public:
  using TurboState<ShooterStateEnum>::TurboState;
};
