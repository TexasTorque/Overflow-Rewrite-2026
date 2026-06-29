// Texas Torque 1477
// Overflow 2026

#pragma once

#include "turbolib/state/TurboState.hpp"

enum class GateStateEnum { Off, On, Outtake };

class GateState : public TurboState<GateStateEnum> {
 public:
  using TurboState<GateStateEnum>::TurboState;
};
