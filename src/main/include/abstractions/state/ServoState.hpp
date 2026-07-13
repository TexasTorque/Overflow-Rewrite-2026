// Texas Torque 1477
// Overflow 2026

#pragma once

#include "turbolib/state/TurboState.hpp"

enum class ServoStateEnum { Idle, Laser };

class ServoState : public TurboState<ServoStateEnum> {
 public:
  using TurboState<ServoStateEnum>::TurboState;
};
