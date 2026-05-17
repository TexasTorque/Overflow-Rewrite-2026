// Texas Torque 1477
// Overflow 2026

#pragma once

#include "magic_enum.hpp"
#include "turbolib/state/TurboState.hpp"
#include <string>
#include <utility>

enum class ShooterStateEnum { Off, Idle, Regression, Layup, Climb, Trench, Laser };

inline std::string stateToString(ShooterStateEnum state) {
  return std::string(magic_enum::enum_name(state));
}

class ShooterState : public TurboState<ShooterStateEnum> {
 public:
  explicit ShooterState(ShooterStateEnum initialState, ApplyFunc applyFunc)
      : TurboState(std::move(initialState), std::move(applyFunc)) {}
};
