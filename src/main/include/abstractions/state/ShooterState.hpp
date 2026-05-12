#pragma once

#include "turbolib/state/TurboState.hpp"
#include <string>

enum class ShooterStateEnum { Off, Idle, Regression, Layup, Climb, Trench, Laser };

inline constexpr std::string stateToString(ShooterStateEnum state) {
  switch (state) {
    case ShooterStateEnum::Off:
      return "Off";
    case ShooterStateEnum::Idle:
      return "Idle";
    case ShooterStateEnum::Regression:
      return "Regression";
    case ShooterStateEnum::Layup:
      return "Layup";
    case ShooterStateEnum::Climb:
      return "Climb";
    case ShooterStateEnum::Trench:
      return "Trench";
    case ShooterStateEnum::Laser:
      return "Laser";
  }
}

class ShooterState : public TurboState<ShooterStateEnum> {
 public:
  explicit ShooterState(ShooterStateEnum initialState, ApplyFunc applyFunc)
      : TurboState(std::move(initialState), std::move(applyFunc)) {}
};