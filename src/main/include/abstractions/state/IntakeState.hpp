#pragma once

#include "turbolib/state/TurboState.hpp"
#include <string>

enum class IntakeStateEnum { Intake, Stow };

inline constexpr std::string stateToString(IntakeStateEnum state) {
  switch (state) {
    case IntakeStateEnum::Stow:
      return "Stow";
    case IntakeStateEnum::Intake:
      return "Intake";
  }
}

class IntakeState : public TurboState<IntakeStateEnum> {
 public:
  explicit IntakeState(IntakeStateEnum initialState, ApplyFunc applyFunc)
      : TurboState(std::move(initialState), std::move(applyFunc)) {}
};