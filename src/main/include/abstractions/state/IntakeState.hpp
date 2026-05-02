#pragma once

#include "turbolib/state/TurboState.hpp"
#include <string>

enum class IntakeStateEnum { Intake, Stow };

namespace std {
inline std::string to_string(IntakeStateEnum state) {
  switch (state) {
    case IntakeStateEnum::Intake:
      return "Intake";
    case IntakeStateEnum::Stow:
      return "Stow";
    default:
      return "Unknown";
  }
}
}  // namespace std

class IntakeState : public TurboState<IntakeStateEnum> {
 public:
  explicit IntakeState(IntakeStateEnum initialState, ApplyFunc applyFunc)
      : TurboState(std::move(initialState), std::move(applyFunc)) {}
};