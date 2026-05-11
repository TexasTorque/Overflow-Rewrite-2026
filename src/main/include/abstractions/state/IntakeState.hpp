#pragma once

#include <string>
#include "magic_enum.hpp"
#include "turbolib/state/TurboState.hpp"

enum class IntakeStateEnum { Intake, Stow, Outtake, SlowPullup };

inline std::string stateToString(IntakeStateEnum state) {
  return std::string(magic_enum::enum_name(state));
}

class IntakeState : public TurboState<IntakeStateEnum> {
 public:
  explicit IntakeState(IntakeStateEnum initialState, ApplyFunc applyFunc)
      : TurboState(std::move(initialState), std::move(applyFunc)) {}
};
