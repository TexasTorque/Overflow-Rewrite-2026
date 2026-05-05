#pragma once

#include <string>
#include <utility>
#include "turbolib/state/TurboState.hpp"

enum class GateStateEnum { Off, On, Outtake };

inline constexpr std::string stateToString(GateStateEnum state) {
  switch (state) {
    case GateStateEnum::Off:
      return "Off";
    case GateStateEnum::On:
      return "On";
    case GateStateEnum::Outtake:
      return "Outtake";
  }
}

class GateState : public TurboState<GateStateEnum> {
 public:
  explicit GateState(GateStateEnum initialState, ApplyFunc applyFunc)
      : TurboState(std::move(initialState), std::move(applyFunc)) {}
};