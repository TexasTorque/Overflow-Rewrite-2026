#pragma once

#include <string>
#include <utility>
#include "turbolib/state/TurboState.hpp"

enum class HopperStateEnum { Intake, Outtake, Off };

inline constexpr std::string stateToString(HopperStateEnum state) {
  switch (state) {
    case HopperStateEnum::Intake:
      return "Intake";
    case HopperStateEnum::Off:
      return "Off";
    case HopperStateEnum::Outtake:
      return "Outtake";
  }
}

class HopperState : public TurboState<HopperStateEnum> {
 public:
  explicit HopperState(HopperStateEnum initialState, ApplyFunc applyFunc)
      : TurboState(std::move(initialState), std::move(applyFunc)) {}
};