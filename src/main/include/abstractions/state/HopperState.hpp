// Texas Torque 1477
// Overflow 2026

#pragma once

#include <string>
#include <utility>
#include <magic_enum.hpp>
#include "turbolib/state/TurboState.hpp"

enum class HopperStateEnum { Intake, Outtake, Off };

inline std::string stateToString(HopperStateEnum state) {
  return std::string(magic_enum::enum_name(state));
}

class HopperState : public TurboState<HopperStateEnum> {
 public:
  explicit HopperState(HopperStateEnum initialState, ApplyFunc applyFunc)
      : TurboState(std::move(initialState), std::move(applyFunc)) {}
};
