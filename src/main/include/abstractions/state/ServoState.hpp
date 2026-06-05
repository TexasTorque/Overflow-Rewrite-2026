// Texas Torque 1477
// Overflow 2026

#pragma once

#include <string>
#include <utility>
#include <magic_enum.hpp>
#include "turbolib/state/TurboState.hpp"

enum class ServoStateEnum { Idle, Laser };

inline std::string stateToString(ServoStateEnum state) {
  return std::string(magic_enum::enum_name(state));
}

class ServoState : public TurboState<ServoStateEnum> {
 public:
  explicit ServoState(ServoStateEnum initialState, ApplyFunc applyFunc)
      : TurboState(std::move(initialState), std::move(applyFunc)) {}
};
