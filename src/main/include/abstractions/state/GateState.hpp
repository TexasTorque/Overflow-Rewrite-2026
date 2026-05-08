#pragma once

#include <string>
#include <utility>
#include <magic_enum/magic_enum.hpp>
#include "turbolib/state/TurboState.hpp"

enum class GateStateEnum { Off, On, Outtake };

inline std::string stateToString(GateStateEnum state) {
  return std::string(magic_enum::enum_name(state));
}

class GateState : public TurboState<GateStateEnum> {
 public:
  explicit GateState(GateStateEnum initialState, ApplyFunc applyFunc)
      : TurboState(std::move(initialState), std::move(applyFunc)) {}
};
