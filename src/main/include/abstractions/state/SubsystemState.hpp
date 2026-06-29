// Texas Torque 1477
// Overflow 2026

#pragma once

#include <string>
#include <magic_enum.hpp>

template <typename StateEnum>
inline std::string stateToString(StateEnum state) {
  return std::string(magic_enum::enum_name(state));
}
