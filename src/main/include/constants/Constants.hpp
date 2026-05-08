#pragma once

#include "units/voltage.h"
#include "units/velocity.h"
#include "units/angular_velocity.h"

namespace DriveConstants {
inline constexpr units::meters_per_second_t kMaxSpeed = 4.5_mps;
inline constexpr units::radians_per_second_t kMaxAngularRate = 2.0_tps;
}  // namespace DriveConstants

namespace IntakeConstants {
inline constexpr int kIntakeRotaryMotorPort = 13;
inline constexpr int kIntakeRollerMotorRightPort = 14;
inline constexpr int kIntakeRollerMotorLeftPort = 15;

inline constexpr double kRotaryUpPosition = 0.0;
inline constexpr double kRotaryDownPosition = 24.6;
inline constexpr double kRotarySlowZeroPosition = 12.5;

inline constexpr units::volt_t kIntakeVoltage = 10_V;
inline constexpr units::volt_t kOuttakeVoltage = -12_V;
}  // namespace IntakeConstants

namespace HopperConstants {
inline constexpr int kHopperMotorPort = 16;

inline constexpr units::volt_t kIntakeVoltage = -10_V;
inline constexpr units::volt_t kOuttakeVoltage = 12_V;
}  // namespace HopperConstants

namespace GateConstants {
inline constexpr int kGateRightMotorPort = 18;
inline constexpr int kGateLeftMotorPort = 17;

inline constexpr units::volt_t kGateVoltage = 10_V;
inline constexpr units::volt_t kOuttakeVoltage = -10_V;
}  // namespace GateConstants