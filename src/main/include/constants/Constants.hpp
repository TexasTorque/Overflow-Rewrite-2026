// Texas Torque 1477
// Overflow 2026

#pragma once

#include "units/voltage.h"
#include "units/velocity.h"
#include "units/angular_velocity.h"

namespace DriveConstants {
inline constexpr units::meters_per_second_t kMaxSpeed = 3.75_mps;
inline constexpr units::radians_per_second_t kMaxAngularRate = 0.75_tps;
}  // namespace DriveConstants
#include "units/angular_velocity.h"

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

namespace ShooterConstants {
inline constexpr int kFlywheelMotorRightPort = 20;
inline constexpr int kFlywheelMotorLeftPort = 19;

inline constexpr double kFlywheelkP = 0.2;
inline constexpr double kFlywheelkI = 0;
inline constexpr double kFlywheelkD = 0;
inline constexpr double kFlywheelkS = 0.2;
inline constexpr double kFlywheelkV = 0.1122;

inline constexpr double kFlywheelTolerance = 50;

inline constexpr units::revolutions_per_minute_t kLayupRPM = 2900_rpm;
inline constexpr units::revolutions_per_minute_t kLaserRPM = 5000_rpm;
inline constexpr units::revolutions_per_minute_t kClimbRPM = 3600_rpm;
inline constexpr units::revolutions_per_minute_t kTrenchRPM = 3950_rpm;
inline constexpr units::revolutions_per_minute_t kIdleRPM = 2000_rpm;
}  // namespace ShooterConstants
