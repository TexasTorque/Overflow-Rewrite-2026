#pragma once

#include "units/voltage.h"
#include "units/angular_velocity.h"

namespace IntakeConstants {
inline constexpr int kIntakeRotaryMotorPort = 13;
inline constexpr int kIntakeRollerMotorRightPort = 14;
inline constexpr int kIntakeRollerMotorLeftPort = 15;

inline constexpr double kRotaryUpPosition = 0.0;
inline constexpr double kRotaryDownPosition = 24.6;

inline constexpr units::volt_t kIntakeVoltage = 10_V;
} // namespace IntakeConstants

namespace ShooterConstants {
  inline constexpr int kFlywheelMotorRightPort = 20;
  inline constexpr int kFlywheelMotorLeftPort = 19;

  inline constexpr double kFlywheelkP = 1;
  inline constexpr double kFlywheelkI = 0;
  inline constexpr double kFlywheelkD = 0;
  inline constexpr double kFlywheelkS = 1;
  inline constexpr double kFlywheelkV = 1;

  inline constexpr double kFlywheelTolerance = 50;

  inline constexpr units::revolutions_per_minute_t kLayupV = 2900_rpm;
  inline constexpr units::revolutions_per_minute_t kLaserV = 5000_rpm;
  inline constexpr units::revolutions_per_minute_t kClimbV = 3600_rpm;
  inline constexpr units::revolutions_per_minute_t kTrenchV = 3950_rpm;
  inline constexpr units::revolutions_per_minute_t kIdleV = 2000_rpm;
}