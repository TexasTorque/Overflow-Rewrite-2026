// Texas Torque 1477
// Overflow 2026

#pragma once

#include "frc/LEDPattern.h"
#include "frc/geometry/Transform3d.h"
#include "frc/util/Color.h"
#include "units/voltage.h"
#include "units/velocity.h"
#include "units/angular_velocity.h"

namespace DriveConstants {
inline constexpr units::meters_per_second_t kMaxSpeed = 3.6_mps;
inline constexpr units::radians_per_second_t kMaxAngularRate = 1.00_tps;
}  // namespace DriveConstants

namespace PerceptionConstants {
inline constexpr frc::Transform3d kShooterRightCamTransform{
    -10.88111_in, -7.985419_in, 10.326243_in, {0_deg, -22.5_deg, 180_deg}};
inline constexpr frc::Transform3d kHopperLeftCamTransform{
    2.50874_in, 12.29184_in, 6.912581_in, {0_deg, -25_deg, 90_deg}};
inline constexpr frc::Transform3d kHopperRightCamTransform{
    2.50874_in, -12.29184_in, 6.912581_in, {0_deg, -25_deg, -90_deg}};
inline constexpr frc::Transform3d kShooterLeftCamTransform{
    -10.88111_in, 8.002581_in, 10.326243_in, {0_deg, -22.5_deg, 180_deg}};

inline constexpr frc::Pose2d kRedHubPose{11.92_m, 4_m, 0_rad};
inline constexpr frc::Pose2d kBlueHubPose{4.625_m, 4_m, 0_rad};
}  // namespace PerceptionConstants

namespace IntakeConstants {
inline constexpr int kIntakeRotaryMotorPort = 13;
inline constexpr int kIntakeRollerMotorLeftPort = 14;
inline constexpr int kIntakeRollerMotorRightPort = 15;

inline constexpr double kRotaryUpPosition = 0.0;
inline constexpr double kRotaryDownPosition = 24.6;
inline constexpr double kRotarySlowZeroPosition = 10.5;

inline constexpr units::volt_t kIntakeVoltage = -10_V;
inline constexpr units::volt_t kOuttakeVoltage = 12_V;
}  // namespace IntakeConstants

namespace HopperConstants {
inline constexpr int kHopperMotorPort = 16;

inline constexpr units::volt_t kIntakeVoltage = -6_V;
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
inline constexpr units::revolutions_per_minute_t kIdleRPM = 0_rpm;
}  // namespace ShooterConstants

namespace DebugConstants {
inline constexpr bool kDebugSignalLogging = true;
}  // namespace DebugConstants

namespace ServoConstants {
inline constexpr int kServoLeftPort = 7;
inline constexpr int kServoRightPort = 8;

inline constexpr double kServoIdlePos = 0.35;
inline constexpr double kServoLaserPos = 0.1;
}  // namespace ServoConstants

namespace LEDConstants {
inline constexpr int kLEDPort = 9;
inline constexpr int kLEDLength = 34;

inline constexpr int kFrontLength = 18;
inline constexpr int kBackLength = kLEDLength - kFrontLength;

// IDLE
inline const std::array<frc::Color, 2> kIdleColors = {frc::Color{1.0, 0.0, 0.1}, frc::Color{0.1, 0.0, 1.0}};
inline frc::LEDPattern kIdle =
    frc::LEDPattern::Gradient(frc::LEDPattern::kContinuous, kIdleColors).ScrollAtRelativeSpeed(units::hertz_t{0.5});

// RUNNING
inline frc::LEDPattern kRunning = frc::LEDPattern::Solid(frc::Color::kRed);

// INTAKING
inline const std::array<std::pair<double, frc::Color>, 4> kIntakeFrontSteps = {
    std::pair{0.0, frc::Color::kYellow}, std::pair{0.22, frc::Color::kBlack}, std::pair{0.5, frc::Color::kOrange},
    std::pair{0.72, frc::Color::kBlack}};
inline const std::array<std::pair<double, frc::Color>, 6> kIntakeBackSteps = {
    std::pair{0.0, frc::Color::kYellow}, std::pair{0.12, frc::Color::kBlack},  std::pair{0.33, frc::Color::kOrange},
    std::pair{0.45, frc::Color::kBlack}, std::pair{0.67, frc::Color::kYellow}, std::pair{0.79, frc::Color::kBlack}};
inline frc::LEDPattern kIntakeFront =
    frc::LEDPattern::Steps(kIntakeFrontSteps).ScrollAtRelativeSpeed(units::hertz_t{1.2});
inline frc::LEDPattern kIntakeBack =
    frc::LEDPattern::Steps(kIntakeBackSteps).ScrollAtRelativeSpeed(units::hertz_t{1.6});

// SHOOTER SPINNING UP
inline frc::LEDPattern kSpinUp = frc::LEDPattern::Solid(frc::Color::kOrange).Blink(0.3_s);

// ACTIVELY SHOOTING
inline frc::LEDPattern kShooting = frc::LEDPattern::Solid(frc::Color::kGreen).Blink(0.1_s);

// AUTO ALIGN
inline const std::array<std::pair<double, frc::Color>, 4> kAutoAlignSteps = {
    std::pair{0.0, frc::Color::kBlue}, std::pair{0.25, frc::Color::kBlack}, std::pair{0.5, frc::Color::kBlue},
    std::pair{0.75, frc::Color::kBlack}};

inline frc::LEDPattern kAutoAlignRight =
    frc::LEDPattern::Steps(kAutoAlignSteps).ScrollAtRelativeSpeed(units::hertz_t{-1.5});

inline frc::LEDPattern kAutoAlignLeft =
    frc::LEDPattern::Steps(kAutoAlignSteps).ScrollAtRelativeSpeed(units::hertz_t{1.5});
}  // namespace LEDConstants
