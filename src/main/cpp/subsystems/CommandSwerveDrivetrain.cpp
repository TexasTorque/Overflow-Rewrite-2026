// Texas Torque 1477
// Overflow 2026

#include "subsystems/CommandSwerveDrivetrain.h"
#include <frc/RobotController.h>
#include <memory>
#include <utility>
#include "frc/smartdashboard/SmartDashboard.h"

using namespace subsystems;

void CommandSwerveDrivetrain::Periodic() {
  /*
   * Periodically try to apply the operator perspective.
   * If we haven't applied the operator perspective before, then we should apply it regardless of DS state.
   * This allows us to correct the perspective in case the robot code restarts mid-match.
   * Otherwise, only check and apply the operator perspective if the DS is disabled.
   * This ensures driving behavior doesn't change until an explicit disable event occurs during testing.
   */
  if (!m_hasAppliedOperatorPerspective || frc::DriverStation::IsDisabled()) {
    auto const allianceColor = frc::DriverStation::GetAlliance();
    if (allianceColor) {
      SetOperatorPerspectiveForward(*allianceColor == frc::DriverStation::Alliance::kRed
                                        ? kRedAlliancePerspectiveRotation
                                        : kBlueAlliancePerspectiveRotation);
      m_hasAppliedOperatorPerspective = true;
    }
  }

  frc::SmartDashboard::PutNumber("Distance from Hub", GetDistanceToHub().value());
}

void CommandSwerveDrivetrain::StartSimThread() {
  m_lastSimTime = utils::GetCurrentTime();

  /* Run simulation at a faster rate so PID gains behave more reasonably */
  m_simNotifier = std::make_unique<frc::Notifier>([this] {
    units::second_t const currentTime = utils::GetCurrentTime();
    auto const deltaTime = currentTime - m_lastSimTime;
    m_lastSimTime = currentTime;

    /* use the measured time delta, get battery voltage from WPILib */
    UpdateSimState(deltaTime, frc::RobotController::GetBatteryVoltage());
  });
  m_simNotifier->StartPeriodic(kSimLoopPeriod);
}

void CommandSwerveDrivetrain::ConfigurePathPlanner() {
  pathplanner::RobotConfig config = pathplanner::RobotConfig::fromGUISettings();

  pathplanner::AutoBuilder::configure(
      [this]() { return GetState().Pose; }, [this](frc::Pose2d pose) { ResetPose(pose); },
      [this]() { return GetState().Speeds; },
      [this](frc::ChassisSpeeds const& speeds, pathplanner::DriveFeedforwards const& feedforwards) {
        return SetControl(m_pathApplyRobotSpeeds.WithSpeeds(frc::ChassisSpeeds::Discretize(speeds, 20_ms))
                              .WithWheelForceFeedforwardsX(feedforwards.robotRelativeForcesX)
                              .WithWheelForceFeedforwardsY(feedforwards.robotRelativeForcesY));
      },
      std::make_shared<pathplanner::PPHolonomicDriveController>(pathplanner::PIDConstants(3.0, 0.0, 0.0),
                                                                pathplanner::PIDConstants(3.0, 0.0, 0.0)),
      std::move(config),
      [] {
        auto const alliance = frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kBlue);
        return alliance == frc::DriverStation::Alliance::kRed;
      },
      this);
}
