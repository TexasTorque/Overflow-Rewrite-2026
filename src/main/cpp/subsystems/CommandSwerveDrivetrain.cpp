// Texas Torque 1477
// Overflow 2026

#include "subsystems/CommandSwerveDrivetrain.h"
#include <limits>
#include <frc/RobotController.h>
#include <memory>
#include <utility>
#include <networktables/IntegerTopic.h>

using namespace subsystems;

void CommandSwerveDrivetrain::Periodic() {
  m_perceptionSubsystem->UpdateHeading(GetState().Pose.Rotation());

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

    /* Move the simulated cameras with the robot so they observe real motion. */
    if (m_perceptionSubsystem)
      m_perceptionSubsystem->UpdateSim(GetState().Pose);
  });
  m_simNotifier->StartPeriodic(kSimLoopPeriod);
}

void CommandSwerveDrivetrain::PublishAutoAlign(const frc::Rotation2d& rawTarget, units::radians_per_second_t omega,
                                               units::radian_t filterStep, units::degree_t rawFilteredErr) {
  const frc::Rotation2d heading = GetState().Pose.Rotation();

  m_rawTargetPub.Set(rawTarget.Degrees().value());
  m_filteredTargetPub.Set(m_filteredTarget.Degrees().value());
  m_errorPub.Set((m_filteredTarget - heading).Degrees().value());
  m_pidErrorPub.Set(m_thetaController.GetError());
  m_omegaPub.Set(omega.value());
  m_headingDegPub.Set(heading.Degrees().value());
  m_filterStepDegPub.Set(units::degree_t{filterStep}.value());
  m_rawFilteredErrDegPub.Set(rawFilteredErr.value());
  m_filterInitializedPub.Set(m_filterInitialized);

  auto hubTarget = m_perceptionSubsystem ? m_perceptionSubsystem->GetNearestHubTarget() : std::nullopt;

  m_hasTargetPub.Set(hubTarget.has_value());

  constexpr double kNoData = std::numeric_limits<double>::quiet_NaN();

  if (hubTarget.has_value()) {
    m_tagIdPub.Set(hubTarget->id);
    m_tagYawDegPub.Set(hubTarget->yaw.value());
    m_tagDistanceMetersPub.Set(hubTarget->distance.value());
    m_tagAmbiguityPub.Set(hubTarget->ambiguity);
    m_targetAgeMsPub.Set(hubTarget->age.value() * 1000.0);
  } else {
    m_tagIdPub.Set(-1);
    m_tagYawDegPub.Set(kNoData);
    m_tagDistanceMetersPub.Set(kNoData);
    m_tagAmbiguityPub.Set(kNoData);
    m_targetAgeMsPub.Set(kNoData);
  }

  switch (m_autoAlignState) {
    case AutoAlignState::Left:
      m_statePub.Set("Left");
      break;
    case AutoAlignState::Right:
      m_statePub.Set("Right");
      break;
    case AutoAlignState::None:
      m_statePub.Set("None");
      break;
  }
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
