// Texas Torque 1477
// Overflow 2026

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>
#include <telemetrykit/TelemetryKit.h>
#include <memory>
#include "abstractions/state/IntakeState.hpp"
#include "frc/RobotBase.h"
#include "frc2/command/Commands.h"
#include "telemetrykit/core/Logger.h"
#include "telemetrykit/receiver/NetworkTablesReceiver.h"
#include "telemetrykit/receiver/WPILogWriter.h"

Robot::Robot() {
  auto& logger = tkit::Logger::GetInstance();

  logger.AddReceiver(std::make_unique<tkit::NetworkTablesReceiver>());

  if (frc::RobotBase::IsReal()) {
    logger.AddReceiver(std::make_unique<tkit::WPILogWriter>());
  } else {
    logger.AddReceiver(std::make_unique<tkit::WPILogWriter>("logs"));
  }

  logger.Start();

  m_autonomousCommand = frc2::cmd::Run(
      [this] {
        if (m_trajectory) {
          if (auto sample = m_trajectory.value().SampleAt(m_timer.Get(), IsRedAlliance())) {
            m_container.GetDriveSubsystem().FollowTrajectory(sample.value());
          }

          for (const auto& event : m_trajectory.value().GetEvents("IntakeDown")) {
            if (std::abs(m_timer.Get().value() - event.timestamp.value()) < 0.02) {
              m_container.GetIntakeSubsystem().SetState(IntakeStateEnum::Intake);
            }
          }

          for (const auto& event : m_trajectory.value().GetEvents("IntakeStop")) {
            if (std::abs(m_timer.Get().value() - event.timestamp.value()) < 0.02) {
              m_container.GetIntakeSubsystem().SetState(IntakeStateEnum::Stow);
            }
          }
        }
      },
      {&m_container.GetDriveSubsystem()});
}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();

  m_timeAndJoystickReplay.Update();
  tkit::Logger::GetInstance().Periodic();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
  if (m_trajectory) {
    if (auto initialPose = m_trajectory.value().GetInitialPose(IsRedAlliance())) {
      m_container.GetDriveSubsystem().ResetPose(initialPose.value());
    }
  }

  m_timer.Reset();
  m_timer.Start();

  if (m_autonomousCommand) {
    frc2::CommandScheduler::GetInstance().Schedule(m_autonomousCommand.value());
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
  if (m_autonomousCommand && m_autonomousCommand->IsScheduled()) {
    m_autonomousCommand->Cancel();
  }

  auto& intakeSubsystem = m_container.GetIntakeSubsystem();
  auto& gateSubsystem = m_container.GetGateSubsystem();
  auto& hopperSubsystem = m_container.GetHopperSubsystem();
  auto& shooterSubsystem = m_container.GetShooterSubsystem();

  intakeSubsystem.SetState(intakeSubsystem.GetState());
  gateSubsystem.SetState(gateSubsystem.GetState());
  hopperSubsystem.SetState(hopperSubsystem.GetState());
  shooterSubsystem.SetState(shooterSubsystem.GetState());
}

void Robot::TeleopPeriodic() {}

void Robot::TeleopExit() {
  auto& intakeSubsystem = m_container.GetIntakeSubsystem();
  auto& gateSubsystem = m_container.GetGateSubsystem();
  auto& hopperSubsystem = m_container.GetHopperSubsystem();
  auto& shooterSubsystem = m_container.GetShooterSubsystem();

  intakeSubsystem.Clean();
  gateSubsystem.Clean();
  hopperSubsystem.Clean();
  shooterSubsystem.Clean();
}

void Robot::TestInit() {
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::TestExit() {}

bool Robot::IsRedAlliance() {
  return frc::DriverStation::GetAlliance().value_or(frc::DriverStation::kBlue) == frc::DriverStation::kRed;
}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
