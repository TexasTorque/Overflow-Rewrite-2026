// Texas Torque 1477
// Overflow 2026

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>
#include <telemetrykit/TelemetryKit.h>
#include <memory>
#include "abstractions/autonomous/AutonomousCommand.hpp"
#include "frc/RobotBase.h"
#include "frc/smartdashboard/SmartDashboard.h"
#include "telemetrykit/core/Logger.h"
#include "telemetrykit/receiver/NetworkTablesReceiver.h"
#include "telemetrykit/receiver/WPILogWriter.h"

Robot::Robot() : m_autoChooser([this] { m_trajectory = m_autoChooser.GetSelectedTrajectory(); }) {
  frc::SmartDashboard::PutData("Command Scheduler", &frc2::CommandScheduler::GetInstance());

  auto& logger = tkit::Logger::GetInstance();

  logger.AddReceiver(std::make_unique<tkit::NetworkTablesReceiver>());

  if (frc::RobotBase::IsReal()) {
    logger.AddReceiver(std::make_unique<tkit::WPILogWriter>());
  } else {
    logger.AddReceiver(std::make_unique<tkit::WPILogWriter>("logs"));
  }

  logger.Start();

  m_eventMap = {
      {"IntakeDown", [this] { return m_container.GetIntakeSubsystem().RunIntakeCommand(); }},
      {"IntakeStop", [this] { return m_container.GetIntakeSubsystem().StopIntakeCommand(); }},
      {"AutoAlign", [this] { return m_container.GetDriveSubsystem().RotateToHub(); }},
  };

  m_autonomousCommand =
      AutonomousCommand(
          m_container.GetDriveSubsystem(), [this] { return m_trajectory; }, m_timer, std::move(m_eventMap))
          .ToPtr();
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
  if (m_autonomousCommand.has_value()) {
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

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
