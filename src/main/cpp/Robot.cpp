// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>
#include <telemetrykit/TelemetryKit.h>
#include <memory>
#include "frc/RobotBase.h"
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
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand) {
    frc2::CommandScheduler::GetInstance().Schedule(m_autonomousCommand.value());
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
  if (m_autonomousCommand) {
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
