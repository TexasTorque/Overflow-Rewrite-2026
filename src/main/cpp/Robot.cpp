// Texas Torque 1477
// Overflow 2026

#include "Robot.h"

#include <frc2/command/CommandScheduler.h>
#include "frc/DataLogManager.h"
#include "frc/DriverStation.h"
#include "frc/RobotBase.h"
#include "frc/smartdashboard/SmartDashboard.h"

Robot::Robot() {
  frc::DataLogManager::Start();
  frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());

  frc::SmartDashboard::PutData("Command Scheduler", &frc2::CommandScheduler::GetInstance());
}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();

  frc::SmartDashboard::PutNumber("MatchTime", frc::DriverStation::GetMatchTime().value());
  frc::SmartDashboard::PutNumber("Voltage", frc::DriverStation::GetBatteryVoltage());

  m_timeAndJoystickReplay.Update();
}

void Robot::DisabledInit() {}

void Robot::DisabledPeriodic() {}

void Robot::DisabledExit() {}

void Robot::AutonomousInit() {
  m_autonomousCommand = m_container.GetAutonomousCommand();

  if (m_autonomousCommand) {
    frc2::CommandScheduler::GetInstance().Schedule(m_autonomousCommand);
  }
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() {}

void Robot::TeleopInit() {
  if (m_autonomousCommand) {
    frc2::CommandScheduler::GetInstance().Cancel(m_autonomousCommand);
  }

  auto& intakeSubsystem = m_container.GetIntakeSubsystem();
  auto& gateSubsystem = m_container.GetGateSubsystem();
  auto& hopperSubsystem = m_container.GetHopperSubsystem();
  auto& shooterSubsystem = m_container.GetShooterSubsystem();
  auto& servoSubsystem = m_container.GetServoSubsystem();

  m_container.GetPerceptionSubsystem().EnableVision();
  intakeSubsystem.SetState(intakeSubsystem.GetState());
  gateSubsystem.SetState(gateSubsystem.GetState());
  hopperSubsystem.SetState(hopperSubsystem.GetState());
  shooterSubsystem.SetState(shooterSubsystem.GetState());
  servoSubsystem.SetState(servoSubsystem.GetState());
}

void Robot::TeleopPeriodic() {}

void Robot::TeleopExit() {
  auto& intakeSubsystem = m_container.GetIntakeSubsystem();
  auto& gateSubsystem = m_container.GetGateSubsystem();
  auto& hopperSubsystem = m_container.GetHopperSubsystem();
  auto& shooterSubsystem = m_container.GetShooterSubsystem();
  auto& servoSubsystem = m_container.GetServoSubsystem();

  intakeSubsystem.Clean();
  gateSubsystem.Clean();
  hopperSubsystem.Clean();
  shooterSubsystem.Clean();
  servoSubsystem.Clean();
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
