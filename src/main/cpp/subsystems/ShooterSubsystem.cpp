#include "subsystems/ShooterSubsystem.hpp"
#include <functional>
#include "abstractions/io/shooter/ShooterIO.hpp"
#include "abstractions/state/ShooterState.hpp"
#include "constants/Constants.hpp"
#include "frc/DataLogManager.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/Commands.h"
#include "units/angular_velocity.h"

ShooterSubsystem::ShooterSubsystem(std::unique_ptr<ShooterIO> io)
    : m_io(std::move(io)),
      m_state(ShooterStateEnum::Off, [this](const ShooterStateEnum& newState) { ApplyState(newState); }) {
  SetName("ShooterSubsystem");
}

void ShooterSubsystem::Periodic() {
  m_io->UpdateInputs(m_inputs);
  m_logger.UpdateTelemetry(m_inputs, GetState());
}

frc2::CommandPtr ShooterSubsystem::RunLayupCommand() {
  return frc2::cmd::StartEnd([this] { SetState(ShooterStateEnum::Layup); },
                             [this] { SetState(ShooterStateEnum::Idle); }, {this})
      .WithName("Shooter Layup");
}

frc2::CommandPtr ShooterSubsystem::RunLaserCommand() {
  return frc2::cmd::StartEnd([this] { SetState(ShooterStateEnum::Laser); },
                             [this] { SetState(ShooterStateEnum::Idle); }, {this})
      .WithName("Shooter Laser");
}

frc2::CommandPtr ShooterSubsystem::RunClimbCommand() {
  return frc2::cmd::StartEnd([this] { SetState(ShooterStateEnum::Climb); },
                             [this] { SetState(ShooterStateEnum::Idle); }, {this})
      .WithName("Shooter Climb");
}

frc2::CommandPtr ShooterSubsystem::RunTrenchCommand() {
  return frc2::cmd::StartEnd([this] { SetState(ShooterStateEnum::Trench); },
                             [this] { SetState(ShooterStateEnum::Idle); }, {this})
      .WithName("Shooter Trench");
}

frc2::CommandPtr ShooterSubsystem::RunRegressionCommand() {
  return frc2::cmd::StartEnd([this] { SetState(ShooterStateEnum::Regression); },
                             [this] { SetState(ShooterStateEnum::Idle); }, {this})
      .WithName("Shooter Regression");
}

void ShooterSubsystem::SetState(const ShooterStateEnum& newState) {
  m_state.Set(newState);
  m_state.Apply();
}

void ShooterSubsystem::Clean() {
  SetState(ShooterStateEnum::Off);
}

void ShooterSubsystem::ApplyState(const ShooterStateEnum& newState) {
  switch (newState) {
    case ShooterStateEnum::Off:
      m_io->SetFlywheelRPM(0_rpm);
      break;
    case ShooterStateEnum::Idle:
      m_io->SetFlywheelRPM(ShooterConstants::kIdleRPM);
      break;
    case ShooterStateEnum::Layup:
      m_io->SetFlywheelRPM(ShooterConstants::kLayupRPM);
      break;
    case ShooterStateEnum::Laser:
      m_io->SetFlywheelRPM(ShooterConstants::kLaserRPM);
      break;
    case ShooterStateEnum::Trench:
      m_io->SetFlywheelRPM(ShooterConstants::kTrenchRPM);
      break;
    case ShooterStateEnum::Climb:
      m_io->SetFlywheelRPM(ShooterConstants::kClimbRPM);
      break;
    case ShooterStateEnum::Regression:
      frc::DataLogManager::Log("Regression not implemented");
      break;
  }
}
