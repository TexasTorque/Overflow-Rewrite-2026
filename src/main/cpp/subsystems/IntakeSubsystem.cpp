// Texas Torque 1477
// Overflow 2026

#include "subsystems/IntakeSubsystem.hpp"
#include "constants/Constants.hpp"
#include "frc2/command/Commands.h"

IntakeSubsystem::IntakeSubsystem(std::unique_ptr<IntakeIO> io)
    : TurboSubsystem(std::move(io), IntakeStateEnum::Stow, "IntakeSubsystem") {}

frc2::CommandPtr IntakeSubsystem::RunIntakeCommand() {
  return frc2::cmd::StartEnd([this] { SetState(IntakeStateEnum::Intake); }, [this] { SetState(IntakeStateEnum::Stow); },
                             {this})
      .WithName("Run Intake");
}

frc2::CommandPtr IntakeSubsystem::RunOuttakeCommand() {
  return frc2::cmd::StartEnd([this] { SetState(IntakeStateEnum::Outtake); },
                             [this] { SetState(IntakeStateEnum::Stow); }, {this})
      .WithName("Run Outtake");
}

frc2::CommandPtr IntakeSubsystem::StopIntakeCommand() {
  return frc2::cmd::RunOnce([this] { SetState(IntakeStateEnum::Stow); }, {this}).WithName("Stop Intake");
}

frc2::CommandPtr IntakeSubsystem::SlowZeroCommand() {
  return frc2::cmd::StartEnd([this] { SetState(IntakeStateEnum::SlowPullup); },
                             [this] { SetState(IntakeStateEnum::Stow); }, {this});
}

frc2::CommandPtr IntakeSubsystem::StowIntakeCommand() {
  return frc2::cmd::StartEnd([this] { SetState(IntakeStateEnum::PullIn); }, [this] { SetState(IntakeStateEnum::Stow); },
                             {this})
      .WithName("Pull In Intake");
}

void IntakeSubsystem::UpdateTelemetry() {
  m_logging.UpdateTelemetry(m_inputs, GetState());
}

void IntakeSubsystem::PeriodicExtras() {
  m_io->Process();
}

void IntakeSubsystem::ApplyState(const IntakeStateEnum& newState) {
  switch (newState) {
    case IntakeStateEnum::Stow:
      m_io->SetIntakeVoltage(0_V);
      m_io->SetIntakePivotSetpoint(IntakeConstants::kRotaryDownPosition);
      break;
    case IntakeStateEnum::Intake:
      m_io->SetIntakeVoltage(IntakeConstants::kIntakeVoltage);
      m_io->SetIntakePivotSetpoint(IntakeConstants::kRotaryDownPosition);
      break;
    case IntakeStateEnum::Outtake:
      m_io->SetIntakeVoltage(IntakeConstants::kOuttakeVoltage);
      m_io->SetIntakePivotSetpoint(IntakeConstants::kRotaryDownPosition);
      break;
    case IntakeStateEnum::SlowPullup:
      m_io->SetIntakeVoltage(0_V);
      m_io->SetIntakePivotSetpoint(IntakeConstants::kRotaryUpPosition, true);
      break;
    case IntakeStateEnum::PullIn:
      m_io->SetIntakeVoltage(0_V);
      m_io->SetIntakePivotSetpoint(IntakeConstants::kRotaryUpPosition, true);
      break;
    case IntakeStateEnum::Agitation:
      m_io->SetIntakeVoltage(IntakeConstants::kAgitationVoltage);
      m_io->SetIntakePivotSetpoint(IntakeConstants::kRotaryUpPosition, true);
      break;
  }
}
