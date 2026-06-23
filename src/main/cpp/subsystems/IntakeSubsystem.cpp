// Texas Torque 1477
// Overflow 2026

#include "subsystems/IntakeSubsystem.hpp"
#include "abstractions/logging/IntakeLogging.hpp"
#include "abstractions/state/IntakeState.hpp"
#include "constants/constants.hpp"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/Commands.h"
#include <utility>

IntakeSubsystem::IntakeSubsystem(std::unique_ptr<IntakeIO> io)
    : m_io(std::move(io)),
      m_state(IntakeStateEnum::Stow, [this](const IntakeStateEnum& newState) { ApplyState(newState); }) {
  SetName("IntakeSubsystem");
}

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

void IntakeSubsystem::SetState(const IntakeStateEnum& newState) {
  m_state.Set(newState);
  m_state.Apply();
}

void IntakeSubsystem::Clean() {
  SetState(IntakeStateEnum::Stow);
}

void IntakeSubsystem::Periodic() {
  m_io->UpdateInputs(m_inputs);
  m_io->Process();
  IntakeLogging::UpdateTelemetry(m_inputs, GetState());
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
      m_io->SetIntakePivotSetpoint(IntakeConstants::kRotarySlowZeroPosition, true);
      break;
  }
}
