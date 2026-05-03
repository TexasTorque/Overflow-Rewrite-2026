#include "subsystems/IntakeSubsystem.hpp"
#include "abstractions/state/IntakeState.hpp"
#include "constants/constants.hpp"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/Commands.h"

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

void IntakeSubsystem::Periodic() {
  m_io->UpdateInputs(m_inputs);
  m_logger.UpdateTelemetry(m_inputs, GetState());
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
  }
}
