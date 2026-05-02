#include "subsystems/IntakeSubsystem.hpp"
#include "abstractions/state/IntakeState.hpp"
#include "constants/constants.hpp"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/Commands.h"

IntakeSubsystem::IntakeSubsystem(std::unique_ptr<IntakeIO> io)
    : io(std::move(io)),
      state(IntakeStateEnum::Stow, [this](const IntakeStateEnum& newState) { ApplyState(newState); }) {
  SetName("IntakeSubsystem");
}

frc2::CommandPtr IntakeSubsystem::RunIntakeCommand() {
  return frc2::cmd::StartEnd([this] { SetState(IntakeStateEnum::Intake); }, [this] { SetState(IntakeStateEnum::Stow); },
                             {this})
      .WithName("Run Intake");
}

void IntakeSubsystem::Periodic() {
  io->UpdateInputs(inputs);
  logger.UpdateTelemetry(inputs, GetState());
}

void IntakeSubsystem::ApplyState(const IntakeStateEnum& newState) {
  switch (newState) {
    case IntakeStateEnum::Stow:
      io->SetIntakeVoltage(0_V);
      io->SetIntakePivotSetpoint(IntakeConstants::kRotaryDownPosition);
      break;
    case IntakeStateEnum::Intake:
      io->SetIntakeVoltage(IntakeConstants::kIntakeVoltage);
      io->SetIntakePivotSetpoint(IntakeConstants::kRotaryDownPosition);
      break;
  }
}
