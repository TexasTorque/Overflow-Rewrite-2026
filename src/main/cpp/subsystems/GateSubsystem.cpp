#include "subsystems/GateSubsystem.hpp"
#include "abstractions/state/GateState.hpp"
#include "constants/Constants.hpp"
#include "frc2/command/Commands.h"

GateSubsystem::GateSubsystem(std::unique_ptr<GateIO> io)
    : m_io(std::move(io)),
      m_state(GateStateEnum::Off, [this](const GateStateEnum& newState) { ApplyState(newState); }) {
  SetName("GateSubsystem");
}

void GateSubsystem::Periodic() {
  m_io->UpdateInputs(m_inputs);
  m_logger.UpdateTelemetry(m_inputs, GetState());
}

frc2::CommandPtr GateSubsystem::RunGateCommand() {
  return frc2::cmd::StartEnd([this] { SetState(GateStateEnum::On); }, [this] { SetState(GateStateEnum::Off); }, {this})
      .WithName("Run Gate");
}

frc2::CommandPtr GateSubsystem::RunOuttakeCommand() {
  return frc2::cmd::StartEnd([this] { SetState(GateStateEnum::Outtake); }, [this] { SetState(GateStateEnum::Off); },
                             {this})
      .WithName("Gate Outtake");
}

void GateSubsystem::SetState(const GateStateEnum& newState) {
  m_state.Set(newState);
  m_state.Apply();
}

void GateSubsystem::Clean() {
  SetState(GateStateEnum::Off);
}

void GateSubsystem::ApplyState(const GateStateEnum& newState) {
  switch (newState) {
    case GateStateEnum::Off:
      m_io->SetGateVoltage(0_V);
      break;
    case GateStateEnum::On:
      m_io->SetGateVoltage(GateConstants::kGateVoltage);
      break;
    case GateStateEnum::Outtake:
      m_io->SetGateVoltage(GateConstants::kOuttakeVoltage);
      break;
  }
}
