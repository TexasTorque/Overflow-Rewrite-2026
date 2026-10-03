// Texas Torque 1477
// Overflow 2026
#include "subsystems/GateSubsystem.hpp"
#include "constants/Constants.hpp"
#include "frc2/command/Commands.h"


GateSubsystem::GateSubsystem(std::unique_ptr<GateIO> io)
    : TurboSubsystem(std::move(io), GateStateEnum::Off, "GateSubsystem") {}

void GateSubsystem::UpdateTelemetry() {
  m_logging.UpdateTelemetry(m_inputs, GetState());
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
