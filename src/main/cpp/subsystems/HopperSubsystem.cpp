// Texas Torque 1477
// Overflow 2026

#include "subsystems/HopperSubsystem.hpp"
#include "constants/Constants.hpp"
#include "frc2/command/Commands.h"

HopperSubsystem::HopperSubsystem(std::unique_ptr<HopperIO> io)
    : TurboSubsystem(std::move(io), HopperStateEnum::Off, "HopperSubsystem") {}

void HopperSubsystem::UpdateTelemetry() {
  m_logging.UpdateTelemetry(m_inputs, GetState());
}

frc2::CommandPtr HopperSubsystem::RunHopperCommand() {
  return frc2::cmd::StartEnd([this] { SetState(HopperStateEnum::Intake); }, [this] { SetState(HopperStateEnum::Off); },
                             {this})
      .WithName("Run Hopper");
}

frc2::CommandPtr HopperSubsystem::RunOuttakeCommand() {
  return frc2::cmd::StartEnd([this] { SetState(HopperStateEnum::Outtake); }, [this] { SetState(HopperStateEnum::Off); },
                             {this})
      .WithName("Hopper Outtake");
}

void HopperSubsystem::ApplyState(const HopperStateEnum& newState) {
  switch (newState) {
    case HopperStateEnum::Intake:
      m_io->SetHopperVoltage(HopperConstants::kIntakeVoltage);
      break;
    case HopperStateEnum::Off:
      m_io->SetHopperVoltage(0_V);
      break;
    case HopperStateEnum::Outtake:
      m_io->SetHopperVoltage(HopperConstants::kOuttakeVoltage);
      break;
  }
}
