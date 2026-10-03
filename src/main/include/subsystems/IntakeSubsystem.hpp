// Texas Torque 1477
// Overflow 2026

#pragma once

#include "abstractions/io/intake/IntakeIO.hpp"
#include "abstractions/logging/IntakeLogging.hpp"
#include "abstractions/state/IntakeState.hpp"
#include "frc2/command/CommandPtr.h"
#include "turbolib/subsystem/TurboSubsystem.hpp"
#include "units/current.h"

class IntakeSubsystem : public turbolib::TurboSubsystem<IntakeIO, IntakeStateEnum> {
 public:
  explicit IntakeSubsystem(std::unique_ptr<IntakeIO> io);

  frc2::CommandPtr RunIntakeCommand();
  frc2::CommandPtr RunOuttakeCommand();
  frc2::CommandPtr StopIntakeCommand();
  frc2::CommandPtr StowIntakeCommand();
  frc2::CommandPtr SlowZeroCommand();

  units::ampere_t GetRollerCurrent() const { return m_inputs.rollerCurrent; }

 protected:
  void ApplyState(const IntakeStateEnum& newState) override;
  IntakeStateEnum GetCleanState() const override { return IntakeStateEnum::Stow; }
  void UpdateTelemetry() override;
  void PeriodicExtras() override;

 private:
  IntakeLogging m_logging;
};
