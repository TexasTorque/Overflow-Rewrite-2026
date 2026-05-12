// Texas Torque 1477
// Overflow 2026

#pragma once

#include "abstractions/io/intake/IntakeIO.hpp"
#include "abstractions/logging/IntakeLogging.hpp"
#include "abstractions/state/IntakeState.hpp"
#include "frc2/command/CommandPtr.h"
#include <frc2/command/SubsystemBase.h>
#include <memory>

class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  IntakeSubsystem(std::unique_ptr<IntakeIO> io);

  void Periodic() override;

  frc2::CommandPtr RunIntakeCommand();
  frc2::CommandPtr RunOuttakeCommand();
  frc2::CommandPtr SlowZeroCommand();

  void SetState(const IntakeStateEnum& newState);
  void Clean();

  IntakeStateEnum GetState() const { return m_state.Get(); }

 private:
  std::unique_ptr<IntakeIO> m_io;
  IntakeLogging m_logger;
  IntakeIOInputs m_inputs;
  IntakeState m_state;

  void ApplyState(const IntakeStateEnum& newState);
};
