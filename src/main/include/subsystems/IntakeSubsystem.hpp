#pragma once

#include "abstractions/io/intake/IntakeIO.hpp"
#include "abstractions/logging/IntakeLogging.hpp"
#include "abstractions/state/IntakeState.hpp"
#include "frc2/command/CommandPtr.h"
#include <frc2/command/SubsystemBase.h>
#include <memory>

class IntakeSubsystem : public frc2::SubsystemBase {
 public:
  IntakeSubsystem(std::unique_ptr<IntakeIO> intakeIO);

  void Periodic() override;

  frc2::CommandPtr RunIntakeCommand();

  void SetState(const IntakeStateEnum& newState) {
    state.Set(newState);
    state.Apply();
  }
  IntakeStateEnum GetState() const { return state.Get(); }

 private:
  std::unique_ptr<IntakeIO> io;
  IntakeLogging logger;
  IntakeIOInputs inputs;
  IntakeState state;

  void ApplyState(const IntakeStateEnum& newState);
};