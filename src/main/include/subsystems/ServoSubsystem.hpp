// Texas Torque 1477
// Overflow 2026

#pragma once

#include "abstractions/io/servo/ServoIO.hpp"
#include "abstractions/state/ServoState.hpp"
#include "frc2/command/CommandPtr.h"
#include <frc2/command/SubsystemBase.h>
#include <memory>

class ServoSubsystem : public frc2::SubsystemBase {
 public:
  explicit ServoSubsystem(std::unique_ptr<ServoIO> io);

  void SetServoPos(double pos) { m_io->SetServoPos(pos); }

  frc2::CommandPtr SetServoUpCommand();
  frc2::CommandPtr SetServoLaserCommand();

  void SetState(const ServoStateEnum& newState);
  void Clean();

  void Periodic() override;

  ServoStateEnum GetState() const { return m_state.Get(); }

 private:
  std::unique_ptr<ServoIO> m_io;
  ServoIOInputs m_inputs;
  ServoState m_state;

  void ApplyState(const ServoStateEnum& newState);
};
