#pragma once

#include <memory>
#include "abstractions/io/gate/GateIO.hpp"
#include "abstractions/logging/GateLogging.hpp"
#include "abstractions/state/GateState.hpp"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/SubsystemBase.h"

class GateSubsystem : public frc2::SubsystemBase {
 public:
  GateSubsystem(std::unique_ptr<GateIO> io);

  void Periodic() override;

  frc2::CommandPtr RunGateCommand();
  frc2::CommandPtr RunOuttakeCommand();

  void SetState(const GateStateEnum& newState);
  void Clean();

  GateStateEnum GetState() const { return m_state.Get(); }

 private:
  std::unique_ptr<GateIO> m_io;
  GateLogging m_logger;
  GateIOInputs m_inputs;
  GateState m_state;

  void ApplyState(const GateStateEnum& newState);
};