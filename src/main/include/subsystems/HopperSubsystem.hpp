// Texas Torque 1477
// Overflow 2026

#pragma once

#include <memory>
#include "abstractions/io/hopper/HopperIO.hpp"
#include "abstractions/logging/HopperLogging.hpp"
#include "abstractions/state/HopperState.hpp"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/SubsystemBase.h"

class HopperSubsystem : public frc2::SubsystemBase {
 public:
  HopperSubsystem(std::unique_ptr<HopperIO> io);

  void Periodic() override;

  frc2::CommandPtr RunHopperCommand();
  frc2::CommandPtr RunOuttakeCommand();

  void SetState(const HopperStateEnum& newState);
  void Clean();

  HopperStateEnum GetState() const { return m_state.Get(); }

 private:
  std::unique_ptr<HopperIO> m_io;
  HopperLogging m_logger;
  HopperIOInputs m_inputs;
  HopperState m_state;

  void ApplyState(const HopperStateEnum& newState);
};
