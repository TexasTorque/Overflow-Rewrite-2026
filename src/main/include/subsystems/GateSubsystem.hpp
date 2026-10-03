// Texas Torque 1477
// Overflow 2026

#pragma once

#include "abstractions/io/gate/GateIO.hpp"
#include "abstractions/logging/GateLogging.hpp"
#include "abstractions/state/GateState.hpp"
#include "frc2/command/CommandPtr.h"
#include "turbolib/subsystem/TurboSubsystem.hpp"

class GateSubsystem : public turbolib::TurboSubsystem<GateIO, GateStateEnum> {
 public:
  explicit GateSubsystem(std::unique_ptr<GateIO> io);

  frc2::CommandPtr RunGateCommand();
  frc2::CommandPtr RunOuttakeCommand();

 protected:
  void ApplyState(const GateStateEnum& newState) override;
  GateStateEnum GetCleanState() const override { return GateStateEnum::Off; }
  void UpdateTelemetry() override;

 private:
  GateLogging m_logging;
};
