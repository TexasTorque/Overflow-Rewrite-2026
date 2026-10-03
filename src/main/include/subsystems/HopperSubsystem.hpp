// Texas Torque 1477
// Overflow 2026

#pragma once

#include "abstractions/io/hopper/HopperIO.hpp"
#include "abstractions/logging/HopperLogging.hpp"
#include "abstractions/state/HopperState.hpp"
#include "frc2/command/CommandPtr.h"
#include "turbolib/subsystem/TurboSubsystem.hpp"

class HopperSubsystem : public turbolib::TurboSubsystem<HopperIO, HopperStateEnum> {
 public:
  explicit HopperSubsystem(std::unique_ptr<HopperIO> io);

  frc2::CommandPtr RunHopperCommand();
  frc2::CommandPtr RunOuttakeCommand();

 protected:
  void ApplyState(const HopperStateEnum& newState) override;
  HopperStateEnum GetCleanState() const override { return HopperStateEnum::Off; }
  void UpdateTelemetry() override;

 private:
  HopperLogging m_logging;
};
