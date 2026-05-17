// Texas Torque 1477
// Overflow 2026

#pragma once

#include <telemetrykit/core/Units.h>
#include "abstractions/io/hopper/HopperIO.hpp"
#include "abstractions/state/HopperState.hpp"

class HopperLogging {
 public:
  static void UpdateTelemetry(const HopperIOInputs& inputs, HopperStateEnum state) {
    tkit::RecordOutput("HopperSubsystem/Voltage", inputs.hopperVoltage);
    tkit::RecordOutput("HopperSubsystem/Current", inputs.hopperCurrent);

    tkit::RecordOutput("HopperSubsystem/State", stateToString(state));
  }
};
