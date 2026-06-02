// Texas Torque 1477
// Overflow 2026

#pragma once

#include "telemetrykit/core/Logger.h"
#include "telemetrykit/core/Units.h"
#include "abstractions/io/gate/GateIO.hpp"
#include "abstractions/state/GateState.hpp"

class GateLogging {
 public:
  static void UpdateTelemetry(const GateIOInputs& inputs, GateStateEnum state) {
    tkit::RecordOutput("GateSubsystem/Voltage", inputs.gateVoltage);
    tkit::RecordOutput("GateSubsystem/Current", inputs.gateCurrent);

    tkit::RecordOutput("GateSubsystem/State", stateToString(state));
  }
};
