// Texas Torque 1477
// Overflow 2026

#pragma once

#include "telemetrykit/core/Logger.h"
#include "abstractions/io/servo/ServoIO.hpp"
#include "abstractions/state/ServoState.hpp"

class ServoLogging {
 public:
  static void UpdateTelemetry(const ServoIOInputs& inputs, ServoStateEnum state) {
    tkit::RecordOutput("ServoSubsystem/Position", inputs.servoSetpoint);

    tkit::RecordOutput("ServoSubsystem/State", stateToString(state));
  }
};
