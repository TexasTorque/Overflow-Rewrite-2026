// Texas Torque 1477
// Overflow 2026
#pragma once

#include <telemetrykit/core/Units.h>
#include "abstractions/io/intake/IntakeIO.hpp"
#include "abstractions/state/IntakeState.hpp"

class IntakeLogging {
 public:
  static void UpdateTelemetry(const IntakeIOInputs& inputs, IntakeStateEnum state) {
    tkit::RecordOutput("IntakeSubsystem/RollerCurrent", inputs.rollerCurrent);
    tkit::RecordOutput("IntakeSubsystem/RollerVoltage", inputs.rollerVoltage);
    tkit::RecordOutput("IntakeSubsystem/PivotPosition", inputs.pivotPosition);
    tkit::RecordOutput("IntakeSubsystem/PivotSetpoint", inputs.pivotSetpoint);

    tkit::RecordOutput("IntakeSubsystem/State", stateToString(state));
  }
};
