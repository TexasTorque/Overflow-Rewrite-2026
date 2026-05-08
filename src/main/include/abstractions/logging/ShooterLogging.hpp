#include <telemetrykit/core/Units.h>
#include "abstractions/io/shooter/ShooterIO.hpp"
#include "abstractions/state/ShooterState.hpp"

class ShooterLogging {
 public:
  void UpdateTelemetry(ShooterIOInputs inputs, ShooterStateEnum state) {
    tkit::RecordOutput("ShooterSubsystem/FlywheelNearDesired", inputs.flywheelNearDesired);
    tkit::RecordOutput("ShooterSubsystem/FlywheelRPM", inputs.flywheelRPM);
    tkit::RecordOutput("ShooterSubsystem/FlywheelRPMSetpoint", inputs.flywheelRPMSetpoint);

    tkit::RecordOutput("ShooterSubsystem/State", stateToString(state));
  }
};
