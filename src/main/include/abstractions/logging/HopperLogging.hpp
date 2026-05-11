#include <telemetrykit/core/Units.h>
#include "abstractions/io/hopper/HopperIO.hpp"
#include "abstractions/state/HopperState.hpp"

class HopperLogging {
 public:
  void UpdateTelemetry(const HopperIOInputs& inputs, HopperStateEnum state) {
    tkit::RecordOutput("HopperSubsystem/Voltage", inputs.hopperVoltage);
    tkit::RecordOutput("HopperSubsystem/RequestedVoltage", inputs.voltageRequest);

    tkit::RecordOutput("HopperSubsystem/State", stateToString(state));
  }
};