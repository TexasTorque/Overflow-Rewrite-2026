#include "abstractions/io/intake/IntakeIO.hpp"
#include "abstractions/state/IntakeState.hpp"
#include "networktables/DoubleTopic.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/StringTopic.h"
#include <string>

class IntakeLogging {
public:
  IntakeLogging() {
    intakeVoltagePublisher =
        nt::NetworkTableInstance::GetDefault()
            .GetDoubleTopic("IntakeSubsystem/RollerVoltage")
            .Publish();
    intakeCurrentPublisher =
        nt::NetworkTableInstance::GetDefault()
            .GetDoubleTopic("IntakeSubsystem/RollerCurrent")
            .Publish();
    pivotPositionPublisher =
        nt::NetworkTableInstance::GetDefault()
            .GetDoubleTopic("IntakeSubsystem/PivotPosition")
            .Publish();
    pivotSetpointPublisher =
        nt::NetworkTableInstance::GetDefault()
            .GetDoubleTopic("IntakeSubsystem/PivotSetpoint")
            .Publish();
    statePublisher = nt::NetworkTableInstance::GetDefault()
                         .GetStringTopic("IntakeSubsystem/State")
                         .Publish();
  }

  void UpdateTelemetry(IntakeIOInputs inputs, IntakeStateEnum state) {
    intakeVoltagePublisher.Set(inputs.rollerVoltage.value());
    intakeCurrentPublisher.Set(inputs.rollerCurrent.value());
    pivotPositionPublisher.Set(inputs.pivotPosition);
    pivotSetpointPublisher.Set(inputs.pivotSetpoint);
    statePublisher.Set(std::to_string(state));
  }

private:
  nt::DoublePublisher pivotSetpointPublisher;
  nt::DoublePublisher pivotPositionPublisher;
  nt::DoublePublisher intakeVoltagePublisher;
  nt::DoublePublisher intakeCurrentPublisher;

  nt::StringPublisher statePublisher;
};
