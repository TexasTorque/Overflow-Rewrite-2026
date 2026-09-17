// Texas Torque 1477
// Overflow 2026

#pragma once

#include "abstractions/state/SubsystemState.hpp"
#include "abstractions/io/intake/IntakeIO.hpp"
#include "abstractions/state/IntakeState.hpp"
#include "networktables/DoubleTopic.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/StringTopic.h"

class IntakeLogging {
 public:
  IntakeLogging()
      : rollerCurrentPublisher{nt::NetworkTableInstance::GetDefault()
                                   .GetDoubleTopic("IntakeSubsystem/RollerCurrent")
                                   .Publish()},
        rollerVoltagePublisher{
            nt::NetworkTableInstance::GetDefault().GetDoubleTopic("IntakeSubsystem/RollerVoltage").Publish()},
        pivotPositionPublisher{
            nt::NetworkTableInstance::GetDefault().GetDoubleTopic("IntakeSubsystem/PivotPosition").Publish()},
        pivotSetpointPublisher{
            nt::NetworkTableInstance::GetDefault().GetDoubleTopic("IntakeSubsystem/PivotSetpoint").Publish()},
        statePublisher{nt::NetworkTableInstance::GetDefault().GetStringTopic("IntakeSubsystem/State").Publish()} {}

  void UpdateTelemetry(const IntakeIOInputs& inputs, IntakeStateEnum state) {
    rollerCurrentPublisher.Set(inputs.rollerCurrent.value());
    rollerVoltagePublisher.Set(inputs.rollerVoltage.value());
    pivotPositionPublisher.Set(inputs.pivotPosition);
    pivotSetpointPublisher.Set(inputs.pivotSetpoint);

    statePublisher.Set(stateToString(state));
  }

 private:
  nt::DoublePublisher rollerCurrentPublisher;
  nt::DoublePublisher rollerVoltagePublisher;
  nt::DoublePublisher pivotPositionPublisher;
  nt::DoublePublisher pivotSetpointPublisher;
  nt::StringPublisher statePublisher;
};
