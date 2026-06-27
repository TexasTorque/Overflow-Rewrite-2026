// Texas Torque 1477
// Overflow 2026

#pragma once

#include "abstractions/io/intake/IntakeIO.hpp"
#include "abstractions/state/IntakeState.hpp"
#include "networktables/DoubleTopic.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/StringTopic.h"

class IntakeLogging {
 public:
  static void UpdateTelemetry(const IntakeIOInputs& inputs, IntakeStateEnum state) {
    rollerCurrentPublisher.Set(inputs.rollerCurrent.value());
    rollerVoltagePublisher.Set(inputs.rollerVoltage.value());
    pivotPositionPublisher.Set(inputs.pivotPosition);
    pivotSetpointPublisher.Set(inputs.pivotSetpoint);

    statePublisher.Set(stateToString(state));
  }

 private:
   static nt::DoublePublisher rollerCurrentPublisher;
   static nt::DoublePublisher rollerVoltagePublisher;
   static nt::DoublePublisher pivotPositionPublisher;
   static nt::DoublePublisher pivotSetpointPublisher;
   static nt::StringPublisher statePublisher;
};

inline nt::DoublePublisher IntakeLogging::rollerCurrentPublisher = nt::NetworkTableInstance::GetDefault().GetDoubleTopic("IntakeSubsystem/RollerCurrent").Publish();
inline nt::DoublePublisher IntakeLogging::rollerVoltagePublisher = nt::NetworkTableInstance::GetDefault().GetDoubleTopic("IntakeSubsystem/RollerVoltage").Publish();
inline nt::DoublePublisher IntakeLogging::pivotPositionPublisher = nt::NetworkTableInstance::GetDefault().GetDoubleTopic("IntakeSubsystem/PivotPosition").Publish();
inline nt::DoublePublisher IntakeLogging::pivotSetpointPublisher = nt::NetworkTableInstance::GetDefault().GetDoubleTopic("IntakeSubsystem/PivotSetpoint").Publish();
inline nt::StringPublisher IntakeLogging::statePublisher = nt::NetworkTableInstance::GetDefault().GetStringTopic("IntakeSubsystem/State").Publish();
