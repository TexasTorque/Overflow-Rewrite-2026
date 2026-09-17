// Texas Torque 1477
// Overflow 2026

#pragma once

#include "abstractions/state/SubsystemState.hpp"
#include "abstractions/io/shooter/ShooterIO.hpp"
#include "abstractions/state/ShooterState.hpp"
#include "networktables/BooleanTopic.h"
#include "networktables/DoubleTopic.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/StringTopic.h"

class ShooterLogging {
 public:
  ShooterLogging()
      : flywheelNearDesiredPublisher{nt::NetworkTableInstance::GetDefault()
                                         .GetBooleanTopic("ShooterSubsystem/FlywheelNearDesired")
                                         .Publish()},
        flywheelRPMPublisher{
            nt::NetworkTableInstance::GetDefault().GetDoubleTopic("ShooterSubsystem/FlywheelRPM").Publish()},
        flywheelRPMSetpointPublisher{
            nt::NetworkTableInstance::GetDefault().GetDoubleTopic("ShooterSubsystem/FlywheelRPMSetpoint").Publish()},
        flywheelCurrentPublisher{
            nt::NetworkTableInstance::GetDefault().GetDoubleTopic("ShooterSubsystem/FlywheelCurrent").Publish()},
        statePublisher{nt::NetworkTableInstance::GetDefault().GetStringTopic("ShooterSubsystem/State").Publish()} {}

  void UpdateTelemetry(ShooterIOInputs inputs, ShooterStateEnum state) {
    flywheelNearDesiredPublisher.Set(inputs.flywheelNearDesired);
    flywheelRPMPublisher.Set(inputs.flywheelRPM.value());
    flywheelRPMSetpointPublisher.Set(inputs.flywheelRPMSetpoint.value());
    flywheelCurrentPublisher.Set(inputs.flywheelCurrent.value());
    statePublisher.Set(stateToString(state));
  }

 private:
  nt::BooleanPublisher flywheelNearDesiredPublisher;
  nt::DoublePublisher flywheelRPMPublisher;
  nt::DoublePublisher flywheelRPMSetpointPublisher;
  nt::DoublePublisher flywheelCurrentPublisher;
  nt::StringPublisher statePublisher;
};
