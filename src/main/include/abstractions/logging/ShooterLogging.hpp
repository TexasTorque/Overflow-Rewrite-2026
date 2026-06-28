// Texas Torque 1477
// Overflow 2026

#pragma once

#include "abstractions/io/shooter/ShooterIO.hpp"
#include "abstractions/state/ShooterState.hpp"
#include "networktables/BooleanTopic.h"
#include "networktables/DoubleTopic.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/StringTopic.h"

class ShooterLogging {
 public:
  static void UpdateTelemetry(ShooterIOInputs inputs, ShooterStateEnum state) {
    flywheelNearDesiredPublisher.Set(inputs.flywheelNearDesired);
    flywheelRPMPublisher.Set(inputs.flywheelRPM.value());
    flywheelRPMSetpointPublisher.Set(inputs.flywheelRPMSetpoint.value());
    flywheelCurrentPublisher.Set(inputs.flywheelCurrent.value());
    statePublisher.Set(stateToString(state));
  }

 private:
  static nt::BooleanPublisher flywheelNearDesiredPublisher;
  static nt::DoublePublisher flywheelRPMPublisher;
  static nt::DoublePublisher flywheelRPMSetpointPublisher;
  static nt::DoublePublisher flywheelCurrentPublisher;
  static nt::StringPublisher statePublisher;
};

inline nt::BooleanPublisher ShooterLogging::flywheelNearDesiredPublisher =
    nt::NetworkTableInstance::GetDefault().GetBooleanTopic("ShooterSubsystem/FlywheelNearDesired").Publish();
inline nt::DoublePublisher ShooterLogging::flywheelRPMPublisher =
    nt::NetworkTableInstance::GetDefault().GetDoubleTopic("ShooterSubsystem/FlywheelRPM").Publish();
inline nt::DoublePublisher ShooterLogging::flywheelRPMSetpointPublisher =
    nt::NetworkTableInstance::GetDefault().GetDoubleTopic("ShooterSubsystem/FlywheelRPMSetpoint").Publish();
inline nt::DoublePublisher ShooterLogging::flywheelCurrentPublisher =
    nt::NetworkTableInstance::GetDefault().GetDoubleTopic("ShooterSubsystem/FlywheelCurrent").Publish();
inline nt::StringPublisher ShooterLogging::statePublisher =
    nt::NetworkTableInstance::GetDefault().GetStringTopic("ShooterSubsystem/State").Publish();
