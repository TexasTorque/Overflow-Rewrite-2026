// Texas Torque 1477
// Overflow 2026

#pragma once

#include "abstractions/state/SubsystemState.hpp"
#include "networktables/DoubleTopic.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/StringTopic.h"
#include "abstractions/io/servo/ServoIO.hpp"
#include "abstractions/state/ServoState.hpp"

class ServoLogging {
 public:
  static void UpdateTelemetry(const ServoIOInputs& inputs, ServoStateEnum state) {
    positionPublisher.Set(inputs.servoSetpoint);
    statePublisher.Set(stateToString(state));
  }

 private:
  static nt::DoublePublisher positionPublisher;
  static nt::StringPublisher statePublisher;
};

inline nt::DoublePublisher ServoLogging::positionPublisher =
    nt::NetworkTableInstance::GetDefault().GetDoubleTopic("ServoSubsystem/Position").Publish();
inline nt::StringPublisher ServoLogging::statePublisher =
    nt::NetworkTableInstance::GetDefault().GetStringTopic("ServoSubsystem/State").Publish();
