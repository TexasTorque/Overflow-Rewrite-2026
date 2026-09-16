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
  ServoLogging()
      : positionPublisher{nt::NetworkTableInstance::GetDefault().GetDoubleTopic("ServoSubsystem/Position").Publish()},
        statePublisher{nt::NetworkTableInstance::GetDefault().GetStringTopic("ServoSubsystem/State").Publish()} {}

  void UpdateTelemetry(const ServoIOInputs& inputs, ServoStateEnum state) {
    positionPublisher.Set(inputs.servoSetpoint);
    statePublisher.Set(stateToString(state));
  }

 private:
  nt::DoublePublisher positionPublisher;
  nt::StringPublisher statePublisher;
};
