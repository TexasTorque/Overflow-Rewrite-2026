// Texas Torque 1477
// Overflow 2026

#pragma once

#include "abstractions/state/SubsystemState.hpp"
#include "networktables/DoubleTopic.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/StringTopic.h"
#include "abstractions/io/gate/GateIO.hpp"
#include "abstractions/state/GateState.hpp"

class GateLogging {
 public:
  GateLogging()
      : voltagePublisher{nt::NetworkTableInstance::GetDefault().GetDoubleTopic("GateSubsystem/Voltage").Publish()},
        currentPublisher{nt::NetworkTableInstance::GetDefault().GetDoubleTopic("GateSubsystem/Current").Publish()},
        statePublisher{nt::NetworkTableInstance::GetDefault().GetStringTopic("GateSubsystem/State").Publish()} {}

  void UpdateTelemetry(const GateIOInputs& inputs, GateStateEnum state) {
    voltagePublisher.Set(inputs.gateVoltage.value());
    currentPublisher.Set(inputs.gateCurrent.value());
    statePublisher.Set(stateToString(state));
  }

 private:
  nt::DoublePublisher voltagePublisher;
  nt::DoublePublisher currentPublisher;
  nt::StringPublisher statePublisher;
};
