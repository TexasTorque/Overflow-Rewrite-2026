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
  static void UpdateTelemetry(const GateIOInputs& inputs, GateStateEnum state) {
    voltagePublisher.Set(inputs.gateVoltage.value());
    currentPublisher.Set(inputs.gateCurrent.value());
    statePublisher.Set(stateToString(state));
  }

 private:
  static nt::DoublePublisher voltagePublisher;
  static nt::DoublePublisher currentPublisher;
  static nt::StringPublisher statePublisher;
};

inline nt::DoublePublisher GateLogging::voltagePublisher =
    nt::NetworkTableInstance::GetDefault().GetDoubleTopic("GateSubsystem/Voltage").Publish();
inline nt::DoublePublisher GateLogging::currentPublisher =
    nt::NetworkTableInstance::GetDefault().GetDoubleTopic("GateSubsystem/Current").Publish();
inline nt::StringPublisher GateLogging::statePublisher =
    nt::NetworkTableInstance::GetDefault().GetStringTopic("GateSubsystem/State").Publish();
