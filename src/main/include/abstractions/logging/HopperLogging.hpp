// Texas Torque 1477
// Overflow 2026

#pragma once

#include "abstractions/state/SubsystemState.hpp"
#include "abstractions/io/hopper/HopperIO.hpp"
#include "abstractions/state/HopperState.hpp"
#include "networktables/DoubleTopic.h"
#include "networktables/NetworkTableInstance.h"
#include "networktables/StringTopic.h"

class HopperLogging {
 public:
  static void UpdateTelemetry(const HopperIOInputs& inputs, HopperStateEnum state) {
    voltagePublisher.Set(inputs.hopperVoltage.value());
    currentPublisher.Set(inputs.hopperCurrent.value());
    statePublisher.Set(stateToString(state));
  }

 private:
  static nt::DoublePublisher voltagePublisher;
  static nt::DoublePublisher currentPublisher;
  static nt::StringPublisher statePublisher;
};

inline nt::DoublePublisher HopperLogging::voltagePublisher =
    nt::NetworkTableInstance::GetDefault().GetDoubleTopic("HopperSubsystem/Voltage").Publish();
inline nt::DoublePublisher HopperLogging::currentPublisher =
    nt::NetworkTableInstance::GetDefault().GetDoubleTopic("HopperSubsystem/Current").Publish();
inline nt::StringPublisher HopperLogging::statePublisher =
    nt::NetworkTableInstance::GetDefault().GetStringTopic("HopperSubsystem/State").Publish();
