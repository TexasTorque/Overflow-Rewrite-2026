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
  HopperLogging()
      : voltagePublisher{nt::NetworkTableInstance::GetDefault().GetDoubleTopic("HopperSubsystem/Voltage").Publish()},
        currentPublisher{nt::NetworkTableInstance::GetDefault().GetDoubleTopic("HopperSubsystem/Current").Publish()},
        statePublisher{nt::NetworkTableInstance::GetDefault().GetStringTopic("HopperSubsystem/State").Publish()} {}

  void UpdateTelemetry(const HopperIOInputs& inputs, HopperStateEnum state) {
    voltagePublisher.Set(inputs.hopperVoltage.value());
    currentPublisher.Set(inputs.hopperCurrent.value());
    statePublisher.Set(stateToString(state));
  }

 private:
  nt::DoublePublisher voltagePublisher;
  nt::DoublePublisher currentPublisher;
  nt::StringPublisher statePublisher;
};
