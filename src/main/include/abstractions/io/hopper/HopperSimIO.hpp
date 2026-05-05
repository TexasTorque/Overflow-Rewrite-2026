#pragma once

#include "abstractions/io/hopper/HopperIO.hpp"
#include "units/voltage.h"

class HopperSimIO : public HopperIO {
 public:
  void UpdateInputs(HopperIOInputs& inputs) override {
    inputs.hopperVoltage = m_voltage;
    inputs.voltageRequest = m_voltage;
  }

  void SetHopperVoltage(units::volt_t voltage) override { m_voltage = voltage; }

 private:
  units::volt_t m_voltage;
};