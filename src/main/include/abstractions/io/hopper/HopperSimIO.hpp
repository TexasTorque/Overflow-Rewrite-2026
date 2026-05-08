#pragma once

#include "abstractions/io/hopper/HopperIO.hpp"
#include "units/voltage.h"

class HopperSimIO : public HopperIO {
 public:
  void UpdateInputs(HopperIOInputs& inputs) override {
    inputs.hopperVoltage = m_voltage;
    inputs.hopperCurrent = 0_A;
  }

  void SetHopperVoltage(units::volt_t voltage) override { m_voltage = voltage; }

 private:
  units::volt_t m_voltage;
};