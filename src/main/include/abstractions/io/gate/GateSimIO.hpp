#pragma once

#include "abstractions/io/gate/GateIO.hpp"
#include "units/voltage.h"

class GateSimIO : public GateIO {
 public:
  void UpdateInputs(GateIOInputs& inputs) override { inputs.gateVoltage = m_voltage; }

  void SetGateVoltage(units::volt_t voltage) override { m_voltage = voltage; }

 private:
  units::volt_t m_voltage;
};