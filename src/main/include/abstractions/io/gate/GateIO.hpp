#pragma once

#include "units/current.h"
#include "units/voltage.h"

struct GateIOInputs {
  units::volt_t gateVoltage{0_V};
  units::ampere_t gateCurrent{0_A};
};

class GateIO {
 public:
  virtual void UpdateInputs(GateIOInputs& inputs) = 0;

  virtual void SetGateVoltage(units::volt_t voltage) = 0;

  virtual ~GateIO() = default;
};