#pragma once

#include "units/voltage.h"

struct HopperIOInputs {
  units::volt_t hopperVoltage{0_V};
  units::volt_t voltageRequest{0_V};
};

class HopperIO {
 public:
  virtual void UpdateInputs(HopperIOInputs& inputs) = 0;

  virtual void SetHopperVoltage(units::volt_t voltage) = 0;

  virtual ~HopperIO() = default;
};