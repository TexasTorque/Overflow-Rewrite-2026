// Texas Torque 1477
// Overflow 2026

#pragma once

#include "ctre/phoenix6/TalonFX.hpp"
#include "units/current.h"
#include "units/voltage.h"

struct HopperIOInputs {
  units::volt_t hopperVoltage{0_V};
  units::ampere_t hopperCurrent{0_A};
};

class HopperIO {
 public:
  virtual void UpdateInputs(HopperIOInputs& inputs) = 0;

  virtual void SetHopperVoltage(units::volt_t voltage) = 0;

  virtual ctre::phoenix6::hardware::TalonFX& GetHopperMotor() = 0;

  virtual ~HopperIO() = default;
};
