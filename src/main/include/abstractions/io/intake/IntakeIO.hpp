#pragma once

#include "units/current.h"
#include "units/voltage.h"

struct IntakeIOInputs {
  units::volt_t rollerVoltage{0_V};
  units::ampere_t rollerCurrent{0_A};

  double pivotPosition{0.0};
  double pivotSetpoint{0.0};
};

class IntakeIO {
public:
  virtual void UpdateInputs(IntakeIOInputs &inputs) = 0;

  virtual void SetIntakeVoltage(units::volt_t voltage) = 0;
  virtual void SetIntakePivotSetpoint(double setpoint) = 0;

  virtual ~IntakeIO() = default;
};
