#pragma once

#include "abstractions/io/intake/IntakeIO.hpp"
#include "frc/controller/PIDController.h"

class IntakeSimIO : public IntakeIO {
 public:
  void UpdateInputs(IntakeIOInputs& inputs) override {
    pivotPosition += pivotController.Calculate(pivotPosition, pivotSetpoint) * 0.02;

    inputs.rollerVoltage = rollerVoltage;
    inputs.pivotPosition = pivotPosition;
    inputs.pivotSetpoint = pivotSetpoint;
  }

  void SetIntakeVoltage(units::volt_t voltage) override { rollerVoltage = voltage; }

  void SetIntakePivotSetpoint(double setpoint) override { pivotSetpoint = setpoint; }

 private:
  units::volt_t rollerVoltage{0_V};

  frc::PIDController pivotController{10.0, 0.0, 0.0};

  double pivotSetpoint{0.0};
  double pivotPosition{0.0};
};