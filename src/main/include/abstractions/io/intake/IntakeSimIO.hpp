#pragma once

#include "abstractions/io/intake/IntakeIO.hpp"
#include "frc/controller/PIDController.h"

class IntakeSimIO : public IntakeIO {
 public:
  void UpdateInputs(IntakeIOInputs& inputs) override {
    m_pivotPosition += m_pivotController.Calculate(m_pivotPosition, m_pivotSetpoint) * (m_slow ? 0.005 : 0.02);

    inputs.rollerVoltage = m_rollerVoltage;
    inputs.pivotPosition = m_pivotPosition;
    inputs.pivotSetpoint = m_pivotSetpoint;
  }

  void SetIntakeVoltage(units::volt_t voltage) override { m_rollerVoltage = voltage; }

  void SetIntakePivotSetpoint(double setpoint, bool slow = false) override {
    m_pivotSetpoint = setpoint;
    m_slow = slow;
  }

 private:
  units::volt_t m_rollerVoltage{0_V};

  frc::PIDController m_pivotController{10.0, 0.0, 0.0};

  double m_pivotSetpoint{0.0};
  double m_pivotPosition{0.0};
  bool m_slow = false;
};