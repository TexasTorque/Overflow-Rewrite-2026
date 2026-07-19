// Texas Torque 1477
// Overflow 2026

#pragma once

#include "abstractions/io/shooter/ShooterIO.hpp"
#include "constants/Constants.hpp"
#include "units/angular_velocity.h"
#include <cmath>

class ShooterSimIO : public ShooterIO {
 public:
  void UpdateInputs(ShooterIOInputs& inputs) override {
    inputs.flywheelRPMSetpoint = m_targetRPM;

    double error = m_targetRPM.value() - m_currentRPM.value();
    m_currentRPM += units::revolutions_per_minute_t{error * 0.08};

    inputs.flywheelRPM = m_currentRPM;
    inputs.flywheelCurrent = 0_A;
    inputs.flywheelNearDesired =
        std::abs(m_targetRPM.value() - m_currentRPM.value()) < ShooterConstants::kFlywheelTolerance;
  }

  void SetFlywheelRPM(units::revolutions_per_minute_t rpm) override { m_targetRPM = rpm; }

  ctre::phoenix6::hardware::TalonFX& GetShooterLeftMotor() override;
  ctre::phoenix6::hardware::TalonFX& GetShooterRightMotor() override;

 private:
  units::revolutions_per_minute_t m_targetRPM{0_rpm};
  units::revolutions_per_minute_t m_currentRPM{0_rpm};
};
