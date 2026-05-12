// Texas Torque 1477
// Overflow 2026

#pragma once

#include "abstractions/io/shooter/ShooterIO.hpp"
#include "units/angular_velocity.h"

class ShooterSimIO : public ShooterIO {
 public:
  void UpdateInputs(ShooterIOInputs& inputs) override {
    inputs.flywheelRPM = m_rpm;
    inputs.flywheelRPMSetpoint = m_rpm;
    inputs.flywheelCurrent = 0_A;
  }

  void SetFlywheelRPM(units::revolutions_per_minute_t rpm) override { m_rpm = rpm; }

 private:
  units::revolutions_per_minute_t m_rpm;
};
