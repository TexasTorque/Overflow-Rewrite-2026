// Texas Torque 1477
// Overflow 2026

#pragma once

#include "abstractions/io/hopper/HopperIO.hpp"
#include "constants/Constants.hpp"
#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/controls/VoltageOut.hpp"
#include "ctre/phoenix6/core/CoreTalonFX.hpp"
#include "units/voltage.h"

class HopperRealIO : public HopperIO {
 public:
  HopperRealIO() { ConfigureHopperMotor(); }

  void UpdateInputs(HopperIOInputs& inputs) override {
    inputs.hopperVoltage = m_hopperMotor.GetMotorVoltage().GetValue();
    inputs.hopperCurrent = m_hopperMotor.GetSupplyCurrent().GetValue();
  }

  void SetHopperVoltage(units::volt_t voltage) override {
    m_hopperMotor.SetControl(m_voltageRequest.WithOutput(voltage));
  }

 private:
  ctre::phoenix6::hardware::TalonFX m_hopperMotor{HopperConstants::kHopperMotorPort};

  ctre::phoenix6::controls::VoltageOut m_voltageRequest{0_V};

  void ConfigureHopperMotor() {
    ctre::phoenix6::configs::TalonFXConfiguration config;

    config.CurrentLimits.StatorCurrentLimit = 70_A;
    config.CurrentLimits.SupplyCurrentLimit = 40_A;

    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.MotorOutput.Inverted = true;

    m_hopperMotor.GetConfigurator().Apply(config);
  }
};
