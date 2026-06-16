// Texas Torque 1477
// Overflow 2026

#pragma once

#include "abstractions/io/gate/GateIO.hpp"
#include "constants/Constants.hpp"
#include "rev/ConfigureTypes.h"
#include "rev/SparkLowLevel.h"
#include "rev/SparkMax.h"
#include "rev/config/SparkMaxConfig.h"
#include "units/voltage.h"

class GateRealIO : public GateIO {
 public:
  GateRealIO() {
    ConfigureLeftGateMotor();
    ConfigureRightGateMotor();
  }

  void UpdateInputs(GateIOInputs& inputs) override {
    inputs.gateVoltage = units::volt_t{m_gateRightMotor.GetBusVoltage() * m_gateRightMotor.GetAppliedOutput()};
    inputs.gateCurrent = units::ampere_t{m_gateRightMotor.GetOutputCurrent()};
  }

  void SetGateVoltage(units::volt_t voltage) override { m_gateRightMotor.SetVoltage(voltage); }

 private:
  rev::spark::SparkMax m_gateRightMotor{GateConstants::kGateRightMotorPort,
                                        rev::spark::SparkLowLevel::MotorType::kBrushless};
  rev::spark::SparkMax m_gateLeftMotor{GateConstants::kGateLeftMotorPort,
                                       rev::spark::SparkLowLevel::MotorType::kBrushless};

  void ConfigureLeftGateMotor() {
    rev::spark::SparkMaxConfig config;

    config.SmartCurrentLimit(40);
    config.VoltageCompensation(12);
    config.Follow(GateConstants::kGateRightMotorPort, false);

    config.signals.AppliedOutputPeriodMs(45);
    config.signals.MotorTemperaturePeriodMs(1000);
    config.signals.FaultsPeriodMs(1000);

    m_gateLeftMotor.Configure(config, rev::ResetMode::kResetSafeParameters, rev::PersistMode::kPersistParameters);
  }

  void ConfigureRightGateMotor() {
    rev::spark::SparkMaxConfig config;

    config.SmartCurrentLimit(40);
    config.VoltageCompensation(12);
    config.Inverted(true);

    config.signals.AppliedOutputPeriodMs(45);
    config.signals.OutputCurrentPeriodMs(100);
    config.signals.MotorTemperaturePeriodMs(1000);
    config.signals.FaultsPeriodMs(1000);

    m_gateRightMotor.Configure(config, rev::ResetMode::kResetSafeParameters, rev::PersistMode::kPersistParameters);
  }
};
