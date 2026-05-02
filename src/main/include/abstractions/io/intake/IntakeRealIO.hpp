#pragma once

#include "abstractions/io/intake/IntakeIO.hpp"
#include "constants/constants.hpp"
#include "rev/ConfigureTypes.h"
#include "rev/SparkBase.h"
#include "rev/SparkLowLevel.h"
#include "rev/SparkMax.h"
#include "rev/config/SparkBaseConfig.h"
#include "rev/config/SparkMaxConfig.h"
#include "units/voltage.h"

class IntakeRealIO : public IntakeIO {
public:
  IntakeRealIO() {
    ConfigureRotaryMotor();
    ConfigureRollerRightMotor();
    ConfigureRollerLeftMotor();
  }

  void UpdateInputs(IntakeIOInputs &inputs) override {
    inputs.rollerVoltage = units::volt_t{rollerMotorRight.GetAppliedOutput() *
                                         rollerMotorRight.GetBusVoltage()};
    inputs.rollerCurrent = units::ampere_t{rollerMotorRight.GetOutputCurrent()};
    inputs.pivotPosition = rotaryMotor.GetEncoder().GetPosition();
    inputs.pivotSetpoint = rotaryMotor.GetClosedLoopController().GetSetpoint();
  }

  void SetIntakeVoltage(units::volt_t voltage) override {
    rollerMotorRight.SetVoltage(voltage);
  }
  void SetIntakePivotSetpoint(double setpoint) override {
    rotaryMotor.GetClosedLoopController().SetSetpoint(
        setpoint, rev::spark::SparkLowLevel::ControlType::kPosition);
  }

private:
  rev::spark::SparkMax rotaryMotor{
      IntakeConstants::kIntakeRotaryMotorPort,
      rev::spark::SparkLowLevel::MotorType::kBrushless};

  rev::spark::SparkMax rollerMotorRight{
      IntakeConstants::kIntakeRollerMotorRightPort,
      rev::spark::SparkLowLevel::MotorType::kBrushless};

  rev::spark::SparkMax rollerMotorLeft{
      IntakeConstants::kIntakeRollerMotorLeftPort,
      rev::spark::SparkLowLevel::MotorType::kBrushless};

  void ConfigureRotaryMotor() {
    rev::spark::SparkMaxConfig config;

    config.SmartCurrentLimit(40);
    config.SetIdleMode(rev::spark::SparkBaseConfig::kBrake);
    config.Inverted(true);
    config.VoltageCompensation(12);

    config.closedLoop.Pid(1.5, 0, 0);

    rotaryMotor.Configure(config, rev::ResetMode::kResetSafeParameters,
                          rev::PersistMode::kPersistParameters);
  }

  void ConfigureRollerRightMotor() {
    rev::spark::SparkMaxConfig config;

    config.SmartCurrentLimit(40);
    config.VoltageCompensation(12);

    rollerMotorRight.Configure(config, rev::ResetMode::kResetSafeParameters,
                               rev::PersistMode::kPersistParameters);
  }

  void ConfigureRollerLeftMotor() {
    rev::spark::SparkMaxConfig config;

    config.SmartCurrentLimit(40);
    config.Follow(IntakeConstants::kIntakeRollerMotorRightPort, true);
    config.VoltageCompensation(12);

    rollerMotorLeft.Configure(config, rev::ResetMode::kResetSafeParameters,
                              rev::PersistMode::kPersistParameters);
  }
};