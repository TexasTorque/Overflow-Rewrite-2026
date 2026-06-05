// Texas Torque 1477
// Overflow 2026

#pragma once

#include "abstractions/io/intake/IntakeIO.hpp"
#include "constants/constants.hpp"
#include "rev/ClosedLoopTypes.h"
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

  void UpdateInputs(IntakeIOInputs& inputs) override {
    inputs.rollerVoltage = units::volt_t{m_rollerMotorRight.GetAppliedOutput() * m_rollerMotorRight.GetBusVoltage()};
    inputs.rollerCurrent = units::ampere_t{m_rollerMotorRight.GetOutputCurrent()};
    inputs.pivotPosition = m_rotaryMotor.GetEncoder().GetPosition();
    inputs.pivotSetpoint = m_rotaryMotor.GetClosedLoopController().GetSetpoint();
  }

  void SetIntakeVoltage(units::volt_t voltage) override { m_rollerMotorRight.SetVoltage(voltage); }

  void SetIntakePivotSetpoint(double setpoint, bool slow = false) override {
    m_rotaryMotor.GetClosedLoopController().SetSetpoint(setpoint, rev::spark::SparkLowLevel::ControlType::kPosition);
  }

 private:
  rev::spark::SparkMax m_rotaryMotor{IntakeConstants::kIntakeRotaryMotorPort,
                                     rev::spark::SparkLowLevel::MotorType::kBrushless};

  rev::spark::SparkMax m_rollerMotorRight{IntakeConstants::kIntakeRollerMotorRightPort,
                                          rev::spark::SparkLowLevel::MotorType::kBrushless};

  rev::spark::SparkMax m_rollerMotorLeft{IntakeConstants::kIntakeRollerMotorLeftPort,
                                         rev::spark::SparkLowLevel::MotorType::kBrushless};

  void ConfigureRotaryMotor() {
    rev::spark::SparkMaxConfig config;

    config.SmartCurrentLimit(40);
    config.SetIdleMode(rev::spark::SparkBaseConfig::kBrake);
    config.Inverted(true);
    config.VoltageCompensation(12);

    config.closedLoop.Pid(1.5, 0, 0, rev::spark::kSlot0);
    config.closedLoop.Pid(1.5, 0, 0, rev::spark::kSlot1);

    config.closedLoop.maxMotion.CruiseVelocity(30);
    config.closedLoop.maxMotion.MaxAcceleration(10);

    config.closedLoop.maxMotion.CruiseVelocity(15, rev::spark::kSlot1);
    config.closedLoop.maxMotion.MaxAcceleration(5, rev::spark::kSlot1);

    m_rotaryMotor.Configure(config, rev::ResetMode::kResetSafeParameters, rev::PersistMode::kPersistParameters);
  }

  void ConfigureRollerRightMotor() {
    rev::spark::SparkMaxConfig config;

    config.SmartCurrentLimit(40);
    config.VoltageCompensation(12);

    m_rollerMotorRight.Configure(config, rev::ResetMode::kResetSafeParameters, rev::PersistMode::kPersistParameters);
  }

  void ConfigureRollerLeftMotor() {
    rev::spark::SparkMaxConfig config;

    config.SmartCurrentLimit(40);
    config.Follow(IntakeConstants::kIntakeRollerMotorRightPort, true);
    config.VoltageCompensation(12);

    m_rollerMotorLeft.Configure(config, rev::ResetMode::kResetSafeParameters, rev::PersistMode::kPersistParameters);
  }
};
