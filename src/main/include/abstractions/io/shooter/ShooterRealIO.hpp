// Texas Torque 1477
// Overflow 2026

#pragma once

#include "abstractions/io/shooter/ShooterIO.hpp"
#include "constants/Constants.hpp"
#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/controls/Follower.hpp"
#include "ctre/phoenix6/controls/VelocityVoltage.hpp"
#include "ctre/phoenix6/core/CoreTalonFX.hpp"
#include "ctre/phoenix6/signals/SpnEnums.hpp"
#include "units/current.h"
#include "units/angular_velocity.h"

class ShooterRealIO : public ShooterIO {
 public:
  ShooterRealIO() {
    ConfigureFlywheelRightMotor();
    ConfigureFlywheelLeftMotor();
  }

  void UpdateInputs(ShooterIOInputs& inputs) override {
    inputs.flywheelRPMSetpoint = rpmRequest.Velocity;
    inputs.flywheelRPM = m_flywheelMotorRight.GetVelocity().GetValue();
    inputs.flywheelCurrent = m_flywheelMotorRight.GetSupplyCurrent().GetValue();

    inputs.flywheelNearDesired = std::abs(units::revolutions_per_minute_t{rpmRequest.Velocity}.value() -
                                          inputs.flywheelRPM.value()) < ShooterConstants::kFlywheelTolerance;
  }

  void SetFlywheelRPM(units::revolutions_per_minute_t rpm) override {
    m_flywheelMotorRight.SetControl(rpmRequest.WithVelocity(rpm));
  }

 private:
  ctre::phoenix6::hardware::TalonFX m_flywheelMotorRight{ShooterConstants::kFlywheelMotorRightPort};
  ctre::phoenix6::hardware::TalonFX m_flywheelMotorLeft{ShooterConstants::kFlywheelMotorLeftPort};

  ctre::phoenix6::controls::VelocityVoltage rpmRequest{0_rpm};

  void ConfigureFlywheelRightMotor() {
    ctre::phoenix6::configs::TalonFXConfiguration conf;

    conf.CurrentLimits.StatorCurrentLimit = 100_A;
    conf.CurrentLimits.SupplyCurrentLimit = 50_A;

    conf.CurrentLimits.StatorCurrentLimitEnable = true;
    conf.CurrentLimits.SupplyCurrentLimitEnable = true;

    conf.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;

    conf.Slot0.kP = ShooterConstants::kFlywheelkP;
    conf.Slot0.kI = ShooterConstants::kFlywheelkI;
    conf.Slot0.kD = ShooterConstants::kFlywheelkD;
    conf.Slot0.kS = ShooterConstants::kFlywheelkS;
    conf.Slot0.kV = ShooterConstants::kFlywheelkV;

    m_flywheelMotorRight.GetConfigurator().Apply(conf);

    m_flywheelMotorRight.OptimizeBusUtilization(4_Hz);
    m_flywheelMotorRight.GetVelocity(false).SetUpdateFrequency(50_Hz);
  }

  void ConfigureFlywheelLeftMotor() {
    ctre::phoenix6::configs::TalonFXConfiguration conf;

    conf.CurrentLimits.StatorCurrentLimit = 100_A;
    conf.CurrentLimits.SupplyCurrentLimit = 50_A;

    conf.CurrentLimits.StatorCurrentLimitEnable = true;
    conf.CurrentLimits.SupplyCurrentLimitEnable = true;

    conf.MotorOutput.NeutralMode = ctre::phoenix6::signals::NeutralModeValue::Coast;

    m_flywheelMotorLeft.GetConfigurator().Apply(conf);

    m_flywheelMotorLeft.SetControl(ctre::phoenix6::controls::Follower{m_flywheelMotorRight.GetDeviceID(), true});
    m_flywheelMotorLeft.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);

    m_flywheelMotorLeft.OptimizeBusUtilization(4_Hz);
  }
};
