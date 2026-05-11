#pragma once

#include "abstractions/io/shooter/ShooterIO.hpp"
#include "constants/constants.hpp"
#include "ctre/phoenix6/TalonFX.hpp"
#include "ctre/phoenix6/configs/Configurator.hpp"
#include "ctre/phoenix6/controls/ControlRequest.hpp"
#include "ctre/phoenix6/controls/Follower.hpp"
#include "ctre/phoenix6/controls/VelocityVoltage.hpp"
#include "ctre/phoenix6/core/CoreTalonFX.hpp"
#include "ctre/phoenix6/signals/SpnEnums.hpp"
#include "rev/ConfigureTypes.h"
#include "rev/SparkBase.h"
#include "rev/SparkLowLevel.h"
#include "rev/SparkMax.h"
#include "rev/config/SparkBaseConfig.h"
#include "rev/config/SparkMaxConfig.h"
#include "units/current.h"
#include "units/voltage.h"
#include "units/angular_velocity.h"

using namespace std;

class ShooterRealIO : public ShooterIO {
  public:
    ShooterRealIO() {
      ConfigureFlywheelRightMotor();
      ConfigureFlywheelLeftMotor();
    }

    void UpdateInputs(ShooterIOInputs& inputs) override {
      inputs.flywheelRPMSetpoint = units::revolutions_per_minute_t{rpmRequest.Velocity};
      inputs.flywheelRPM = m_flywheelMotorRight.GetVelocity().GetValue();

      inputs.flywheelNearDesired = abs(rpmRequest.Velocity.value() - inputs.flywheelRPM.value()) < ShooterConstants::kFlywheelTolerance;
    }

    void SetFlywheelRPM(units::revolutions_per_minute_t rpm) override { m_flywheelMotorRight.SetControl(rpmRequest.WithVelocity(rpm)); }

    private:
      ctre::phoenix6::hardware::TalonFX m_flywheelMotorRight{ShooterConstants::kFlywheelMotorRightPort};
      ctre::phoenix6::hardware::TalonFX m_flywheelMotorLeft{ShooterConstants::kFlywheelMotorLeftPort};

      ctre::phoenix6::controls::VelocityVoltage rpmRequest{0_rpm};

      void ConfigureFlywheelRightMotor() {
        ctre::phoenix6::configs::TalonFXConfiguration conf;

        conf.CurrentLimits.WithStatorCurrentLimit(100_A);
        conf.CurrentLimits.WithSupplyCurrentLimit(50_A);
        conf.MotorOutput.WithNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);

        conf.Slot0.kP = ShooterConstants::kFlywheelkP;
        conf.Slot0.kI = ShooterConstants::kFlywheelkI;
        conf.Slot0.kD = ShooterConstants::kFlywheelkD;
        conf.Slot0.kS = ShooterConstants::kFlywheelkS;
        conf.Slot0.kV = ShooterConstants::kFlywheelkV;

        m_flywheelMotorRight.GetConfigurator().Apply(conf);
        m_flywheelMotorLeft.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);
      }

      void ConfigureFlywheelLeftMotor() {
        ctre::phoenix6::configs::TalonFXConfiguration conf;

        conf.CurrentLimits.WithStatorCurrentLimit(100_A);
        conf.CurrentLimits.WithSupplyCurrentLimit(50_A);
        conf.MotorOutput.WithNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);

        m_flywheelMotorLeft.GetConfigurator().Apply(conf);
        m_flywheelMotorLeft.SetControl(ctre::phoenix6::controls::Follower{m_flywheelMotorRight.GetDeviceID(), true});
        m_flywheelMotorLeft.SetNeutralMode(ctre::phoenix6::signals::NeutralModeValue::Coast);
      }
};