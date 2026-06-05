// Texas Torque 1477
// Overflow 2026

#pragma once

#include "abstractions/io/servo/ServoIO.hpp"
#include "constants/Constants.hpp"
#include "frc/Servo.h"

class ServoRealIO : public ServoIO {
 public:
  ServoRealIO() {
    ConfigureServoLeft();
    ConfigureServoRight();
  }

  void UpdateInputs(ServoIOInputs& inputs) override { inputs.servoSetpoint = m_servoLeft.GetPosition(); }

  void SetServoPos(double servoSetpoint) override {
    m_servoLeft.SetPosition(servoSetpoint);
    m_servoLeft.SetPosition(1 - servoSetpoint);
  }

 private:
  frc::Servo m_servoRight{ServoConstants::kServoRightPort};

  frc::Servo m_servoLeft{ServoConstants::kServoLeftPort};

  void ConfigureServoRight() { m_servoRight.SetBounds(2500_us, 0_us, 0_us, 0_us, 500_us); }

  void ConfigureServoLeft() { m_servoLeft.SetBounds(2500_us, 0_us, 0_us, 0_us, 500_us); }
};
