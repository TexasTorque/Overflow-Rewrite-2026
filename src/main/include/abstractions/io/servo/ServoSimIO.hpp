// Texas Torque 1477
// Overflow 2026

#pragma once

#include "abstractions/io/servo/ServoIO.hpp"

class ServoSimIO : public ServoIO {
  void UpdateInputs(ServoIOInputs& inputs) override { inputs.servoSetpoint = m_servoPosition; }

  void SetServoPos(double servoSetpoint) override { m_servoPosition = servoSetpoint; }

 private:
  double m_servoPosition{0};
};
