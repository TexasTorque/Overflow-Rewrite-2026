// Texas Torque 1477
// Overflow 2026

#pragma once

#include "units/current.h"
#include "units/voltage.h"

struct ServoIOInputs {
  double servoSetpoint{0.0};
};

class ServoIO {
 public:
  virtual void UpdateInputs(ServoIOInputs& inputs) = 0;

  virtual void SetServoPos(double servoSetpoint) = 0;

  virtual ~ServoIO() = default;
};
