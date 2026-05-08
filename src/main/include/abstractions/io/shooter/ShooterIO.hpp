#pragma once

#include "units/current.h"
#include "units/angular_velocity.h"

struct ShooterIOInputs {
  units::revolutions_per_minute_t flywheelRPMSetpoint{0_rpm};
  units::revolutions_per_minute_t flywheelRPM{0_rpm};

  bool flywheelNearDesired{false};
};

class ShooterIO {
 public:
  virtual void UpdateInputs(ShooterIOInputs& inputs) = 0;

  virtual void SetFlywheelRPM(units::revolutions_per_minute_t rpm) = 0;

  virtual ~ShooterIO() = default;
};
