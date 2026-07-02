// Texas Torque 1477
// Overflow 2026

#pragma once

#include <vector>
#include "units/angular_velocity.h"
#include "units/length.h"

class Regression {
 public:
  static units::revolutions_per_minute_t PerformRPMRegression(units::meter_t distance) {
    double rpm = 0.0;

    for (double coeff : rpmCoefficients) {
      rpm = rpm * distance.value() + coeff;
    }

    return units::revolutions_per_minute_t{rpm};
  }

 private:
  static std::vector<double> rpmCoefficients;
};

inline std::vector<double> Regression::rpmCoefficients = {1050.00, 695.00};
