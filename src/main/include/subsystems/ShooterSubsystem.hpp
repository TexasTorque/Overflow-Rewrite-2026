// Texas Torque 1477
// Overflow 2026

#pragma once

#include "abstractions/io/shooter/ShooterIO.hpp"
#include "abstractions/logging/ShooterLogging.hpp"
#include "abstractions/state/ShooterState.hpp"
#include "constants/Constants.hpp"
#include "frc2/command/CommandPtr.h"
#include "turbolib/subsystem/TurboSubsystem.hpp"
#include "units/angular_velocity.h"
#include "units/length.h"
#include <functional>

class ShooterSubsystem : public turbolib::TurboSubsystem<ShooterIO, ShooterStateEnum> {
 public:
  explicit ShooterSubsystem(std::unique_ptr<ShooterIO> io);

  void SetFlywheelVelocity(units::revolutions_per_minute_t rpm) { m_io->SetFlywheelRPM(rpm); }

  frc2::CommandPtr RunPrespinCommand();
  frc2::CommandPtr RunLayupCommand();
  frc2::CommandPtr RunLaserCommand();
  frc2::CommandPtr RunClimbCommand();
  frc2::CommandPtr RunTrenchCommand();
  frc2::CommandPtr RunRegressionCommand(std::function<units::meter_t()> distance);
  frc2::CommandPtr RunDebugShotCommand(std::function<units::revolutions_per_minute_t()> rpm);

  bool IsReadyToShoot() const {
    if (m_inputs.flywheelRPMSetpoint == 0_rpm || m_inputs.flywheelRPMSetpoint == ShooterConstants::kIdleRPM) {
      return false;
    }

    return m_inputs.flywheelNearDesired;
  }

 protected:
  void ApplyState(const ShooterStateEnum& newState) override;
  ShooterStateEnum GetCleanState() const override { return ShooterStateEnum::Off; }
  void UpdateTelemetry() override;

 private:
  ShooterLogging m_logging;

  units::meter_t m_distance = 0.0_m;
  units::revolutions_per_minute_t m_debugRPM = 0.0_rpm;
};
