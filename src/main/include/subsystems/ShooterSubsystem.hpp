// Texas Torque 1477
// Overflow 2026

#pragma once

#include "abstractions/io/shooter/ShooterIO.hpp"
#include "abstractions/state/ShooterState.hpp"
#include "frc2/command/CommandPtr.h"
#include "units/angular_velocity.h"
#include <frc2/command/SubsystemBase.h>
#include <memory>

class ShooterSubsystem : public frc2::SubsystemBase {
 public:
  explicit ShooterSubsystem(std::unique_ptr<ShooterIO> io);

  void SetFlywheelVelocity(units::revolutions_per_minute_t rpm) { m_io->SetFlywheelRPM(rpm); }

  frc2::CommandPtr RunLayupCommand();
  frc2::CommandPtr RunLaserCommand();
  frc2::CommandPtr RunClimbCommand();
  frc2::CommandPtr RunTrenchCommand();
  frc2::CommandPtr RunRegressionCommand();

  void SetState(const ShooterStateEnum& newState);
  void Clean();

  void Periodic() override;

  ShooterStateEnum GetState() const { return m_state.Get(); }

 private:
  std::unique_ptr<ShooterIO> m_io;
  ShooterIOInputs m_inputs;
  ShooterState m_state;

  void ApplyState(const ShooterStateEnum& newState);
};
