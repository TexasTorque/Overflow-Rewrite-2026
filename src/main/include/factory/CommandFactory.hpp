// Texas Torque 1477
// Overflow 2026

#pragma once

#include "frc2/command/CommandPtr.h"
#include "frc2/command/Commands.h"
#include "subsystems/GateSubsystem.hpp"
#include "subsystems/HopperSubsystem.hpp"
#include "subsystems/IntakeSubsystem.hpp"
#include "subsystems/ServoSubsystem.hpp"
#include "subsystems/ShooterSubsystem.hpp"

namespace CommandFactory {
inline frc2::CommandPtr OuttakeCommand(IntakeSubsystem& intake, HopperSubsystem& hopper, GateSubsystem& gate) {
  return frc2::cmd::Parallel(intake.RunOuttakeCommand(), hopper.RunOuttakeCommand(), gate.RunOuttakeCommand());
}

inline frc2::CommandPtr PassThroughCommand(HopperSubsystem& hopper, GateSubsystem& gate) {
  return frc2::cmd::Parallel(hopper.RunHopperCommand(), gate.RunGateCommand());
}

// since default servo state is down, need to bring it up for any shots
inline frc2::CommandPtr LayupShotCommand(ShooterSubsystem& shooter, ServoSubsystem& servo) {
  return frc2::cmd::Parallel(shooter.RunLayupCommand(), servo.SetServoUpCommand());
}

inline frc2::CommandPtr LaserShotCommand(ShooterSubsystem& shooter) {
  return shooter.RunLaserCommand(); // only one where servo stays down
}

inline frc2::CommandPtr ClimbShotCommand(ShooterSubsystem& shooter, ServoSubsystem& servo) {
  return frc2::cmd::Parallel(shooter.RunClimbCommand(), servo.SetServoUpCommand());
}

inline frc2::CommandPtr TrenchShotCommand(ShooterSubsystem& shooter, ServoSubsystem& servo) {
  return frc2::cmd::Parallel(shooter.RunTrenchCommand(), servo.SetServoUpCommand());
}

inline frc2::CommandPtr RegressionShotCommand(ShooterSubsystem& shooter, ServoSubsystem& servo) {
  return frc2::cmd::Parallel(shooter.RunRegressionCommand(), servo.SetServoUpCommand());
}
}  // namespace CommandFactory
