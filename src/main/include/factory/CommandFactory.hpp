#pragma once

#include "abstractions/state/ShooterState.hpp"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/Commands.h"
#include "subsystems/GateSubsystem.hpp"
#include "subsystems/HopperSubsystem.hpp"
#include "subsystems/IntakeSubsystem.hpp"
#include "subsystems/ShooterSubsystem.hpp"

namespace CommandFactory {
inline frc2::CommandPtr OuttakeCommand(IntakeSubsystem& intake, HopperSubsystem& hopper, GateSubsystem& gate) {
  return frc2::cmd::Parallel(intake.RunOuttakeCommand(), hopper.RunOuttakeCommand(), gate.RunOuttakeCommand());
}

inline frc2::CommandPtr PassThroughCommand(IntakeSubsystem& intake, HopperSubsystem& hopper, GateSubsystem& gate) {
  return frc2::cmd::Parallel(hopper.RunHopperCommand(), intake.SlowZeroCommand(), gate.RunGateCommand());
};

inline frc2::CommandPtr LayupShotCommand(ShooterSubsystem& shooter) {
  return shooter.RunLayupCommand();
}

inline frc2::CommandPtr LaserShotCommand(ShooterSubsystem& shooter) {
  return shooter.RunLaserCommand();
}

inline frc2::CommandPtr TrenchShotCommand(ShooterSubsystem& shooter) {
  return shooter.RunTrenchCommand();
}

inline frc2::CommandPtr ClimbShotCommand(ShooterSubsystem& shooter) {
  return shooter.RunClimbCommand();
}

inline frc2::CommandPtr RegressionShotCommand(ShooterSubsystem& shooter) {
  return shooter.RunRegressionCommand();
}
}  // namespace CommandFactory