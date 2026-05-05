#pragma once

#include "frc2/command/CommandPtr.h"
#include "frc2/command/Commands.h"
#include "subsystems/GateSubsystem.hpp"
#include "subsystems/HopperSubsystem.hpp"
#include "subsystems/IntakeSubsystem.hpp"

namespace CommandFactory {
inline frc2::CommandPtr OuttakeCommand(IntakeSubsystem& intake, HopperSubsystem& hopper) {
  return intake.RunOuttakeCommand().AlongWith(hopper.RunOuttakeCommand());
}

inline frc2::CommandPtr PassThroughCommand(IntakeSubsystem& intake, HopperSubsystem& hopper, GateSubsystem& gate) {
  return frc2::cmd::Parallel(hopper.RunHopperCommand(), intake.SlowZeroCommand(), gate.RunGateCommand());
};
}  // namespace CommandFactory