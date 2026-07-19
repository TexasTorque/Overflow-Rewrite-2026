// Texas Torque 1477
// Overflow 2026

#pragma once

#include <functional>
#include <string>
#include <string_view>

#include "abstractions/state/IntakeState.hpp"
#include "constants/Constants.hpp"
#include "ctre/phoenix6/Orchestra.hpp"
#include "frc/smartdashboard/SmartDashboard.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/Commands.h"
#include "abstractions/state/GateState.hpp"
#include "abstractions/state/HopperState.hpp"
#include "abstractions/state/ServoState.hpp"
#include "abstractions/state/ShooterState.hpp"
#include "subsystems/GateSubsystem.hpp"
#include "subsystems/HopperSubsystem.hpp"
#include "subsystems/IntakeSubsystem.hpp"
#include "subsystems/ServoSubsystem.hpp"
#include "subsystems/ShooterSubsystem.hpp"
#include "units/angular_velocity.h"
#include "units/length.h"

namespace CommandFactory {
inline frc2::CommandPtr OuttakeCommand(IntakeSubsystem& intake, HopperSubsystem& hopper, GateSubsystem& gate) {
  return frc2::cmd::Parallel(intake.RunOuttakeCommand(), hopper.RunOuttakeCommand(), gate.RunOuttakeCommand())
      .WithName("Outtake");
}

inline frc2::CommandPtr PassThroughCommand(HopperSubsystem& hopper, GateSubsystem& gate) {
  return frc2::cmd::Parallel(hopper.RunHopperCommand(), gate.RunGateCommand());
}

inline frc2::CommandPtr ShotCommand(std::function<frc2::CommandPtr()> makeShooterCmd,
                                    std::function<frc2::CommandPtr()> makeServoCmd, ShooterSubsystem& shooter,
                                    HopperSubsystem& hopper, GateSubsystem& gate) {
  return (frc2::cmd::Parallel(makeShooterCmd(), makeServoCmd()).Until([&] { return shooter.IsReadyToShoot(); }))
      .AndThen(PassThroughCommand(hopper, gate).AlongWith(frc2::cmd::Parallel(makeShooterCmd(), makeServoCmd())));
}

inline frc2::CommandPtr WithAutoPullup(std::function<frc2::CommandPtr()> makeShotCmd, IntakeSubsystem& intake) {
  return makeShotCmd()
      .AlongWith(frc2::cmd::Wait(ShooterConstants::kStartTime)
                     .AndThen((frc2::cmd::RunOnce([&] { intake.SetState(IntakeStateEnum::SlowPullup); })
                                   .AndThen(frc2::cmd::Wait(ShooterConstants::kHoldTime))
                                   .AndThen(frc2::cmd::RunOnce([&] { intake.SetState(IntakeStateEnum::Stow); }))
                                   .AndThen(frc2::cmd::Wait(ShooterConstants::kWaitTime)))
                                  .Repeatedly()))
      .FinallyDo([&] { intake.SetState(IntakeStateEnum::Stow); });
}

inline frc2::CommandPtr LayupShotCommand(ShooterSubsystem& shooter, ServoSubsystem& servo, HopperSubsystem& hopper,
                                         GateSubsystem& gate, IntakeSubsystem& intake) {
  return WithAutoPullup(
             [&] {
               return ShotCommand([&] { return shooter.RunLayupCommand(); }, [&] { return servo.SetServoUpCommand(); },
                                  shooter, hopper, gate);
             },
             intake)
      .WithName("Layup Shot");
}

inline frc2::CommandPtr LaserShotCommand(ShooterSubsystem& shooter, ServoSubsystem& servo, HopperSubsystem& hopper,
                                         GateSubsystem& gate, IntakeSubsystem& intake) {
  return WithAutoPullup(
             [&] {
               return ShotCommand([&] { return shooter.RunClimbCommand(); },
                                  [&] { return servo.SetServoLaserCommand(); }, shooter, hopper, gate);
             },
             intake)
      .WithName("Laser Shot");
}

inline frc2::CommandPtr ClimbShotCommand(ShooterSubsystem& shooter, ServoSubsystem& servo, HopperSubsystem& hopper,
                                         GateSubsystem& gate, IntakeSubsystem& intake) {
  return WithAutoPullup(
             [&] {
               return ShotCommand([&] { return shooter.RunClimbCommand(); }, [&] { return servo.SetServoUpCommand(); },
                                  shooter, hopper, gate);
             },
             intake)
      .WithName("Climb Shot");
}

inline frc2::CommandPtr TrenchShotCommand(ShooterSubsystem& shooter, ServoSubsystem& servo, HopperSubsystem& hopper,
                                          GateSubsystem& gate, IntakeSubsystem& intake) {
  return WithAutoPullup(
             [&] {
               return ShotCommand([&] { return shooter.RunTrenchCommand(); }, [&] { return servo.SetServoUpCommand(); },
                                  shooter, hopper, gate);
             },
             intake)
      .WithName("Trench Shot");
}

inline frc2::CommandPtr RegressionShotCommand(ShooterSubsystem& shooter, ServoSubsystem& servo, HopperSubsystem& hopper,
                                              GateSubsystem& gate, IntakeSubsystem& intake,
                                              std::function<units::meter_t()> distance) {
  return WithAutoPullup(
             [&] {
               return ShotCommand([&, distance] { return shooter.RunRegressionCommand(distance); },
                                  [&] { return servo.SetServoUpCommand(); }, shooter, hopper, gate);
             },
             intake)
      .WithName("Regression Shot");
}

inline frc2::CommandPtr DebugShotCommand(ShooterSubsystem& shooter, ServoSubsystem& servo, HopperSubsystem& hopper,
                                         GateSubsystem& gate, IntakeSubsystem& intake) {
  return WithAutoPullup(
             [&] {
               return ShotCommand(
                   [&] {
                     return shooter.RunDebugShotCommand([] {
                       return units::revolutions_per_minute_t{frc::SmartDashboard::GetNumber("Debug RPM", 0.0)};
                     });
                   },
                   [&] { return servo.SetServoUpCommand(); }, shooter, hopper, gate);
             },
             intake)
      .WithName("Debug Shot");
}

inline frc2::CommandPtr DebugShotCommand(ShooterSubsystem& shooter, ServoSubsystem& servo, HopperSubsystem& hopper,
                                         GateSubsystem& gate, IntakeSubsystem& intake,
                                         units::revolutions_per_minute_t rpm) {
  return WithAutoPullup(
             [&] {
               return ShotCommand([&] { return shooter.RunDebugShotCommand([rpm] { return rpm; }); },
                                  [&] { return servo.SetServoUpCommand(); }, shooter, hopper, gate);
             },
             intake)
      .WithName("Debug Shot");
}

inline frc2::CommandPtr StopShotCommand(ShooterSubsystem& shooter, ServoSubsystem& servo, HopperSubsystem& hopper,
                                        GateSubsystem& gate) {
  return frc2::cmd::RunOnce(
             [&] {
               shooter.SetState(ShooterStateEnum::Idle);
               servo.SetState(ServoStateEnum::Idle);
               hopper.SetState(HopperStateEnum::Off);
               gate.SetState(GateStateEnum::Off);
             },
             {&shooter, &servo, &hopper, &gate})
      .WithName("Stop Shot");
}

inline frc2::CommandPtr PlaySong(ctre::phoenix6::Orchestra& orchestra) {
  return frc2::cmd::RunOnce([&] { orchestra.Play(); });
}
}  // namespace CommandFactory
