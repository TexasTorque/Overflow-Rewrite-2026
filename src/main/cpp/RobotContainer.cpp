// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "abstractions/io/intake/IntakeIO.hpp"
#include "abstractions/io/intake/IntakeRealIO.hpp"
#include "abstractions/io/intake/IntakeSimIO.hpp"
#include "subsystems/IntakeSubsystem.hpp"
#include "turbolib/util/MakeIO.hpp"

#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() : intakeSubsystem(turbolib::utils::MakeIO<IntakeIO, IntakeRealIO, IntakeSimIO>()) {
  ConfigureBindings();
  ConfigureIntakeBindings();
}

void RobotContainer::ConfigureBindings() {}

void RobotContainer::ConfigureIntakeBindings() {
  operatorController.A().ToggleOnTrue(intakeSubsystem.RunIntakeCommand());
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}
