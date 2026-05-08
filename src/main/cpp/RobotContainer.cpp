// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "RobotContainer.h"
#include "abstractions/io/gate/GateIO.hpp"
#include "abstractions/io/gate/GateRealIO.hpp"
#include "abstractions/io/gate/GateSimIO.hpp"
#include "abstractions/io/hopper/HopperIO.hpp"
#include "abstractions/io/hopper/HopperRealIO.hpp"
#include "abstractions/io/hopper/HopperSimIO.hpp"
#include "abstractions/io/intake/IntakeIO.hpp"
#include "abstractions/io/intake/IntakeRealIO.hpp"
#include "abstractions/io/intake/IntakeSimIO.hpp"
#include "constants/Constants.hpp"
#include "factory/CommandFactory.hpp"
#include "subsystems/IntakeSubsystem.hpp"
#include "turbolib/util/MakeIO.hpp"
#include <frc2/command/button/RobotModeTriggers.h>

#include <frc2/command/Commands.h>

RobotContainer::RobotContainer()
    : m_intakeSubsystem(turbolib::utils::MakeIO<IntakeIO, IntakeRealIO, IntakeSimIO>()),
      m_hopperSubsystem(turbolib::utils::MakeIO<HopperIO, HopperRealIO, HopperSimIO>()),
      m_gateSubsystem(turbolib::utils::MakeIO<GateIO, GateRealIO, GateSimIO>()) {
  ConfigureBindings();
  ConfigureIntakeBindings();
  ConfigureHopperBindings();
}

void RobotContainer::ConfigureBindings() {
  m_operatorController.POVUp().WhileTrue(
      CommandFactory::OuttakeCommand(m_intakeSubsystem, m_hopperSubsystem, m_gateSubsystem));
  m_operatorController.A().WhileTrue(
      CommandFactory::PassThroughCommand(m_intakeSubsystem, m_hopperSubsystem, m_gateSubsystem));

  m_driveSubsystem.SetDefaultCommand(m_driveSubsystem.ApplyRequest([this]() -> auto&& {
    return drive.WithVelocityX(-m_driverController.GetLeftY() * DriveConstants::kMaxSpeed)
        .WithVelocityY(-m_driverController.GetLeftX() * DriveConstants::kMaxSpeed)
        .WithRotationalRate(-m_driverController.GetRightX() * DriveConstants::kMaxAngularRate);
  }));

  frc2::RobotModeTriggers::Disabled().WhileTrue(
      m_driveSubsystem.ApplyRequest([] { return swerve::requests::Idle{}; }).IgnoringDisable(true));

  m_driveSubsystem.RegisterTelemetry([this](auto const& state) { logger.Telemeterize(state); });
}

void RobotContainer::ConfigureIntakeBindings() {
  m_operatorController.RightBumper().ToggleOnTrue(m_intakeSubsystem.RunIntakeCommand());
}

void RobotContainer::ConfigureHopperBindings() {}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
  return frc2::cmd::Print("No autonomous command configured");
}