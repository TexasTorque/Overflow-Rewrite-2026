// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "Telemetry.h"
#include "constants/Constants.hpp"
#include "generated/TunerConstants.h"
#include "subsystems/CommandSwerveDrivetrain.h"
#include "subsystems/GateSubsystem.hpp"
#include "subsystems/HopperSubsystem.hpp"
#include "subsystems/HubSubsystem.hpp"
#include "subsystems/IntakeSubsystem.hpp"
#include "subsystems/LEDSubsystem.hpp"
#include "subsystems/PerceptionSubsystem.hpp"
#include "subsystems/ServoSubsystem.hpp"
#include "subsystems/ShooterSubsystem.hpp"
#include "utils/AutoChooser.hpp"
#include <ctre/phoenix6/swerve/SwerveRequest.hpp>
#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <optional>

using namespace ctre::phoenix6;

class RobotContainer {
 public:
  RobotContainer();

  subsystems::CommandSwerveDrivetrain& GetDriveSubsystem() { return m_driveSubsystem; }
  IntakeSubsystem& GetIntakeSubsystem() { return m_intakeSubsystem; }
  HopperSubsystem& GetHopperSubsystem() { return m_hopperSubsystem; }
  ShooterSubsystem& GetShooterSubsystem() { return m_shooterSubsystem; }
  GateSubsystem& GetGateSubsystem() { return m_gateSubsystem; }
  ServoSubsystem& GetServoSubsystem() { return m_servoSubsystem; }
  PerceptionSubsystem& GetPerceptionSubsystem() { return m_perceptionSubsystem; }

  frc2::Command* GetAutonomousCommand();

 private:
  void ConfigurePlannerCommands();
  void ConfigureBindings();
  void ConfigureIntakeBindings();
  void ConfigureShooterBindings();
  void ConfigureLEDBindings();

  frc2::CommandPtr WithShotSetup(frc2::CommandPtr shotCommand);

  frc2::CommandXboxController m_driverController{0};
  frc2::CommandXboxController m_operatorController{1};

  swerve::requests::FieldCentric m_drive = swerve::requests::FieldCentric{}.WithDriveRequestType(
      ctre::phoenix6::swerve::impl::DriveRequestType::OpenLoopVoltage);
  swerve::requests::SwerveDriveBrake m_brake{};
  swerve::requests::PointWheelsAt m_point{};

  Telemetry logger{DriveConstants::kMaxSpeed};

  subsystems::CommandSwerveDrivetrain m_driveSubsystem{TunerConstants::CreateDrivetrain()};
  PerceptionSubsystem m_perceptionSubsystem{m_driveSubsystem};
  IntakeSubsystem m_intakeSubsystem;
  ShooterSubsystem m_shooterSubsystem;
  HopperSubsystem m_hopperSubsystem;
  GateSubsystem m_gateSubsystem;
  ServoSubsystem m_servoSubsystem;
  HubSubsystem m_hubSubsystem;
  LEDSubsystem m_ledSubsystem;

  std::optional<AutoChooser> m_autoChooser;
};
