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
#include "subsystems/IntakeSubsystem.hpp"
#include "subsystems/PerceptionSubsystem.hpp"
#include "subsystems/ShooterSubsystem.hpp"
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/CommandXboxController.h>
#include <ctre/phoenix6/swerve/SwerveRequest.hpp>

using namespace ctre::phoenix6;

class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();

  IntakeSubsystem& GetIntakeSubsystem() { return m_intakeSubsystem; }
  HopperSubsystem& GetHopperSubsystem() { return m_hopperSubsystem; }
  ShooterSubsystem& GetShooterSubsystem() { return m_shooterSubsystem; }
  GateSubsystem& GetGateSubsystem() { return m_gateSubsystem; }

 private:
  void ConfigureBindings();
  void ConfigureIntakeBindings();
  void ConfigureHopperBindings();
  void ConfigureShooterBindings();

  frc2::CommandXboxController m_driverController{0};
  frc2::CommandXboxController m_operatorController{1};

  swerve::requests::FieldCentric drive =
      swerve::requests::FieldCentric{}
          .WithDeadband(DriveConstants::kMaxSpeed * 0.05)
          .WithRotationalDeadband(DriveConstants::kMaxAngularRate * 0.05)  // Add a 10% deadband
          .WithDriveRequestType(ctre::phoenix6::swerve::impl::DriveRequestType::OpenLoopVoltage);
  swerve::requests::SwerveDriveBrake brake{};
  swerve::requests::PointWheelsAt point{};

  Telemetry logger{DriveConstants::kMaxSpeed};

  subsystems::CommandSwerveDrivetrain m_driveSubsystem{TunerConstants::CreateDrivetrain()};
  PerceptionSubsystem m_perception{m_driveSubsystem};
  IntakeSubsystem m_intakeSubsystem;
  ShooterSubsystem m_shooterSubsystem;
  HopperSubsystem m_hopperSubsystem;
  GateSubsystem m_gateSubsystem;
};
