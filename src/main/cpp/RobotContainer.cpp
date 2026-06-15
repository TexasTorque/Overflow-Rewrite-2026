// Texas Torque 1477
// Overflow 2026

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
#include "abstractions/io/servo/ServoRealIO.hpp"
#include "abstractions/io/servo/ServoSimIO.hpp"
#include "abstractions/io/shooter/ShooterIO.hpp"
#include "abstractions/io/shooter/ShooterRealIO.hpp"
#include "abstractions/io/shooter/ShooterSimIO.hpp"
#include "factory/CommandFactory.hpp"
#include "frc/smartdashboard/SmartDashboard.h"
#include "frc2/command/Command.h"
#include "pathplanner/lib/auto/AutoBuilder.h"
#include "pathplanner/lib/auto/NamedCommands.h"
#include "subsystems/IntakeSubsystem.hpp"
#include "turbolib/util/MakeIO.hpp"
#include <frc2/command/button/RobotModeTriggers.h>
#include <pathplanner/lib/events/EventTrigger.h>

#include <frc2/command/Commands.h>

RobotContainer::RobotContainer()
    : m_intakeSubsystem(turbolib::utils::MakeIO<IntakeIO, IntakeRealIO, IntakeSimIO>()),
      m_shooterSubsystem(turbolib::utils::MakeIO<ShooterIO, ShooterRealIO, ShooterSimIO>()),
      m_hopperSubsystem(turbolib::utils::MakeIO<HopperIO, HopperRealIO, HopperSimIO>()),
      m_gateSubsystem(turbolib::utils::MakeIO<GateIO, GateRealIO, GateSimIO>()),
      m_servoSubsystem(turbolib::utils::MakeIO<ServoIO, ServoRealIO, ServoSimIO>()) {
  ConfigurePlannerCommands();
  ConfigureBindings();
  ConfigureIntakeBindings();
  ConfigureShooterBindings();

  m_autoChooser = pathplanner::AutoBuilder::buildAutoChooser();

  frc::SmartDashboard::PutData("Auto Chooser", &m_autoChooser);
}

void RobotContainer::ConfigureBindings() {
  m_operatorController.POVUp().WhileTrue(
      CommandFactory::OuttakeCommand(m_intakeSubsystem, m_hopperSubsystem, m_gateSubsystem));
  m_operatorController.B().ToggleOnTrue(CommandFactory::PassThroughCommand(m_hopperSubsystem, m_gateSubsystem));

  m_driverController.A().ToggleOnTrue(m_driveSubsystem.RotateToHub());
  m_operatorController.A().WhileTrue(m_intakeSubsystem.SlowZeroCommand());

  m_driveSubsystem.SetDefaultCommand(
      m_driveSubsystem
          .ApplyRequest([this]() -> auto&& {
            return m_drive.WithVelocityX(-m_driverController.GetLeftY() * DriveConstants::kMaxSpeed)
                .WithVelocityY(-m_driverController.GetLeftX() * DriveConstants::kMaxSpeed)
                .WithRotationalRate(-m_driverController.GetRightX() * DriveConstants::kMaxAngularRate);
          })
          .WithName("Default Drive"));

  m_driverController.LeftBumper().OnTrue(frc2::cmd::RunOnce([this] { m_driveSubsystem.SeedFieldCentric(); }));

  frc2::RobotModeTriggers::Disabled().WhileTrue(
      m_driveSubsystem.ApplyRequest([] { return swerve::requests::Idle{}; }).IgnoringDisable(true));

  m_driveSubsystem.RegisterTelemetry([this](auto const& state) { logger.Telemeterize(state); });
}

void RobotContainer::ConfigureIntakeBindings() {
  m_operatorController.RightBumper().ToggleOnTrue(m_intakeSubsystem.RunIntakeCommand());
}

void RobotContainer::ConfigureShooterBindings() {
  m_operatorController.POVDown().WhileTrue(
      CommandFactory::LayupShotCommand(m_shooterSubsystem, m_servoSubsystem, m_hopperSubsystem, m_gateSubsystem));
  m_operatorController.POVLeft().WhileTrue(
      CommandFactory::RegressionShotCommand(m_shooterSubsystem, m_servoSubsystem, m_hopperSubsystem, m_gateSubsystem,
                                            [this] { return m_driveSubsystem.GetDistanceToHub(); }));
  m_operatorController.X().WhileTrue(
      CommandFactory::TrenchShotCommand(m_shooterSubsystem, m_servoSubsystem, m_hopperSubsystem, m_gateSubsystem));
  m_operatorController.POVRight().WhileTrue(
      CommandFactory::LaserShotCommand(m_shooterSubsystem, m_servoSubsystem, m_hopperSubsystem, m_gateSubsystem));
  m_operatorController.Y().WhileTrue(
      CommandFactory::ClimbShotCommand(m_shooterSubsystem, m_servoSubsystem, m_hopperSubsystem, m_gateSubsystem));
}

void RobotContainer::ConfigurePlannerCommands() {
  pathplanner::EventTrigger("IntakeDown").OnTrue(m_intakeSubsystem.RunIntakeCommand());
  pathplanner::EventTrigger("IntakeStop").OnTrue(m_intakeSubsystem.StopIntakeCommand());

  pathplanner::NamedCommands::registerCommand("AutoAlign", m_driveSubsystem.RotateToHub());
  pathplanner::NamedCommands::registerCommand("Shoot", m_shooterSubsystem.RunClimbCommand());
  pathplanner::NamedCommands::registerCommand("RegressionShoot", m_shooterSubsystem.RunRegressionCommand([this] {
    return m_driveSubsystem.GetDistanceToHub();
  }));
  pathplanner::NamedCommands::registerCommand("DisableVision",
                                              frc2::cmd::RunOnce([this] { m_perception.DisableVision(); }));
  pathplanner::NamedCommands::registerCommand("EnableVision",
                                              frc2::cmd::RunOnce([this] { m_perception.EnableVision(); }));
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  return m_autoChooser.GetSelected();
}
