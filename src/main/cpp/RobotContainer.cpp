#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/Commands.h>
#include <frc2/command/button/RobotModeTriggers.h>

#include <pathplanner/lib/auto/NamedCommands.h>
#include <pathplanner/lib/events/EventTrigger.h>

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
#include "abstractions/state/GateState.hpp"
#include "abstractions/state/IntakeState.hpp"
#include "abstractions/state/ShooterState.hpp"

#include "constants/Constants.hpp"
#include "factory/CommandFactory.hpp"
#include "frc2/command/button/Trigger.h"
#include "frc2/command/sysid/SysIdRoutine.h"
#include "subsystems/HubSubsystem.hpp"
#include "subsystems/IntakeSubsystem.hpp"
#include "turbolib/util/MakeIO.hpp"
#include "utils/AutoChooser.hpp"

#include <string>

RobotContainer::RobotContainer()
    : m_intakeSubsystem(turbolib::utils::MakeIO<IntakeIO, IntakeRealIO, IntakeSimIO>()),
      m_shooterSubsystem(turbolib::utils::MakeIO<ShooterIO, ShooterRealIO, ShooterSimIO>()),
      m_hopperSubsystem(turbolib::utils::MakeIO<HopperIO, HopperRealIO, HopperSimIO>()),
      m_gateSubsystem(turbolib::utils::MakeIO<GateIO, GateRealIO, GateSimIO>()),
      m_servoSubsystem(turbolib::utils::MakeIO<ServoIO, ServoRealIO, ServoSimIO>()),
      m_hubSubsystem(),
      m_ledSubsystem() {
  ConfigurePlannerCommands();
  ConfigureBindings();
  ConfigureIntakeBindings();
  ConfigureShooterBindings();
  ConfigureLEDBindings();
  // ConfigureSysIDBindings();

  m_autoChooser = AutoChooser{};
  frc::SmartDashboard::PutData("Auto Chooser", m_autoChooser->GetChooser());
}

void RobotContainer::ConfigurePlannerCommands() {
  pathplanner::NamedCommands::registerCommand("IntakeDown", m_intakeSubsystem.RunIntakeCommand());
  pathplanner::NamedCommands::registerCommand("IntakeStop", m_intakeSubsystem.StopIntakeCommand());
  pathplanner::NamedCommands::registerCommand("IntakePullUp", m_intakeSubsystem.SlowZeroCommand());

  pathplanner::NamedCommands::registerCommand("AutoAlign",
                                              m_driveSubsystem.RotateToHub().WithDeadline(frc2::cmd::Wait(0.7_s)));
  pathplanner::NamedCommands::registerCommand(
      "RegressionShoot",
      CommandFactory::RegressionShotCommand(m_shooterSubsystem, m_servoSubsystem, m_hopperSubsystem, m_gateSubsystem,
                                            [this] { return m_driveSubsystem.GetDistanceToHub(); }));
  pathplanner::NamedCommands::registerCommand(
      "ClimbShoot",
      CommandFactory::ClimbShotCommand(m_shooterSubsystem, m_servoSubsystem, m_hopperSubsystem, m_gateSubsystem));

  pathplanner::NamedCommands::registerCommand("DisableVision",
                                              frc2::cmd::RunOnce([this] { m_perceptionSubsystem.DisableVision(); }));
  pathplanner::NamedCommands::registerCommand("EnableVision",
                                              frc2::cmd::RunOnce([this] { m_perceptionSubsystem.EnableVision(); }));
}

void RobotContainer::ConfigureBindings() {
  m_driveSubsystem.SetDefaultCommand(
      m_driveSubsystem
          .ApplyRequest([this]() -> auto&& {
            return m_drive.WithVelocityX(-m_driverController.GetLeftY() * DriveConstants::kMaxSpeed)
                .WithVelocityY(-m_driverController.GetLeftX() * DriveConstants::kMaxSpeed)
                .WithRotationalRate(-m_driverController.GetRightX() * DriveConstants::kMaxAngularRate);
          })
          .WithName("Default Drive"));

  m_driverController.LeftBumper().OnTrue(frc2::cmd::RunOnce([this] { m_driveSubsystem.SeedFieldCentric(); }));
  m_driverController.RightTrigger().WhileTrue(m_driveSubsystem.RotateToHub());

  m_operatorController.A().WhileTrue(m_intakeSubsystem.SlowZeroCommand());
  m_operatorController.POVUp().WhileTrue(
      CommandFactory::OuttakeCommand(m_intakeSubsystem, m_hopperSubsystem, m_gateSubsystem));

  frc2::RobotModeTriggers::Disabled().WhileTrue(m_driveSubsystem.ApplyRequest([] { return swerve::requests::Idle{}; })
                                                    .IgnoringDisable(true)
                                                    .WithName("Idle Drive"));

  m_driveSubsystem.RegisterTelemetry([this](auto const& state) { logger.Telemeterize(state); });
}

void RobotContainer::ConfigureIntakeBindings() {
  m_operatorController.RightBumper().ToggleOnTrue(m_intakeSubsystem.RunIntakeCommand());
}

void RobotContainer::ConfigureShooterBindings() {
  m_operatorController.POVDown().OnTrue(
      CommandFactory::StopShotCommand(m_shooterSubsystem, m_servoSubsystem, m_hopperSubsystem, m_gateSubsystem));

  m_operatorController.LeftTrigger().ToggleOnTrue(WithShotSetup(
      CommandFactory::RegressionShotCommand(m_shooterSubsystem, m_servoSubsystem, m_hopperSubsystem, m_gateSubsystem,
                                            [this] { return m_driveSubsystem.GetDistanceToHub(); })));

  m_operatorController.X().ToggleOnTrue(WithShotSetup(
      CommandFactory::TrenchShotCommand(m_shooterSubsystem, m_servoSubsystem, m_hopperSubsystem, m_gateSubsystem)));

  m_operatorController.POVRight().ToggleOnTrue(WithShotSetup(
      CommandFactory::DebugShotCommand(m_shooterSubsystem, m_servoSubsystem, m_hopperSubsystem, m_gateSubsystem)));

  m_operatorController.Y().ToggleOnTrue(WithShotSetup(
      CommandFactory::ClimbShotCommand(m_shooterSubsystem, m_servoSubsystem, m_hopperSubsystem, m_gateSubsystem)));
}

void RobotContainer::ConfigureSysIDBindings() {
  m_driverController.A().WhileTrue(m_driveSubsystem.SysIdDynamic(frc2::sysid::kForward));
  m_driverController.B().WhileTrue(m_driveSubsystem.SysIdDynamic(frc2::sysid::kReverse));
  m_driverController.X().WhileTrue(m_driveSubsystem.SysIdQuasistatic(frc2::sysid::kForward));
  m_driverController.Y().WhileTrue(m_driveSubsystem.SysIdQuasistatic(frc2::sysid::kReverse));
}

frc2::CommandPtr RobotContainer::WithShotSetup(frc2::CommandPtr shotCommand) {
  auto wasIntaking = std::make_shared<bool>(false);

  return std::move(shotCommand)
      .AlongWith(m_driveSubsystem.BrakeInPlace())
      .BeforeStarting([this, wasIntaking] {
        *wasIntaking = (m_intakeSubsystem.GetState() == IntakeStateEnum::Intake);
        m_intakeSubsystem.SetState(IntakeStateEnum::Stow);
      })
      .FinallyDo([this, wasIntaking](bool) {
        if (*wasIntaking) {
          frc2::CommandScheduler::GetInstance().Schedule(m_intakeSubsystem.RunIntakeCommand());
        }
      });
}

void RobotContainer::ConfigureLEDBindings() {
  m_ledSubsystem.SetDefaultCommand(m_ledSubsystem.ShowRunningCommand());

  frc2::RobotModeTriggers::Disabled().WhileTrue(m_ledSubsystem.ShowIdleCommand().IgnoringDisable(true));

  frc2::Trigger([this] {
    return m_intakeSubsystem.GetState() == IntakeStateEnum::Intake;
  }).WhileTrue(m_ledSubsystem.ShowIntakingCommand());

  frc2::Trigger([this] {
    auto state = m_shooterSubsystem.GetState();
    return state != ShooterStateEnum::Off && state != ShooterStateEnum::Idle;
  }).WhileTrue(m_ledSubsystem.ShowSpinUpCommand());

  frc2::Trigger([this] {
    return m_gateSubsystem.GetState() == GateStateEnum::On;
  }).WhileTrue(m_ledSubsystem.ShowShootingCommand());
}

frc2::Command* RobotContainer::GetAutonomousCommand() {
  return m_autoChooser->GetChooser()->GetSelected();
}
