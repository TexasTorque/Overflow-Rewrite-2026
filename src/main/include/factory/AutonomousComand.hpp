#pragma once

#include <optional>
#include "choreo/trajectory/SwerveSample.h"
#include "choreo/trajectory/Trajectory.h"
#include "frc/DriverStation.h"
#include "frc/Timer.h"
#include "frc2/command/Command.h"
#include "frc2/command/CommandHelper.h"
#include "subsystems/CommandSwerveDrivetrain.h"

class AutonomousCommand : public frc2::CommandHelper<frc2::Command, AutonomousCommand> {
 public:
  AutonomousCommand(subsystems::CommandSwerveDrivetrain& drive,
                    std::optional<choreo::Trajectory<choreo::SwerveSample>>& traj, frc::Timer& timer)
      : m_drivebase(drive), m_autoTimer(timer), m_trajectory(traj) {
    AddRequirements(&m_drivebase);
  };

  void Initialize() override {
    if (auto initialPose = m_trajectory.value().GetInitialPose(IsRedAlliance())) {
      m_drivebase.ResetPose(initialPose.value());
    }

    m_autoTimer.Reset();
    m_autoTimer.Start();
  }

  void Execute() override {
    if (auto sample = m_trajectory.value().SampleAt(m_autoTimer.Get(), IsRedAlliance())) {
      m_drivebase.FollowTrajectory(sample.value());
    }
  }

  void End(bool) override {}
  bool IsFinished() override { return false; }

 private:
  subsystems::CommandSwerveDrivetrain& m_drivebase;
  frc::Timer& m_autoTimer;
  std::optional<choreo::Trajectory<choreo::SwerveSample>>& m_trajectory;

  bool IsRedAlliance() {
    return frc::DriverStation::GetAlliance().value_or(frc::DriverStation::kBlue) == frc::DriverStation::kRed;
  }
};