#pragma once

#include <algorithm>
#include <cstddef>
#include <functional>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include "choreo/trajectory/SwerveSample.h"
#include "choreo/trajectory/Trajectory.h"
#include "frc/DriverStation.h"
#include "frc/Timer.h"
#include "frc2/command/Command.h"
#include "frc2/command/CommandHelper.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/CommandScheduler.h"
#include "subsystems/CommandSwerveDrivetrain.h"
#include "units/time.h"

class AutonomousCommand : public frc2::CommandHelper<frc2::Command, AutonomousCommand> {
 public:
  using EventCallback = std::function<frc2::CommandPtr()>;

  AutonomousCommand(subsystems::CommandSwerveDrivetrain& drive,
                    std::optional<choreo::Trajectory<choreo::SwerveSample>> traj, frc::Timer& timer,
                    std::unordered_map<std::string, EventCallback> eventMap = {})
      : m_drivebase(drive), m_autoTimer(timer), m_trajectory(std::move(traj)), m_eventMap(std::move(eventMap)) {
    AddRequirements(&m_drivebase);
  };

  void Initialize() override {
    if (!m_trajectory.has_value()) {
      return;
    }

    if (auto initialPose = m_trajectory.value().GetInitialPose(IsRedAlliance())) {
      m_drivebase.ResetPose(initialPose.value());
    }

    m_timedEvents.clear();
    for (const auto& marker : m_trajectory.value().events) {
      auto it = m_eventMap.find(marker.event);

      if (it != m_eventMap.end()) {
        m_timedEvents.emplace_back(marker.timestamp, it->second);
      }
    }
    std::sort(m_timedEvents.begin(), m_timedEvents.end(),
              [](const TimedEvent& a, const TimedEvent& b) { return a.timestamp < b.timestamp; });
    m_nextEventIdx = 0;

    m_autoTimer.Reset();
    m_autoTimer.Start();
  }

  void Execute() override {
    if (!m_trajectory.has_value()) {
      return;
    }

    if (auto sample = m_trajectory.value().SampleAt(m_autoTimer.Get(), IsRedAlliance())) {
      m_drivebase.FollowTrajectory(sample.value());
    }

    double currentTime = m_autoTimer.Get().value();
    while (m_nextEventIdx < m_timedEvents.size() && currentTime >= m_timedEvents[m_nextEventIdx].timestamp.value()) {
      frc2::CommandScheduler::GetInstance().Schedule(m_timedEvents[m_nextEventIdx].callback());
      m_nextEventIdx++;
    }
  }

  void End(bool) override {}
  bool IsFinished() override {
    if (!m_trajectory.has_value()) {
      return true;
    }

    return m_autoTimer.Get() > m_trajectory.value().GetTotalTime();
  }

 private:
  struct TimedEvent {
    units::second_t timestamp;
    EventCallback callback;
  };

  subsystems::CommandSwerveDrivetrain& m_drivebase;
  frc::Timer& m_autoTimer;
  std::optional<choreo::Trajectory<choreo::SwerveSample>> m_trajectory;

  std::unordered_map<std::string, EventCallback> m_eventMap;
  std::vector<TimedEvent> m_timedEvents;
  size_t m_nextEventIdx = 0;

  bool IsRedAlliance() const {
    return frc::DriverStation::GetAlliance().value_or(frc::DriverStation::kBlue) == frc::DriverStation::kRed;
  }
};