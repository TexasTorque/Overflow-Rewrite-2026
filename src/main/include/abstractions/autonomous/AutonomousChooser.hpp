#pragma once

#include <frc/smartdashboard/SendableChooser.h>
#include <filesystem>
#include <functional>
#include <string>
#include "choreo/Choreo.h"
#include "choreo/trajectory/SwerveSample.h"
#include "choreo/trajectory/Trajectory.h"
#include "filesystem"
#include "frc/smartdashboard/SmartDashboard.h"
#include <frc/Filesystem.h>

class AutonomousChooser {
 public:
  AutonomousChooser(std::function<void()> onSelectCallback = nullptr)
      : m_onSelectCallback(std::move(onSelectCallback)) {
    for (const auto& entry : std::filesystem::directory_iterator(frc::filesystem::GetDeployDirectory() + "/choreo/")) {
      if (entry.path().extension() == ".traj") {
        std::string trajName = entry.path().stem().string();
        m_chooser.AddOption(trajName, trajName);
      }
    }

    m_chooser.SetDefaultOption("None", "");

    frc::SmartDashboard::PutData("Autonomous Chooser", &m_chooser);

    m_chooser.OnChange([this](std::string selected) {
      m_selectedAuto = std::move(selected);

      if (m_onSelectCallback) {
        m_onSelectCallback();
      }
    });
  }

  std::optional<choreo::Trajectory<choreo::SwerveSample>> GetSelectedTrajectory() const {
    if (m_chooser.GetSelected().empty()) {
      return std::nullopt;
    }

    return choreo::Choreo::LoadTrajectory<choreo::SwerveSample>(m_selectedAuto);
  }

 private:
  frc::SendableChooser<std::string> m_chooser;
  std::string m_selectedAuto;

  std::function<void()> m_onSelectCallback;
};