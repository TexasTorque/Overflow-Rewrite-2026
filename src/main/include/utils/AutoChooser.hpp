// Texas Torque 1477
// Overflow 2026

#pragma once

#include <filesystem>
#include <string>
#include <utility>
#include <vector>
#include "frc/smartdashboard/SendableChooser.h"
#include "frc2/command/Command.h"
#include "frc2/command/CommandPtr.h"
#include "pathplanner/lib/auto/AutoBuilder.h"
#include "pathplanner/lib/commands/PathPlannerAuto.h"

class AutoChooser {
 public:
  AutoChooser() {
    std::vector<std::filesystem::path> autoPathFilepaths = pathplanner::AutoBuilder::getAllAutoPaths();
    for (std::filesystem::path path : autoPathFilepaths) {
      m_autoCommands.insert_or_assign(path.stem().string(),
                                      pathplanner::PathPlannerAuto(path.replace_extension("").string()).ToPtr());
      m_autoCommands.insert_or_assign(path.stem().string() + "_mirror",
                                      pathplanner::PathPlannerAuto(path.replace_extension("").string(), true).ToPtr());
    }

    for (const std::pair<const std::string, frc2::CommandPtr>& entry : m_autoCommands) {
      m_chooser.AddOption(entry.first, entry.second.get());
    }
  }

  frc::SendableChooser<frc2::Command*>* GetChooser() { return &m_chooser; }

 private:
  frc::SendableChooser<frc2::Command*> m_chooser;
  std::map<std::string, frc2::CommandPtr> m_autoCommands;
};
