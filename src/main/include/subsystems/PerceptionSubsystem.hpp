// Texas Torque 1477
// Overflow 2026

#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>
#include "frc/RobotBase.h"
#include "frc2/command/SubsystemBase.h"
#include "abstractions/perception/VisionMeasurementConsumer.hpp"
#include "networktables/BooleanTopic.h"
#include "turbolib/perception/TurboPhotonCamera.hpp"

class PerceptionSubsystem : frc2::SubsystemBase {
 public:
  explicit PerceptionSubsystem(VisionMeasurementConsumer& visionConsumer);

  void Update();
  void AddLocalizationCamera(const std::string& cameraName, const frc::Transform3d& cameraInBotSpace,
                             frc::AprilTagField field, bool enableSim = frc::RobotBase::IsSimulation()) {
    m_localizationCameras.push_back(
        std::make_unique<turbolib::perception::TurboPhotonCamera>(cameraName, cameraInBotSpace, field, enableSim));
  }

  void DisableVision() { m_isEnabled = false; }
  void EnableVision() { m_isEnabled = true; }

  struct HubTarget {
    int id;
    units::degree_t yaw;
    units::meter_t distance;
    double ambiguity;
    frc::Pose2d tagPose;
    units::second_t age;
  };

  std::optional<HubTarget> GetNearestHubTarget() const;

  void UpdateHeading(frc::Rotation2d gyroAngle);

  void Log();
  void Periodic() override;

  void UpdateSim(frc::Pose2d robotPose);

  static bool IsHubTag(int id, bool redAlliance);

 private:
  VisionMeasurementConsumer& m_visionConsumer;
  std::vector<std::unique_ptr<turbolib::perception::TurboPhotonCamera>> m_localizationCameras;

  bool m_isEnabled = true;
  bool m_seesTag = false;

  nt::BooleanPublisher m_seesTagPublisher;
};
