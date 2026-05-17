#pragma once

#include <memory>
#include <vector>
#include "frc2/command/SubsystemBase.h"
#include "abstractions/perception/VisionMeasurementConsumer.hpp"
#include "turbolib/perception/TurboPhotonCamera.hpp"

class PerceptionSubsystem : frc2::SubsystemBase {
 public:
  explicit PerceptionSubsystem(VisionMeasurementConsumer& visionConsumer);

  void Update();
  void AddLocalizationCamera(const std::string& cameraName, const frc::Transform3d& cameraInBotSpace,
                             frc::AprilTagField field, bool enableSim = false) {
    m_localizationCameras.push_back(
        std::make_unique<turbolib::perception::TurboPhotonCamera>(cameraName, cameraInBotSpace, field, enableSim));
  }

  void Periodic() override;

 private:
  VisionMeasurementConsumer& m_visionConsumer;
  std::vector<std::unique_ptr<turbolib::perception::TurboPhotonCamera>> m_localizationCameras;
};