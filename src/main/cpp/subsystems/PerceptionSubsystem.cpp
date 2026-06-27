// Texas Torque 1477
// Overflow 2026

#include "subsystems/PerceptionSubsystem.hpp"
#include <algorithm>
#include <memory>
#include <vector>
#include "constants/Constants.hpp"
#include "frc/apriltag/AprilTagFields.h"
#include "turbolib/perception/TurboPhotonCamera.hpp"

PerceptionSubsystem::PerceptionSubsystem(VisionMeasurementConsumer& visionConsumer) : m_visionConsumer(visionConsumer) {
  AddLocalizationCamera("shooterRightCam", PerceptionConstants::kShooterRightCamTransform,
                        frc::AprilTagField::k2026RebuiltAndyMark);
  AddLocalizationCamera("hopperLeftCam", PerceptionConstants::kHopperLeftCamTransform,
                        frc::AprilTagField::k2026RebuiltAndyMark);
  AddLocalizationCamera("hopperRightCam", PerceptionConstants::kHopperRightCamTransform,
                        frc::AprilTagField::k2026RebuiltAndyMark);

  m_seesTagPublisher = nt::NetworkTableInstance::GetDefault().GetBooleanTopic("PerceptionSubsystem/seesTag").Publish();
}

void PerceptionSubsystem::Update() {
  if (m_localizationCameras.empty() || !m_isEnabled) {
    return;
  }

  for (auto& camera : m_localizationCameras) {
    const std::vector<turbolib::structure::PoseTimestampPair> visionPoses = camera->FetchPose();

    for (const auto& pair : visionPoses) {
      m_visionConsumer.AddVisionMeasurement(pair.getPose(), pair.getLatency());
    }
  }
}

void PerceptionSubsystem::Log() {
  m_seesTagPublisher.Set(m_seesTag);
}

void PerceptionSubsystem::Periodic() {
  Update();

  m_seesTag = std::any_of(m_localizationCameras.begin(), m_localizationCameras.end(), [](const std::unique_ptr<turbolib::perception::TurboPhotonCamera>& camera) {
    return camera->SeesTag();
  });

  Log();
}
