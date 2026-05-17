// Texas Torque 1477
// Overflow 2026

#include "subsystems/PerceptionSubsystem.hpp"
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
}

void PerceptionSubsystem::Update() {
  if (m_localizationCameras.empty()) {
    return;
  }

  for (auto& camera : m_localizationCameras) {
    const std::vector<turbolib::structure::PoseTimestampPair> visionPoses = camera->FetchPose();

    for (const auto& pair : visionPoses) {
      m_visionConsumer.AddVisionMeasurement(pair.getPose(), pair.getLatency());
    }
  }
}

void PerceptionSubsystem::Periodic() {
  Update();
}
