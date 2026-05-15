#include "subsystems/PerceptionSubsystem.hpp"
#include <memory>
#include "frc/apriltag/AprilTagFields.h"
#include "frc/geometry/Rotation3d.h"
#include "frc/geometry/Transform3d.h"
#include "subsystems/CommandSwerveDrivetrain.h"
#include "telemetrykit/core/AlertManager.h"
#include "turbolib/perception/TurboPhotonCamera.hpp"

PerceptionSubsystem::PerceptionSubsystem(subsystems::CommandSwerveDrivetrain& drive) : m_drive(drive) {
  AddLocalizationCamera("shooterRightCam",
                        frc::Transform3d{-10.88111_in, -7.985419_in, 10.326243_in, {0_deg, -22.5_deg, 180_deg}},
                        frc::AprilTagField::k2026RebuiltAndyMark);
  AddLocalizationCamera("hopperLeftCam",
                        frc::Transform3d{2.50874_in, 12.29184_in, 6.912581_in, {0_deg, -25_deg, 90_deg}},
                        frc::AprilTagField::k2026RebuiltAndyMark);
  AddLocalizationCamera("hopperRightCam",
                        frc::Transform3d{2.50874_in, -12.29184_in, 6.912581_in, {0_deg, -25_deg, -90_deg}},
                        frc::AprilTagField::k2026RebuiltAndyMark);
}

void PerceptionSubsystem::Update() {
  if (m_localizationCameras.empty()) {
    auto& alertManager = tkit::AlertManager::GetInstance();

    // TODO: this causes command overruns dont use this (we are using this)
    alertManager.Warning("photon_cam_missing", "No photon cameras configured.");
    return;
  }

  for (auto& camera : m_localizationCameras) {
    const std::vector<turbolib::structure::PoseTimestampPair> visionPoses = camera->FetchPose();

    for (const auto& pair : visionPoses) {
      m_drive.AddVisionMeasurement(pair.getPose(), pair.getLatency());
    }
  }
}

void PerceptionSubsystem::Periodic() {
  Update();
}