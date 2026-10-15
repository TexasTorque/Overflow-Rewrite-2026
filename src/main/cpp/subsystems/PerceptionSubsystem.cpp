// Texas Torque 1477
// Overflow 2026

#include "subsystems/PerceptionSubsystem.hpp"
#include <algorithm>
#include <cmath>
#include <memory>
#include <vector>
#include "constants/Constants.hpp"
#include "frc/RobotBase.h"
#include "frc/Timer.h"
#include "frc/apriltag/AprilTagFields.h"
#include "turbolib/perception/TurboPhotonCamera.hpp"

PerceptionSubsystem::PerceptionSubsystem(VisionMeasurementConsumer& visionConsumer) : m_visionConsumer(visionConsumer) {
  AddLocalizationCamera("shooterRight", PerceptionConstants::kShooterRightCamTransform,
                        frc::AprilTagField::k2026RebuiltAndyMark);
  AddLocalizationCamera("hopperLeft", PerceptionConstants::kHopperLeftCamTransform,
                        frc::AprilTagField::k2026RebuiltAndyMark);
  AddLocalizationCamera("hopperRight", PerceptionConstants::kHopperRightCamTransform,
                        frc::AprilTagField::k2026RebuiltAndyMark);
  AddLocalizationCamera("shooterLeft", PerceptionConstants::kShooterLeftCamTransform,
                        frc::AprilTagField::k2026RebuiltAndyMark);

  m_seesTagPublisher = nt::NetworkTableInstance::GetDefault().GetBooleanTopic("PerceptionSubsystem/seesTag").Publish();
}

void PerceptionSubsystem::UpdateSim(const frc::Pose2d& robotPose, const frc::Rotation2d& heading) {
  if (!frc::RobotBase::IsSimulation()) {
    return;
  }

  for (auto& camera : m_localizationCameras) {
    camera->UpdateSim(robotPose);
    camera->UpdateHeading(heading);
  }
}

void PerceptionSubsystem::Update() {
  if (m_localizationCameras.empty() || !m_isEnabled) {
    return;
  }

  for (auto& camera : m_localizationCameras) {
    const auto visionPoses = camera->FetchPose();

    for (const auto& pair : visionPoses) {
      m_visionConsumer.AddVisionMeasurement(pair.getPose(), pair.getLatency(), pair.getStdDevs());
    }
  }
}

std::optional<units::degree_t> PerceptionSubsystem::GetTrackingYaw(int tagId) {
  // Drain both queues every call so results never go stale in NT, keeping
  // the freshest most-centered detection across the two shooter cameras.
  std::optional<double> freshYaw{std::nullopt};
  photon::PhotonCamera* trackCams[] = {&m_trackShooterLeft, &m_trackShooterRight};

  for (auto* cam : trackCams) {
    for (const auto& result : cam->GetAllUnreadResults()) {
      if (!result.HasTargets())
        continue;
      for (const auto& target : result.GetTargets()) {
        if (target.GetFiducialId() == tagId) {
          double yaw = target.GetYaw();
          if (!freshYaw || std::abs(yaw) < std::abs(*freshYaw)) {
            freshYaw = yaw;
          }
        }
      }
    }
  }

  if (freshYaw) {
    m_lastTrackYaw = units::degree_t{*freshYaw};
    m_lastTrackTime = frc::Timer::GetFPGATimestamp();
    return m_lastTrackYaw;
  }

  if (m_lastTrackYaw &&
      (frc::Timer::GetFPGATimestamp() - m_lastTrackTime) < TrackingConstants::kYawStaleTimeout) {
    return m_lastTrackYaw;
  }

  m_lastTrackYaw = std::nullopt;
  return std::nullopt;
}

void PerceptionSubsystem::Log() {
  m_seesTagPublisher.Set(m_seesTag);
}

void PerceptionSubsystem::Periodic() {
  Update();

  m_seesTag = std::any_of(
      m_localizationCameras.begin(), m_localizationCameras.end(),
      [](const std::unique_ptr<turbolib::perception::TurboPhotonCamera>& camera) { return camera->SeesTag(); });

  Log();
}
