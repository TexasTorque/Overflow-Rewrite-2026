// Texas Torque 1477
// Overflow 2026

#include "subsystems/PerceptionSubsystem.hpp"
#include <algorithm>
#include <memory>
#include <vector>
#include "constants/Constants.hpp"
#include "frc/DriverStation.h"
#include "frc/apriltag/AprilTagFields.h"
#include "turbolib/perception/TurboPhotonCamera.hpp"

PerceptionSubsystem::PerceptionSubsystem(VisionMeasurementConsumer& visionConsumer) : m_visionConsumer(visionConsumer) {
  AddLocalizationCamera("shooterRight", PerceptionConstants::kShooterRightCamTransform,
                        frc::AprilTagField::k2026RebuiltAndyMark, true);
  AddLocalizationCamera("hopperLeft", PerceptionConstants::kHopperLeftCamTransform,
                        frc::AprilTagField::k2026RebuiltAndyMark, true);
  AddLocalizationCamera("hopperRight", PerceptionConstants::kHopperRightCamTransform,
                        frc::AprilTagField::k2026RebuiltAndyMark, true);
  AddLocalizationCamera("shooterLeft", PerceptionConstants::kShooterLeftCamTransform,
                        frc::AprilTagField::k2026RebuiltAndyMark, true);

  m_seesTagPublisher = nt::NetworkTableInstance::GetDefault().GetBooleanTopic("PerceptionSubsystem/seesTag").Publish();
}

void PerceptionSubsystem::UpdateHeading(frc::Rotation2d gyroAngle) {
  for (auto& camera : m_localizationCameras) {
    camera->UpdateHeading(gyroAngle);
  }
}

void PerceptionSubsystem::UpdateSim(frc::Pose2d robotPose) {
  for (auto& camera : m_localizationCameras) {
    camera->UpdateSim(robotPose);
  }
}

void PerceptionSubsystem::Update() {
  if (m_localizationCameras.empty() || !m_isEnabled) {
    return;
  }

  for (auto& camera : m_localizationCameras) {
    const std::vector<turbolib::structure::PoseTimestampPair> visionPoses = camera->FetchPose();

    for (const auto& pair : visionPoses) {
      m_visionConsumer.AddVisionMeasurement(pair.getPose(), pair.getLatency(), pair.getStdDevs());
    }
  }
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

bool PerceptionSubsystem::IsHubTag(int id, bool redAlliance) {
  const auto* hubTags =
      redAlliance ? PerceptionConstants::kRedHubTags.data() : PerceptionConstants::kBlueHubTags.data();
  const size_t count = redAlliance ? PerceptionConstants::kRedHubTags.size() : PerceptionConstants::kBlueHubTags.size();

  for (size_t i = 0; i < count; i++) {
    if (hubTags[i] == id)
      return true;
  }

  return false;
}

std::optional<PerceptionSubsystem::HubTarget> PerceptionSubsystem::GetNearestHubTarget() const {
  const auto alliance = frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kBlue);
  const bool redAlliance = alliance == frc::DriverStation::Alliance::kRed;

  const turbolib::perception::TurboPhotonCamera* nearestCamera = nullptr;

  const turbolib::perception::TurboPhotonCamera::CachedTarget* nearestTarget = nullptr;
  units::meter_t nearestDistance{0_m};
  units::second_t nearestAge{0_s};

  for (const auto& camera : m_localizationCameras) {
    const auto& name = camera->GetCameraName();
    if (name != "shooterLeft" && name != "shooterRight")
      continue;

    const auto age = camera->GetCachedTargetsAge();
    if (!age.has_value() || age.value() > PerceptionConstants::kMaxTargetAge)
      continue;

    for (const auto& target : camera->GetCachedTargets()) {
      if (nearestTarget != nullptr && target.distance >= nearestDistance)
        continue;

      if (!IsHubTag(target.id, redAlliance))
        continue;

      nearestCamera = camera.get();
      nearestTarget = &target;
      nearestDistance = target.distance;
      nearestAge = age.value();
    }
  }

  if (nearestTarget == nullptr)
    return std::nullopt;

  auto tagPose = nearestCamera->GetTagFieldPose(nearestTarget->id);
  if (!tagPose.has_value())
    return std::nullopt;

  return HubTarget{nearestTarget->id,        nearestTarget->yaw, nearestDistance,
                   nearestTarget->ambiguity, tagPose.value(),    nearestAge};
}
