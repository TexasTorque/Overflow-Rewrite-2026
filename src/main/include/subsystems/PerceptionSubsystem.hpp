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
#include "photon/PhotonCamera.h"
#include "turbolib/perception/TurboPhotonCamera.hpp"
#include "units/angle.h"
#include "units/time.h"

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

  // Simulation only: advance each camera's VisionSystemSim and feed heading
  // data to its pose estimator. No-op on real hardware.
  void UpdateSim(const frc::Pose2d& robotPose, const frc::Rotation2d& heading);

  // Raw yaw (camera-frame degrees, positive-left) to a fiducial from the
  // shooter-side tracking cameras only. Returns nullopt when the tag hasn't
  // been seen recently. Independent of m_isEnabled so tracking keeps working
  // while pose estimation is disabled.
  std::optional<units::degree_t> GetTrackingYaw(int tagId);

  void Log();
  void Periodic() override;

 private:
  VisionMeasurementConsumer& m_visionConsumer;
  std::vector<std::unique_ptr<turbolib::perception::TurboPhotonCamera>> m_localizationCameras;

  // Dedicated instances on the shooter-side topics, used only for tag yaw.
  // Each PhotonCamera owns its own NT result queue, so these never steal
  // frames from the TurboPhotonCamera localization instances.
  photon::PhotonCamera m_trackShooterLeft{"shooterLeft"};
  photon::PhotonCamera m_trackShooterRight{"shooterRight"};

  std::optional<units::degree_t> m_lastTrackYaw{std::nullopt};
  units::second_t m_lastTrackTime{-1.0};

  bool m_isEnabled = true;
  bool m_seesTag = false;

  nt::BooleanPublisher m_seesTagPublisher;
};
