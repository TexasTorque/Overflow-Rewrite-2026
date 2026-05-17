// Texas Torque 1477
// Overflow 2026

#pragma once
#include <frc/geometry/Pose2d.h>
#include <units/time.h>

class VisionMeasurementConsumer {
 public:
  virtual ~VisionMeasurementConsumer() = default;
  virtual void AddVisionMeasurement(frc::Pose2d visionRobotPose, units::second_t timestamp) = 0;
};
