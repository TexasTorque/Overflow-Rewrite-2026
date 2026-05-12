// Texas Torque 1477
// Overflow 2026

#include "generated/TunerConstants.h"
#include "subsystems/CommandSwerveDrivetrain.h"

subsystems::CommandSwerveDrivetrain TunerConstants::CreateDrivetrain() {
  return {DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight};
}
