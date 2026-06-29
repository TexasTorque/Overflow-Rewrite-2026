#pragma once

#include <array>
#include "frc/AddressableLED.h"
#include "frc/LEDPattern.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/SubsystemBase.h"
#include "constants/Constants.hpp"

class LEDSubsystem : public frc2::SubsystemBase {
 public:
  LEDSubsystem();

  void SetLEDPattern(const frc::LEDPattern& pattern);
  void SetLEDPatterns(const frc::LEDPattern& frontPattern, const frc::LEDPattern& backPattern);

  void UpdateBuffer();

  frc2::CommandPtr ShowIdleCommand();
  frc2::CommandPtr ShowRunningCommand();
  frc2::CommandPtr ShowIntakingCommand();
  frc2::CommandPtr ShowSpinUpCommand();
  frc2::CommandPtr ShowShootingCommand();

  void Periodic() override;

 private:
  frc::AddressableLED m_leds{LEDConstants::kLEDPort};
  std::array<frc::AddressableLED::LEDData, LEDConstants::kLEDLength> m_ledData;
  frc::LEDPattern m_frontPattern{LEDConstants::kIdle};
  frc::LEDPattern m_backPattern{LEDConstants::kIdle};
};
