#include "subsystems/LEDSubsystem.hpp"
#include "constants/Constants.hpp"

LEDSubsystem::LEDSubsystem() {
  m_leds.SetLength(m_ledData.max_size());
  m_leds.SetData(m_ledData);
  m_leds.Start();
}

void LEDSubsystem::SetLEDPattern(const frc::LEDPattern& pattern) {
  m_pattern = pattern;
}

void LEDSubsystem::UpdateBuffer() {
  m_pattern.ApplyTo(m_ledData);
  m_leds.SetData(m_ledData);
}

void LEDSubsystem::Periodic() {
  UpdateBuffer();
}

frc2::CommandPtr LEDSubsystem::ShowIdleCommand() {
  return StartRun([this] { SetLEDPattern(LEDConstants::kIdle); }, [] {}).WithName("LED Idle");
}

frc2::CommandPtr LEDSubsystem::ShowRunningCommand() {
  return StartRun([this] { SetLEDPattern(LEDConstants::kRunning); }, [] {}).WithName("LED Running");
}
