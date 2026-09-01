#include "subsystems/LEDSubsystem.hpp"
#include "constants/Constants.hpp"
#include "frc2/command/CommandPtr.h"

LEDSubsystem::LEDSubsystem() {
  m_leds.SetLength(m_ledData.max_size());
  m_leds.SetData(m_ledData);
  m_leds.Start();
}

void LEDSubsystem::SetLEDPattern(const frc::LEDPattern& pattern) {
  m_frontPattern = pattern;
  m_backPattern = pattern;
}

void LEDSubsystem::SetLEDPatterns(const frc::LEDPattern& frontPattern, const frc::LEDPattern& backPattern) {
  m_frontPattern = frontPattern;
  m_backPattern = backPattern;
}

void LEDSubsystem::UpdateBuffer() {
  auto ledSpan = std::span{m_ledData};
  m_frontPattern.ApplyTo(ledSpan.subspan(0, LEDConstants::kFrontLength));
  m_backPattern.ApplyTo(ledSpan.subspan(LEDConstants::kFrontLength, LEDConstants::kBackLength));
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

frc2::CommandPtr LEDSubsystem::ShowIntakingCommand() {
  return StartRun([this] { SetLEDPatterns(LEDConstants::kIntakeFront, LEDConstants::kIntakeBack); }, [] {})
      .WithName("LED Intaking");
}

frc2::CommandPtr LEDSubsystem::ShowSpinUpCommand() {
  return StartRun([this] { SetLEDPattern(LEDConstants::kSpinUp); }, [] {}).WithName("LED Spin Up");
}

frc2::CommandPtr LEDSubsystem::ShowShootingCommand() {
  return StartRun([this] { SetLEDPattern(LEDConstants::kShooting); }, [] {}).WithName("LED Shooting");
}

frc2::CommandPtr LEDSubsystem::ShowAutoAlignCommand(std::function<AutoAlignState()>&& stateFunc) {
  return StartRun(
             [this, stateFunc] {
               SetLEDPattern(stateFunc() == AutoAlignState::Right ? LEDConstants::kAutoAlignRight
                                                                  : LEDConstants::kAutoAlignLeft);
             },
             [] {})
      .WithName("LED Auto Align");
}

frc2::CommandPtr LEDSubsystem::ShowStallingCommand() {
  return StartRun([this] { SetLEDPatterns(LEDConstants::kStallFront, LEDConstants::kStallBack); }, [] {})
      .WithName("LED Stalling");
}
