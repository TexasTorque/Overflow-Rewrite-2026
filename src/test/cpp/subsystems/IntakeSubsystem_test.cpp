#include <gtest/gtest.h>

#include <memory>

#include "constants/constants.hpp"
#include "subsystems/IntakeSubsystem.hpp"

namespace {

class FakeIntakeIO : public IntakeIO {
 public:
  void UpdateInputs(IntakeIOInputs& inputs) override { inputs = lastInputs; }

  void SetIntakeVoltage(units::volt_t voltage) override { lastVoltage = voltage; }

  void SetIntakePivotSetpoint(double setpoint, bool slow = false) override { lastPivotSetpoint = setpoint; }

  units::volt_t lastVoltage{0_V};
  double lastPivotSetpoint{0.0};
  IntakeIOInputs lastInputs{};
};

}  // namespace

TEST(IntakeSubsystemStateTest, StartsInStowState) {
  auto io = std::make_unique<FakeIntakeIO>();
  auto* ioPtr = io.get();

  IntakeSubsystem subsystem(std::move(io));

  EXPECT_EQ(subsystem.GetState(), IntakeStateEnum::Stow);
  EXPECT_EQ(ioPtr->lastVoltage, 0_V);
  EXPECT_DOUBLE_EQ(ioPtr->lastPivotSetpoint, 0.0);
}

TEST(IntakeSubsystemStateTest, PivotDownOnStowState) {
  auto io = std::make_unique<FakeIntakeIO>();
  auto* ioPtr = io.get();

  IntakeSubsystem subsystem(std::move(io));
  subsystem.SetState(subsystem.GetState());

  EXPECT_EQ(subsystem.GetState(), IntakeStateEnum::Stow);
  EXPECT_EQ(ioPtr->lastVoltage, 0_V);
  EXPECT_DOUBLE_EQ(ioPtr->lastPivotSetpoint, IntakeConstants::kRotaryDownPosition);
}

TEST(IntakeSubsystemStateTest, SetStateIntakeAppliesIntakeOutputs) {
  auto io = std::make_unique<FakeIntakeIO>();
  auto* ioPtr = io.get();

  IntakeSubsystem subsystem(std::move(io));

  subsystem.SetState(IntakeStateEnum::Intake);

  EXPECT_EQ(subsystem.GetState(), IntakeStateEnum::Intake);
  EXPECT_EQ(ioPtr->lastVoltage, IntakeConstants::kIntakeVoltage);
  EXPECT_DOUBLE_EQ(ioPtr->lastPivotSetpoint, IntakeConstants::kRotaryDownPosition);
}

TEST(IntakeSubsystemStateTest, CleanReturnsToStowOutputs) {
  auto io = std::make_unique<FakeIntakeIO>();
  auto* ioPtr = io.get();

  IntakeSubsystem subsystem(std::move(io));

  subsystem.SetState(IntakeStateEnum::Intake);
  subsystem.Clean();

  EXPECT_EQ(subsystem.GetState(), IntakeStateEnum::Stow);
  EXPECT_EQ(ioPtr->lastVoltage, 0_V);
  EXPECT_DOUBLE_EQ(ioPtr->lastPivotSetpoint, IntakeConstants::kRotaryDownPosition);
}