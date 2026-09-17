// Texas Torque 1477
// Overflow 2026

#include "subsystems/ServoSubsystem.hpp"
#include <functional>
#include "abstractions/io/servo/ServoIO.hpp"
#include "abstractions/logging/ServoLogging.hpp"
#include "abstractions/state/ServoState.hpp"
#include "constants/Constants.hpp"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/Commands.h"

ServoSubsystem::ServoSubsystem(std::unique_ptr<ServoIO> io)
    : m_io(std::move(io)),
      m_state(ServoStateEnum::Laser, [this](const ServoStateEnum& newState) { ApplyState(newState); }),
      m_logging{} {
  SetName("ServoSubsystem");
}

void ServoSubsystem::Periodic() {
  m_io->UpdateInputs(m_inputs);
  m_logging.UpdateTelemetry(m_inputs, GetState());
}

frc2::CommandPtr ServoSubsystem::SetServoUpCommand() {
  return frc2::cmd::StartEnd([this] { SetState(ServoStateEnum::Idle); }, [this] { SetState(ServoStateEnum::Laser); },
                             {this})
      .WithName("Servo Layup");
}

frc2::CommandPtr ServoSubsystem::SetServoLaserCommand() {
  return frc2::cmd::StartEnd([this] { SetState(ServoStateEnum::Laser); }, [this] { SetState(ServoStateEnum::Laser); },
                             {this})
      .WithName("Servo Laser");
}

void ServoSubsystem::SetState(const ServoStateEnum& newState) {
  m_state.Set(newState);
  // m_state.Apply();
}

void ServoSubsystem::Clean() {
  SetState(ServoStateEnum::Laser);
}

void ServoSubsystem::ApplyState(const ServoStateEnum& newState) {
  switch (newState) {
    case ServoStateEnum::Idle:
      m_io->SetServoPos(ServoConstants::kServoIdlePos);
      break;
    case ServoStateEnum::Laser:
      m_io->SetServoPos(ServoConstants::kServoLaserPos);
      break;
  }
}
