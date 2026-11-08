// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "subsystems/linearslide/LinearSlide.h"

#include <cstdio>

#include "subsystems/linearslide/LinearSlideConstants.h"
#include "utils/Util.h"

LinearSlide::LinearSlide(bool enabled, std::unique_ptr<LinearSlideIO> io)
    : DisableSubsystem(enabled, "LinearSlide"), m_io{std::move(io)} {
  m_io->Zero();
  const auto* current = GetCurrentCommand();
  std::printf("Current command: %s\n", current != nullptr ? current->GetName().c_str() : "none");
  m_inputsLog.Add("rightMotorVoltage", &m_inputs.rightMotorVoltage);
  m_inputsLog.Add("rightMotorVelocity", &m_inputs.rightMotorVelocity);
  m_inputsLog.Add("rightMotorPosition", &m_inputs.rightMotorPosition);
  m_inputsLog.Add("rightMotorStatorCurrent", &m_inputs.rightMotorStatorCurrent);
  m_inputsLog.Add("rightMotorSupplyCurrent", &m_inputs.rightMotorSupplyCurrent);
  m_inputsLog.Add("leftMotorVoltage", &m_inputs.leftMotorVoltage);
  m_inputsLog.Add("leftMotorVelocity", &m_inputs.leftMotorVelocity);
  m_inputsLog.Add("leftMotorPosition", &m_inputs.leftMotorPosition);
  m_inputsLog.Add("leftMotorStatorCurrent", &m_inputs.leftMotorStatorCurrent);
  m_inputsLog.Add("leftMotorSupplyCurrent", &m_inputs.leftMotorSupplyCurrent);
}

void LinearSlide::Periodic() {
  DisableSubsystem::Periodic();
  m_io->UpdateInputs(m_inputs);
  m_inputsLog.Publish();
  m_reqPositionLog.Set(m_reqPosition);
}

frc2::CommandPtr LinearSlide::SetPosition(double position) {
  return Run([this, position] {
    m_reqPosition = position;
    if (IsExtended()) {
      m_io->SetExtendedPosition(position);
    } else {
      m_io->SetPosition(position);
    }
  });
}

frc2::CommandPtr LinearSlide::SetNonExtendedPosition(double position) {
  return Run([this, position] {
    m_reqPosition = position;
    m_io->SetPosition(position);
  });
}

frc2::CommandPtr LinearSlide::SetPosition(std::function<double()> position) {
  return Run([this, position = std::move(position)] {
    m_reqPosition = position();
    if (IsExtended()) {
      m_io->SetExtendedPosition(m_reqPosition);
    } else {
      m_io->SetPosition(m_reqPosition);
    }
  });
}

frc2::CommandPtr LinearSlide::SetVoltage(double voltage) {
  return Run([this, voltage] { m_io->SetVoltage(voltage); });
}

frc2::CommandPtr LinearSlide::Off() {
  return RunOnce([this] { m_io->Off(); }).WithName("off");
}

frc2::CommandPtr LinearSlide::GoToStow() { return SetPosition(linear_slide::kStowPosition); }

frc2::CommandPtr LinearSlide::GoToGroundIntake() {
  return SetPosition(linear_slide::kIntakeFinalPosition);
}

frc2::CommandPtr LinearSlide::GoToIntakeSequence() {
  return SetPosition(linear_slide::kIntakeInitialPosition)
      .Until([this] { return ReachedPosition(0.0005); })
      .WithTimeout(2.0_s)
      .AndThen(SetPosition(linear_slide::kIntakeFinalPosition));
}

frc2::CommandPtr LinearSlide::GoToJitter() {
  return SetNonExtendedPosition(linear_slide::kJitterPosition);
}

frc2::CommandPtr LinearSlide::GoToJitterIntermediate() {
  return SetNonExtendedPosition(linear_slide::kJitterIntermediate);
}

frc2::CommandPtr LinearSlide::JitterIntake() {
  return GoToJitter()
      .WithTimeout(.2_s)
      .AndThen(GoToJitterIntermediate().WithTimeout(.2_s))
      .Repeatedly();
}

bool LinearSlide::ReachedPosition(double tolerance) const {
  return util::EpsilonEquals(m_inputs.rightMotorPosition, m_reqPosition, tolerance) &&
         util::EpsilonEquals(m_inputs.leftMotorPosition, m_reqPosition, tolerance);
}

bool LinearSlide::ReachedPosition() const { return ReachedPosition(0.01); }

bool LinearSlide::AtPosition() const {
  const double right = m_inputs.rightMotorPosition;
  const double left = m_inputs.leftMotorPosition;
  return (right >= 0.0 && right <= linear_slide::kBetweenPosMax) &&
         (left >= 0.0 && left <= linear_slide::kBetweenPosMax);
}

bool LinearSlide::IsExtended() const {
  return util::EpsilonEquals(m_inputs.rightMotorPosition, linear_slide::kIntakeFinalPosition, 0.01);
}
