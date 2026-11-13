// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "subsystems/linearslide/LinearSlide.h"

#include <cmath>

#include <frc/Timer.h>

#include "logging/LoggedTracer.h"
#include "subsystems/linearslide/LinearSlideConstants.h"
#include "utils/Util.h"

LinearSlide::LinearSlide(bool enabled, std::unique_ptr<LinearSlideIO> io)
    : DisableSubsystem(enabled, "LinearSlide"), m_io{std::move(io)} {
  m_io->Zero();
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
  m_io->UpdateInputs(m_inputs);
  m_inputsLog.Publish();
  m_reqPositionLog.Set(m_reqPosition);
  logging::LoggedTracer::Record("LinearSlide");
}

void LinearSlide::Drive(double position) {
  m_reqPosition = position;
  if (IsExtended()) {
    m_io->SetExtendedPosition(position);
  } else {
    m_io->SetPosition(position);
  }
}

void LinearSlide::DriveNonExtended(double position) {
  m_reqPosition = position;
  m_io->SetPosition(position);
}

void LinearSlide::SetPosition(double position) {
  if (Request()) {
    Drive(position);
  }
}

void LinearSlide::SetNonExtendedPosition(double position) {
  if (Request()) {
    DriveNonExtended(position);
  }
}

void LinearSlide::SetVoltage(double voltage) {
  if (Request()) {
    m_io->SetVoltage(voltage);
  }
}

void LinearSlide::Off() {
  if (Request() && GoalChanged()) {
    m_io->Off();
  }
}

void LinearSlide::GoToStow() { SetPosition(linear_slide::kStowPosition); }

void LinearSlide::GoToGroundIntake() { SetPosition(linear_slide::kIntakeFinalPosition); }

void LinearSlide::GoToJitter() { SetNonExtendedPosition(linear_slide::kJitterPosition); }

void LinearSlide::GoToJitterIntermediate() {
  SetNonExtendedPosition(linear_slide::kJitterIntermediate);
}

void LinearSlide::GoToIntakeSequence() {
  if (!Request()) {
    return;
  }
  const double now = frc::Timer::GetFPGATimestamp().value();
  if (GoalChanged()) {
    m_sequenceStart = now;
    m_sequenceFinalLeg = false;
  }
  if (!m_sequenceFinalLeg &&
      (now - m_sequenceStart >= 2.0 ||
       (m_reqPosition == linear_slide::kIntakeInitialPosition && ReachedPosition(0.0005)))) {
    m_sequenceFinalLeg = true;
  }
  Drive(m_sequenceFinalLeg ? linear_slide::kIntakeFinalPosition
                           : linear_slide::kIntakeInitialPosition);
}

void LinearSlide::JitterIntake() {
  if (!Request()) {
    return;
  }
  const double now = frc::Timer::GetFPGATimestamp().value();
  if (GoalChanged()) {
    m_sequenceStart = now;
  }
  const bool firstHalf = std::fmod(now - m_sequenceStart, 0.4) < 0.2;
  DriveNonExtended(firstHalf ? linear_slide::kJitterPosition : linear_slide::kJitterIntermediate);
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
