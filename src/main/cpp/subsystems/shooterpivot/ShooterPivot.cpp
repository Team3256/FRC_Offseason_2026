// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "subsystems/shooterpivot/ShooterPivot.h"

#include "logging/LoggedTracer.h"
#include "subsystems/shooterpivot/ShooterPivotConstants.h"
#include "utils/Util.h"

ShooterPivot::ShooterPivot(bool enabled, std::unique_ptr<ShooterPivotIO> io)
    : DisableSubsystem(enabled, "ShooterPivot"), m_io{std::move(io)} {
  m_io->ResetPosition(0);
  m_inputsLog.Add("shooterPivotMotorVoltage", &m_inputs.shooterPivotMotorVoltage);
  m_inputsLog.Add("shooterPivotMotorVelocity", &m_inputs.shooterPivotMotorVelocity);
  m_inputsLog.Add("shooterPivotMotorPosition", &m_inputs.shooterPivotMotorPosition);
  m_inputsLog.Add("shooterPivotMotorStatorCurrent", &m_inputs.shooterPivotMotorStatorCurrent);
  m_inputsLog.Add("shooterPivotMotorSupplyCurrent", &m_inputs.shooterPivotMotorSupplyCurrent);
}

void ShooterPivot::Periodic() {
  m_io->UpdateInputs(m_inputs);
  m_inputsLog.Publish();
  m_reqPositionLog.Set(m_reqPosition);
  m_hoodVel = m_hoodAngleFilter.Calculate((m_reqPosition - m_lastHoodAngle) / 0.02);
  m_lastHoodAngle = m_reqPosition;
  m_hoodVelLog.Set(m_hoodVel);
  logging::LoggedTracer::Record("ShooterPivot");
}

void ShooterPivot::SetPosition(double position) {
  if (Request()) {
    m_reqPosition = m_trenchOverride ? 0.0 : position;
    m_io->SetPosition(m_reqPosition, m_hoodVel);
  }
}

void ShooterPivot::SetVoltage(double voltage) {
  if (Request()) {
    m_io->SetVoltage(voltage);
  }
}

void ShooterPivot::Off() {
  if (Request() && GoalChanged()) {
    m_io->Off();
  }
}

void ShooterPivot::ShootHub(double distance) { SetPosition(shooter_pivot::kHubLUT.Get(distance)); }

void ShooterPivot::FeedCorner(double distance) {
  SetPosition(shooter_pivot::kFeedLUT.Get(distance));
}

bool ShooterPivot::ReachedPosition() const {
  return util::EpsilonEquals(m_inputs.shooterPivotMotorPosition, m_reqPosition, 0.01);
}
