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
  DisableSubsystem::Periodic();
  m_io->UpdateInputs(m_inputs);
  m_inputsLog.Publish();
  m_reqPositionLog.Set(m_reqPosition);
  m_hoodVel = m_hoodAngleFilter.Calculate((m_reqPosition - m_lastHoodAngle) / 0.02);
  m_lastHoodAngle = m_reqPosition;
  m_hoodVelLog.Set(m_hoodVel);
  logging::LoggedTracer::Record("ShooterPivot");
}

frc2::CommandPtr ShooterPivot::SetPosition(double position) {
  return SetPosition([position] { return position; });
}

frc2::CommandPtr ShooterPivot::SetPosition(std::function<double()> position) {
  return Run([this, position = std::move(position)] {
    m_reqPosition = m_trenchOverride ? 0.0 : position();
    m_io->SetPosition(m_reqPosition, m_hoodVel);
  });
}

frc2::CommandPtr ShooterPivot::SetVoltage(double voltage) {
  return Run([this, voltage] { m_io->SetVoltage(voltage); });
}

frc2::CommandPtr ShooterPivot::Off() {
  return RunOnce([this] { m_io->Off(); }).WithName("off");
}

frc2::CommandPtr ShooterPivot::ShootHub(std::function<double()> distance) {
  return SetPosition(
      [distance = std::move(distance)] { return shooter_pivot::kHubLUT.Get(distance()); });
}

frc2::CommandPtr ShooterPivot::FeedCorner(std::function<double()> distance) {
  return SetPosition(
      [distance = std::move(distance)] { return shooter_pivot::kFeedLUT.Get(distance()); });
}

bool ShooterPivot::ReachedPosition() const {
  return util::EpsilonEquals(m_inputs.shooterPivotMotorPosition, m_reqPosition, 0.01);
}
