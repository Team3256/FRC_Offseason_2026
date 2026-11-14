// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "subsystems/shooter/Shooter.h"

#include "logging/LoggedTracer.h"
#include "subsystems/shooter/ShooterConstants.h"
#include "utils/Util.h"

Shooter::Shooter(bool enabled, std::unique_ptr<ShooterIO> io)
    : DisableSubsystem(enabled, "Shooter"), m_io{std::move(io)} {
  m_inputsLog.Add("shooterMotorVoltages", m_inputs.shooterMotorVoltages);
  m_inputsLog.Add("shooterMotorVelocitys", m_inputs.shooterMotorVelocitys);
  m_inputsLog.Add("shooterMotorStatorCurrents", m_inputs.shooterMotorStatorCurrents);
  m_inputsLog.Add("shooterMotorSupplyCurrents", m_inputs.shooterMotorSupplyCurrents);
  m_inputsLog.Add("shooterMotorTemperatures", m_inputs.shooterMotorTemperatures);
  m_inputsLog.Add("shooterMotorIDs", m_inputs.shooterMotorIDs);
}

void Shooter::Periodic() {
  m_io->UpdateInputs(m_inputs);
  m_inputsLog.Publish();
  m_reqVelocityLog.Set(m_reqVelocity);
  logging::LoggedTracer::Record("Shooter");
}

void Shooter::SetVoltage(double voltage) {
  if (Request()) {
    m_reqVelocity = 0;
    m_io->SetShooterVoltage(voltage);
  }
}

void Shooter::SetVelocity(double velocity) {
  if (Request()) {
    m_reqVelocity = velocity;
    m_io->SetShooterVelocity(m_reqVelocity);
  }
}

void Shooter::ShootHub(double distance, double velMultiplier) {
  SetVelocity(shooter::kHubLUT.Get(distance) * velMultiplier);
}

void Shooter::FeedCorner(double distance) { SetVelocity(shooter::kFeedLUT.Get(distance)); }

bool Shooter::ReachedVelocity() const {
  return util::EpsilonEquals(m_inputs.shooterMotorVelocitys[0], m_reqVelocity, 5.0);
}

void Shooter::Off() {
  if (Request() && GoalChanged()) {
    m_reqVelocity = 0;
    m_io->Off();
  }
}
