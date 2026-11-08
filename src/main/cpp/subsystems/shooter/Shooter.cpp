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
  DisableSubsystem::Periodic();
  m_io->UpdateInputs(m_inputs);
  m_inputsLog.Publish();
  m_reqVelocityLog.Set(m_reqVelocity);
  logging::LoggedTracer::Record("Shooter");
}

frc2::CommandPtr Shooter::SetVoltage(double voltage) {
  return Run([this, voltage] { m_io->SetShooterVoltage(voltage); }).FinallyDo([this] {
    m_io->Off();
  });
}

frc2::CommandPtr Shooter::SetVelocity(double velocity) {
  return SetVelocity([velocity] { return velocity; });
}

frc2::CommandPtr Shooter::SetVelocity(std::function<double()> velocity) {
  return Run([this, velocity = std::move(velocity)] {
           m_reqVelocity = velocity();
           m_io->SetShooterVelocity(m_reqVelocity);
         })
      .FinallyDo([this] { m_io->Off(); });
}

frc2::CommandPtr Shooter::ShootHub(std::function<double()> distance,
                                   std::function<double()> velMultiplier) {
  return SetVelocity([distance = std::move(distance), velMultiplier = std::move(velMultiplier)] {
    return shooter::kHubLUT.Get(distance()) * velMultiplier();
  });
}

frc2::CommandPtr Shooter::FeedCorner(std::function<double()> distance) {
  return SetVelocity(
      [distance = std::move(distance)] { return shooter::kFeedLUT.Get(distance()); });
}

bool Shooter::ReachedVelocity() const {
  return util::EpsilonEquals(m_inputs.shooterMotorVelocitys[0], m_reqVelocity, 5.0);
}

frc2::CommandPtr Shooter::Off() {
  return RunOnce([this] { m_io->Off(); });
}
