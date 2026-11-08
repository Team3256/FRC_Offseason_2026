// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "subsystems/shooterpivot/ShooterPivotIOTalonFX.h"

#include <ctre/phoenix6/controls/NeutralOut.hpp>

#include "utils/PhoenixUtil.h"

using namespace ctre::phoenix6;

ShooterPivotIOTalonFX::ShooterPivotIOTalonFX() {
  phoenix_util::ApplyMotorConfigs(m_motor, shooter_pivot::MotorConfigs(),
                                  shooter_pivot::kFlashConfigRetries);
  BaseStatusSignal::SetUpdateFrequencyForAll(shooter_pivot::kUpdateFrequency, m_voltage, m_velocity,
                                             m_position, m_supplyCurrent, m_statorCurrent);
  phoenix_util::RegisterSignals(
      false, {&m_voltage, &m_velocity, &m_position, &m_statorCurrent, &m_supplyCurrent});
  // Like the shooter and rollers: signals the code doesn't read drop to 4 Hz to save CAN bandwidth.
  m_motor.OptimizeBusUtilization();
  phoenix_util::WatchDevice(m_motor, "Shooter pivot motor");
}

void ShooterPivotIOTalonFX::UpdateInputs(ShooterPivotIOInputs& inputs) {
  inputs.shooterPivotMotorVoltage = m_voltage.GetValue().value();
  inputs.shooterPivotMotorVelocity = m_velocity.GetValue().value();
  inputs.shooterPivotMotorPosition = m_position.GetValue().value();
  inputs.shooterPivotMotorStatorCurrent = m_statorCurrent.GetValue().value();
  inputs.shooterPivotMotorSupplyCurrent = m_supplyCurrent.GetValue().value();
}

void ShooterPivotIOTalonFX::SetPosition(double position) {
  m_motor.SetControl(m_motionMagicRequest.WithPosition(units::turn_t{position}));
}

void ShooterPivotIOTalonFX::SetPosition(double position, double velocity) {
  m_motor.SetControl(m_positionVoltage.WithPosition(units::turn_t{position})
                         .WithVelocity(units::turns_per_second_t{velocity}));
}

void ShooterPivotIOTalonFX::SetVoltage(double voltage) {
  m_motor.SetVoltage(units::volt_t{voltage});
}

void ShooterPivotIOTalonFX::Off() { m_motor.SetControl(controls::NeutralOut{}); }

void ShooterPivotIOTalonFX::ResetPosition(double angle) {
  m_motor.SetPosition(units::turn_t{angle});
}
