// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "subsystems/shooter/ShooterIOTalonFX.h"

#include "utils/PhoenixUtil.h"

using namespace ctre::phoenix6;

namespace {
constexpr std::array<const char*, shooter::kNumMotors> kMotorNames = {
    "Shooter top left motor", "Shooter bottom left motor", "Shooter top right motor",
    "Shooter bottom right motor"};

controls::Follower FollowerFor(int i) {
  return controls::Follower{shooter::kMotorIds[0], shooter::kMotorOpposed[i]
                                                       ? signals::MotorAlignmentValue::Opposed
                                                       : signals::MotorAlignmentValue::Aligned};
}
}  // namespace

ShooterIOTalonFX::ShooterIOTalonFX()
    : m_motors{{hardware::TalonFX{shooter::kMotorIds[0]}, hardware::TalonFX{shooter::kMotorIds[1]},
                hardware::TalonFX{shooter::kMotorIds[2]},
                hardware::TalonFX{shooter::kMotorIds[3]}}},
      m_followRequests{{FollowerFor(0), FollowerFor(1), FollowerFor(2), FollowerFor(3)}} {
  const auto config = shooter::MotorConfigs();
  for (int i = 0; i < shooter::kNumMotors; i++) {
    auto& m = m_motors[i];
    m_voltage[i] = &m.GetMotorVoltage(false);
    m_velocity[i] = &m.GetVelocity(false);
    m_statorCurrent[i] = &m.GetStatorCurrent(false);
    m_supplyCurrent[i] = &m.GetSupplyCurrent(false);
    m_temperature[i] = &m.GetDeviceTemp(false);
    phoenix_util::ApplyMotorConfigs(m, config, shooter::kFlashConfigRetries);
    BaseStatusSignal::SetUpdateFrequencyForAll(shooter::kUpdateFrequency, *m_voltage[i],
                                               *m_velocity[i], *m_statorCurrent[i],
                                               *m_supplyCurrent[i], *m_temperature[i]);
    phoenix_util::RegisterSignals(false, {m_voltage[i], m_velocity[i], m_statorCurrent[i],
                                          m_supplyCurrent[i], m_temperature[i]});
    phoenix_util::WatchDevice(m, kMotorNames[i]);
    phoenix_util::WatchTemperature(*m_temperature[i], kMotorNames[i]);
  }
  for (int i = 0; i < shooter::kNumMotors; i++) {
    m_motors[i].OptimizeBusUtilization();
    if (i > 0) {
      m_motors[i].SetControl(m_followRequests[i]);
    }
  }
}

void ShooterIOTalonFX::UpdateInputs(ShooterIOInputs& inputs) {
  for (int i = 0; i < shooter::kNumMotors; i++) {
    inputs.shooterMotorVoltages[i] = m_voltage[i]->GetValueAsDouble();
    inputs.shooterMotorVelocitys[i] = m_velocity[i]->GetValueAsDouble();
    inputs.shooterMotorStatorCurrents[i] = m_statorCurrent[i]->GetValueAsDouble();
    inputs.shooterMotorSupplyCurrents[i] = m_supplyCurrent[i]->GetValueAsDouble();
    inputs.shooterMotorTemperatures[i] = m_temperature[i]->GetValueAsDouble();
    inputs.shooterMotorIDs[i] = shooter::kMotorIds[i];
  }
}

void ShooterIOTalonFX::FollowLeader() {
  for (int i = 1; i < shooter::kNumMotors; i++) {
    m_motors[i].SetControl(m_followRequests[i]);
  }
}

void ShooterIOTalonFX::SetShooterVoltage(double voltage) {
  m_motors[0].SetVoltage(units::volt_t{voltage});
  FollowLeader();
}

void ShooterIOTalonFX::SetShooterVelocity(double velocity) {
  m_motors[0].SetControl(m_velocityRequest.WithVelocity(units::turns_per_second_t{velocity}));
  FollowLeader();
}

void ShooterIOTalonFX::Off() { m_motors[0].SetControl(controls::NeutralOut{}); }
