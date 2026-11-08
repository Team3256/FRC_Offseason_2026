// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "subsystems/rollers/RollerPairIO.h"

#include <ctre/phoenix6/controls/NeutralOut.hpp>

#include "utils/PhoenixUtil.h"

using namespace ctre::phoenix6;

void AddRollerPairInputs(logging::InputsLogger& log, const RollerPairInputs& in,
                         const RollerPairKeys& k) {
  log.Add(k[0], &in.leftVoltage);
  log.Add(k[1], &in.leftVelocity);
  log.Add(k[2], &in.leftStatorCurrent);
  log.Add(k[3], &in.leftSupplyCurrent);
  log.Add(k[4], &in.leftTemperature);
  log.Add(k[5], &in.rightVoltage);
  log.Add(k[6], &in.rightVelocity);
  log.Add(k[7], &in.rightStatorCurrent);
  log.Add(k[8], &in.rightSupplyCurrent);
  log.Add(k[9], &in.rightTemperature);
}

RollerPairIOTalonFX::RollerPairIOTalonFX(const RollerPairConfig& config)
    : m_left{config.leftId},
      m_right{config.rightId},
      m_followerRequest{config.leftId, signals::MotorAlignmentValue::Opposed},
      m_leftVoltage{m_left.GetMotorVoltage(false)},
      m_leftVelocity{m_left.GetVelocity(false)},
      m_leftStator{m_left.GetStatorCurrent(false)},
      m_leftSupply{m_left.GetSupplyCurrent(false)},
      m_leftTemp{m_left.GetDeviceTemp(false)},
      m_rightVoltage{m_right.GetMotorVoltage(false)},
      m_rightVelocity{m_right.GetVelocity(false)},
      m_rightStator{m_right.GetStatorCurrent(false)},
      m_rightSupply{m_right.GetSupplyCurrent(false)},
      m_rightTemp{m_right.GetDeviceTemp(false)} {
  phoenix_util::ApplyMotorConfigs(m_left, config.motorConfigs, config.flashConfigRetries);
  phoenix_util::ApplyMotorConfigs(m_right, config.motorConfigs, config.flashConfigRetries);
  BaseStatusSignal::SetUpdateFrequencyForAll(
      config.updateFrequency, m_leftVoltage, m_leftVelocity, m_leftStator, m_leftSupply, m_leftTemp,
      m_rightVoltage, m_rightVelocity, m_rightStator, m_rightSupply, m_rightTemp);
  phoenix_util::RegisterSignals(
      false, {&m_leftVoltage, &m_leftVelocity, &m_leftStator, &m_leftSupply, &m_leftTemp,
              &m_rightVoltage, &m_rightVelocity, &m_rightStator, &m_rightSupply, &m_rightTemp});
  m_left.OptimizeBusUtilization();
  m_right.OptimizeBusUtilization();
  m_right.SetControl(m_followerRequest);
}

void RollerPairIOTalonFX::UpdateInputs(RollerPairInputs& inputs) {
  inputs.leftVoltage = m_leftVoltage.GetValueAsDouble();
  inputs.leftVelocity = m_leftVelocity.GetValueAsDouble();
  inputs.leftStatorCurrent = m_leftStator.GetValueAsDouble();
  inputs.leftSupplyCurrent = m_leftSupply.GetValueAsDouble();
  inputs.leftTemperature = m_leftTemp.GetValueAsDouble();
  inputs.rightVoltage = m_rightVoltage.GetValueAsDouble();
  inputs.rightVelocity = m_rightVelocity.GetValueAsDouble();
  inputs.rightStatorCurrent = m_rightStator.GetValueAsDouble();
  inputs.rightSupplyCurrent = m_rightSupply.GetValueAsDouble();
  inputs.rightTemperature = m_rightTemp.GetValueAsDouble();
}

void RollerPairIOTalonFX::SetVoltage(double voltage) {
  m_left.SetVoltage(units::volt_t{voltage});
  m_right.SetControl(m_followerRequest);
}

void RollerPairIOTalonFX::SetVelocity(double velocity) {
  m_left.SetControl(m_velocityRequest.WithVelocity(units::turns_per_second_t{velocity}));
  m_right.SetControl(m_followerRequest);
}

void RollerPairIOTalonFX::Off() { m_left.SetControl(controls::NeutralOut{}); }
