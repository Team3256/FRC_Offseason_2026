// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "subsystems/linearslide/LinearSlideIOTalonFX.h"

#include <ctre/phoenix6/controls/NeutralOut.hpp>

#include "utils/PhoenixUtil.h"

using namespace ctre::phoenix6;

LinearSlideIOTalonFX::LinearSlideIOTalonFX() {
  phoenix_util::ApplyMotorConfigs(m_right, linear_slide::RightMotorConfigs(),
                                  linear_slide::kFlashConfigRetries);
  phoenix_util::ApplyMotorConfigs(m_left, linear_slide::LeftMotorConfigs(),
                                  linear_slide::kFlashConfigRetries);
  BaseStatusSignal::SetUpdateFrequencyForAll(linear_slide::kUpdateFrequency, m_rightVoltage,
                                             m_rightVelocity, m_rightPosition, m_rightStator,
                                             m_rightSupply, m_leftVoltage, m_leftVelocity,
                                             m_leftPosition, m_leftStator, m_leftSupply);
  phoenix_util::RegisterSignals(
      false, {&m_rightVoltage, &m_rightVelocity, &m_rightPosition, &m_rightStator, &m_rightSupply,
              &m_leftVoltage, &m_leftVelocity, &m_leftPosition, &m_leftStator, &m_leftSupply});
  // Like the shooter and rollers: signals the code doesn't read drop to 4 Hz to save CAN bandwidth.
  m_right.OptimizeBusUtilization();
  m_left.OptimizeBusUtilization();
  phoenix_util::WatchDevice(m_right, "Linear slide right motor");
  phoenix_util::WatchDevice(m_left, "Linear slide left motor");
}

void LinearSlideIOTalonFX::UpdateInputs(LinearSlideIOInputs& inputs) {
  inputs.rightMotorVoltage = m_rightVoltage.GetValue().value();
  inputs.rightMotorVelocity = m_rightVelocity.GetValue().value();
  inputs.rightMotorPosition = m_rightPosition.GetValueAsDouble();
  inputs.rightMotorStatorCurrent = m_rightStator.GetValue().value();
  inputs.rightMotorSupplyCurrent = m_rightSupply.GetValue().value();

  inputs.leftMotorVoltage = m_leftVoltage.GetValue().value();
  inputs.leftMotorVelocity = m_leftVelocity.GetValue().value();
  inputs.leftMotorPosition = m_leftPosition.GetValueAsDouble();
  inputs.leftMotorStatorCurrent = m_leftStator.GetValue().value();
  inputs.leftMotorSupplyCurrent = m_leftSupply.GetValue().value();
}

void LinearSlideIOTalonFX::SetPosition(double target) { SetPosition(target, 0); }

void LinearSlideIOTalonFX::SetExtendedPosition(double target) { SetPosition(target, 1); }

void LinearSlideIOTalonFX::SetPosition(double target, int slot) {
  if (linear_slide::kUseMotionMagic) {
    m_right.SetControl(m_motionMagicRequest.WithPosition(units::turn_t{target}).WithSlot(slot));
    m_left.SetControl(m_motionMagicRequest.WithPosition(units::turn_t{target}).WithSlot(slot));
  } else {
    m_right.SetControl(m_positionRequest.WithPosition(units::turn_t{target}).WithSlot(slot));
    m_left.SetControl(m_positionRequest.WithPosition(units::turn_t{target}).WithSlot(slot));
  }
}

void LinearSlideIOTalonFX::SetVoltage(double volts) {
  m_right.SetControl(m_voltageRequest.WithOutput(units::volt_t{volts}));
  m_left.SetControl(m_voltageRequest.WithOutput(units::volt_t{volts}));
}

void LinearSlideIOTalonFX::Off() {
  m_right.SetControl(controls::NeutralOut{});
  m_left.SetControl(controls::NeutralOut{});
}

void LinearSlideIOTalonFX::Zero() {
  m_right.SetPosition(0_tr);
  m_left.SetPosition(0_tr);
}

void LinearSlideIOTalonFX::ResetPosition(double angle) {
  m_right.SetPosition(units::turn_t{angle});
  m_left.SetPosition(units::turn_t{angle});
}
