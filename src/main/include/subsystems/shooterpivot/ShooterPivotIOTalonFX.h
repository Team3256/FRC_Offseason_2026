// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/controls/MotionMagicVoltage.hpp>
#include <ctre/phoenix6/controls/PositionVoltage.hpp>

#include "subsystems/shooterpivot/ShooterPivotConstants.h"
#include "subsystems/shooterpivot/ShooterPivotIO.h"

class ShooterPivotIOTalonFX : public ShooterPivotIO {
 public:
  ShooterPivotIOTalonFX();
  void UpdateInputs(ShooterPivotIOInputs& inputs) override;
  void SetPosition(double position) override;
  void SetPosition(double position, double velocity) override;
  void SetVoltage(double voltage) override;
  void Off() override;
  void ResetPosition(double angle) override;

  ctre::phoenix6::hardware::TalonFX& GetMotor() { return m_motor; }

 private:
  ctre::phoenix6::hardware::TalonFX m_motor{shooter_pivot::kPivotMotorId};
  ctre::phoenix6::controls::MotionMagicVoltage m_motionMagicRequest =
      ctre::phoenix6::controls::MotionMagicVoltage{0_tr}.WithSlot(0).WithEnableFOC(
          shooter_pivot::kUseFOC);
  ctre::phoenix6::controls::PositionVoltage m_positionVoltage =
      ctre::phoenix6::controls::PositionVoltage{0_tr}.WithSlot(0).WithEnableFOC(
          shooter_pivot::kUseFOC);
  ctre::phoenix6::StatusSignal<units::volt_t>& m_voltage = m_motor.GetMotorVoltage(false);
  ctre::phoenix6::StatusSignal<units::turns_per_second_t>& m_velocity = m_motor.GetVelocity(false);
  ctre::phoenix6::StatusSignal<units::turn_t>& m_position = m_motor.GetPosition(false);
  ctre::phoenix6::StatusSignal<units::ampere_t>& m_statorCurrent = m_motor.GetStatorCurrent(false);
  ctre::phoenix6::StatusSignal<units::ampere_t>& m_supplyCurrent = m_motor.GetSupplyCurrent(false);
};
