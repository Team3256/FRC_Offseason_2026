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
#include <ctre/phoenix6/controls/VoltageOut.hpp>

#include "subsystems/linearslide/LinearSlideConstants.h"
#include "subsystems/linearslide/LinearSlideIO.h"

class LinearSlideIOTalonFX : public LinearSlideIO {
 public:
  LinearSlideIOTalonFX();
  void UpdateInputs(LinearSlideIOInputs& inputs) override;
  void SetPosition(double target) override;
  void SetExtendedPosition(double target) override;
  void SetVoltage(double volts) override;
  void Off() override;
  void Zero() override;
  void ResetPosition(double angle) override;

  ctre::phoenix6::hardware::TalonFX& GetRightMotor() { return m_right; }
  ctre::phoenix6::hardware::TalonFX& GetLeftMotor() { return m_left; }

 private:
  /** Each motor runs its own PID loop to the same target. */
  void SetPosition(double target, int slot);

  ctre::phoenix6::hardware::TalonFX m_right{linear_slide::kRightMotorID};
  ctre::phoenix6::hardware::TalonFX m_left{linear_slide::kLeftMotorID};
  ctre::phoenix6::controls::PositionVoltage m_positionRequest =
      ctre::phoenix6::controls::PositionVoltage{0_tr}.WithSlot(0).WithEnableFOC(
          linear_slide::kUseFOC);
  ctre::phoenix6::controls::MotionMagicVoltage m_motionMagicRequest =
      ctre::phoenix6::controls::MotionMagicVoltage{0_tr}.WithSlot(0).WithEnableFOC(
          linear_slide::kUseFOC);
  ctre::phoenix6::controls::VoltageOut m_voltageRequest =
      ctre::phoenix6::controls::VoltageOut{0_V}.WithEnableFOC(linear_slide::kUseFOC);

  ctre::phoenix6::StatusSignal<units::volt_t>& m_leftVoltage = m_left.GetMotorVoltage(false);
  ctre::phoenix6::StatusSignal<units::turns_per_second_t>& m_leftVelocity =
      m_left.GetVelocity(false);
  ctre::phoenix6::StatusSignal<units::turn_t>& m_leftPosition = m_left.GetPosition(false);
  ctre::phoenix6::StatusSignal<units::ampere_t>& m_leftStator = m_left.GetStatorCurrent(false);
  ctre::phoenix6::StatusSignal<units::ampere_t>& m_leftSupply = m_left.GetSupplyCurrent(false);
  ctre::phoenix6::StatusSignal<units::volt_t>& m_rightVoltage = m_right.GetMotorVoltage(false);
  ctre::phoenix6::StatusSignal<units::turns_per_second_t>& m_rightVelocity =
      m_right.GetVelocity(false);
  ctre::phoenix6::StatusSignal<units::turn_t>& m_rightPosition = m_right.GetPosition(false);
  ctre::phoenix6::StatusSignal<units::ampere_t>& m_rightStator = m_right.GetStatorCurrent(false);
  ctre::phoenix6::StatusSignal<units::ampere_t>& m_rightSupply = m_right.GetSupplyCurrent(false);
};
