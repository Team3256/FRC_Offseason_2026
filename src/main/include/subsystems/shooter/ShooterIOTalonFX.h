// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <array>

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/controls/Follower.hpp>
#include <ctre/phoenix6/controls/NeutralOut.hpp>
#include <ctre/phoenix6/controls/VelocityVoltage.hpp>

#include "subsystems/shooter/ShooterIO.h"

class ShooterIOTalonFX : public ShooterIO {
 public:
  ShooterIOTalonFX();
  void UpdateInputs(ShooterIOInputs& inputs) override;
  void SetShooterVoltage(double voltage) override;
  void SetShooterVelocity(double velocity) override;
  void Off() override;

  std::array<ctre::phoenix6::hardware::TalonFX, shooter::kNumMotors>& GetMotors() {
    return m_motors;
  }

 private:
  void FollowLeader();

  std::array<ctre::phoenix6::hardware::TalonFX, shooter::kNumMotors> m_motors;
  ctre::phoenix6::controls::VelocityVoltage m_velocityRequest =
      ctre::phoenix6::controls::VelocityVoltage{0_tps}.WithSlot(0);
  std::array<ctre::phoenix6::controls::Follower, shooter::kNumMotors> m_followRequests;
  std::array<ctre::phoenix6::StatusSignal<units::volt_t>*, shooter::kNumMotors> m_voltage;
  std::array<ctre::phoenix6::StatusSignal<units::turns_per_second_t>*, shooter::kNumMotors>
      m_velocity;
  std::array<ctre::phoenix6::StatusSignal<units::ampere_t>*, shooter::kNumMotors> m_statorCurrent;
  std::array<ctre::phoenix6::StatusSignal<units::ampere_t>*, shooter::kNumMotors> m_supplyCurrent;
  std::array<ctre::phoenix6::StatusSignal<units::celsius_t>*, shooter::kNumMotors> m_temperature;
};
