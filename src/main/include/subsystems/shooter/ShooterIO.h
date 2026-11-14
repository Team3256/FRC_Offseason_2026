// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <array>

#include "subsystems/shooter/ShooterConstants.h"

struct ShooterIOInputs {
  std::array<double, shooter::kNumMotors> shooterMotorVoltages{};
  std::array<double, shooter::kNumMotors> shooterMotorVelocitys{};
  std::array<double, shooter::kNumMotors> shooterMotorStatorCurrents{};
  std::array<double, shooter::kNumMotors> shooterMotorSupplyCurrents{};
  std::array<double, shooter::kNumMotors> shooterMotorTemperatures{};
  std::array<double, shooter::kNumMotors> shooterMotorIDs{};
};

class ShooterIO {
 public:
  virtual ~ShooterIO() = default;
  virtual void UpdateInputs(ShooterIOInputs& inputs) {}
  virtual void SetShooterVoltage(double voltage) {}
  virtual void SetShooterVelocity(double velocity) {}
  virtual void Off() {}
};
