// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

struct ShooterPivotIOInputs {
  double shooterPivotMotorVoltage = 0.0;
  double shooterPivotMotorVelocity = 0.0;
  double shooterPivotMotorPosition = 0.0;
  double shooterPivotMotorStatorCurrent = 0.0;
  double shooterPivotMotorSupplyCurrent = 0.0;
};

class ShooterPivotIO {
 public:
  virtual ~ShooterPivotIO() = default;
  virtual void UpdateInputs(ShooterPivotIOInputs& inputs) {}
  virtual void SetPosition(double position) {}
  virtual void SetPosition(double position, double velocity) {}
  virtual void SetVoltage(double voltage) {}
  virtual void ResetPosition(double angle) {}
  virtual void Off() {}
};
