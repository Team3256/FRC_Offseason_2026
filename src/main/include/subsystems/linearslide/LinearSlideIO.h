// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

struct LinearSlideIOInputs {
  double rightMotorVoltage = 0.0;
  double rightMotorVelocity = 0.0;
  double rightMotorPosition = 0.0;
  double rightMotorStatorCurrent = 0.0;
  double rightMotorSupplyCurrent = 0.0;

  double leftMotorVoltage = 0.0;
  double leftMotorVelocity = 0.0;
  double leftMotorPosition = 0.0;
  double leftMotorStatorCurrent = 0.0;
  double leftMotorSupplyCurrent = 0.0;
};

class LinearSlideIO {
 public:
  virtual ~LinearSlideIO() = default;
  virtual void UpdateInputs(LinearSlideIOInputs& inputs) {}
  virtual void SetPosition(double position) {}
  virtual void SetExtendedPosition(double target) {}
  virtual void SetVoltage(double voltage) {}
  virtual void ResetPosition(double angle) {}
  virtual void Off() {}
  virtual void Zero() {}
};
