// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include "subsystems/shooterpivot/ShooterPivotIOTalonFX.h"

/** Simulated hood: the real TalonFX IO, its motor and the hood simulated on sim::PhysicsWorld. */
class ShooterPivotIOSim : public ShooterPivotIOTalonFX {
 public:
  ShooterPivotIOSim();
  void UpdateInputs(ShooterPivotIOInputs& inputs) override;
  void ResetPosition(double angle) override;
  double SimAngleRadians() const;
};
