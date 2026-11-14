// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include "subsystems/shooter/ShooterIOTalonFX.h"

/**
 * Simulated shooter: the real TalonFX IO, with its four motors simulated on sim::PhysicsWorld (a
 * flywheel plant driven by Mayhem's TalonFX emulator configured like the real motors).
 */
class ShooterIOSim : public ShooterIOTalonFX {
 public:
  ShooterIOSim();
  void UpdateInputs(ShooterIOInputs& inputs) override;
};
