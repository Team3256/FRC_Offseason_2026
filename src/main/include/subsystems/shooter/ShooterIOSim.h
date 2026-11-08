// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <frc/simulation/FlywheelSim.h>

#include "subsystems/shooter/ShooterIOTalonFX.h"

/** Simulated shooter: the real TalonFX IO driven by a flywheel physics model. */
class ShooterIOSim : public ShooterIOTalonFX {
 public:
  ShooterIOSim();
  void UpdateInputs(ShooterIOInputs& inputs) override;

 private:
  frc::sim::FlywheelSim m_flywheelSim;
};
