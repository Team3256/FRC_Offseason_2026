// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <frc/simulation/SingleJointedArmSim.h>

#include "subsystems/shooterpivot/ShooterPivotIOTalonFX.h"

class ShooterPivotIOSim : public ShooterPivotIOTalonFX {
 public:
  ShooterPivotIOSim();
  void UpdateInputs(ShooterPivotIOInputs& inputs) override;
  double SimAngleRadians() const { return m_pivotSim.GetAngle().value(); }

 private:
  frc::sim::SingleJointedArmSim m_pivotSim;
};
