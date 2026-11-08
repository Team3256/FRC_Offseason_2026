// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <frc/simulation/ElevatorSim.h>

#include "logging/Logger.h"
#include "subsystems/linearslide/LinearSlideIOTalonFX.h"

class LinearSlideIOSim : public LinearSlideIOTalonFX {
 public:
  LinearSlideIOSim();
  void UpdateInputs(LinearSlideIOInputs& inputs) override;

 private:
  frc::sim::ElevatorSim m_slideSim;
  logging::Output<double> m_positionLog{"LinearSlideSim/positionMeters"};
  logging::Output<double> m_velocityLog{"LinearSlideSim/velocityMetersPerSecond"};
};
