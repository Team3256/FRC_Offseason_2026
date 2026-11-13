// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include "logging/Logger.h"
#include "subsystems/linearslide/LinearSlideIOTalonFX.h"

/**
 * Simulated slide: the real TalonFX IO, both motors and the slide (rack plates, hopper and roller
 * on a 12 degree rack) simulated on sim::PhysicsWorld.
 */
class LinearSlideIOSim : public LinearSlideIOTalonFX {
 public:
  LinearSlideIOSim();
  void UpdateInputs(LinearSlideIOInputs& inputs) override;
  void ResetPosition(double angle) override;
  void Zero() override;

 private:
  logging::Output<double> m_positionLog{"LinearSlideSim/positionMeters"};
};
