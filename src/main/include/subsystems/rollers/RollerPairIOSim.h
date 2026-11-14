// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <functional>
#include <memory>

#include <units/angle.h>

#include "sim/PhysicsWorld.h"
#include "subsystems/rollers/RollerPairIO.h"

/**
 * Simulated roller pair (intake rollers, feeder, indexer): the real TalonFX IO, both motors and the
 * roller simulated on sim::PhysicsWorld. The follower is simulated as Phoenix's Follower request.
 */
class RollerPairIOSim : public RollerPairIOTalonFX {
 public:
  /** Mechanism2d update, given the rotation delta to draw. */
  using Visualize = std::function<void(units::degree_t)>;
  RollerPairIOSim(const RollerPairConfig& config, sim::Mechanism mechanism, Visualize visualize,
                  double angularVelocityScalar);
  void UpdateInputs(RollerPairInputs& inputs) override;
  double SimRps() const { return m_lastRps; }

 private:
  Visualize m_visualize;
  double m_angularVelocityScalar;
  double m_lastRps = 0;
};

std::unique_ptr<RollerPairIO> MakeIntakeRollersIO(bool simulation);
std::unique_ptr<RollerPairIO> MakeFeederIO(bool simulation);
std::unique_ptr<RollerPairIO> MakeIndexerIO(bool simulation);
