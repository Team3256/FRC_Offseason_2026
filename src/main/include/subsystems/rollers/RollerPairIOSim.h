// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <functional>

#include <frc/simulation/FlywheelSim.h>
#include <units/angle.h>

#include "subsystems/rollers/RollerPairIO.h"

/**
 * Simulated roller pair: the real TalonFX IO with a flywheel model driven by the leader's output.
 * The small differences between the Java intake/feeder/indexer simulations are kept as options.
 */
class RollerPairIOSim : public RollerPairIOTalonFX {
 public:
  struct Options {
    /** Follower rotor spins opposite the leader in the sim (intake) or with it (feeder/indexer). */
    bool followerReversed = false;
    /** Also update the follower's supply voltage (the indexer sim did not). */
    bool setFollowerSupply = true;
    /** Mechanism2d update, given the rotation delta to draw. */
    std::function<void(units::degree_t)> visualize;
    double angularVelocityScalar = 1;
  };

  RollerPairIOSim(const RollerPairConfig& config, const frc::LinearSystem<1, 1, 1>& plant,
                  const frc::DCMotor& gearbox, Options options);
  void UpdateInputs(RollerPairInputs& inputs) override;

  double SimRps() const { return m_lastRps; }

 private:
  frc::sim::FlywheelSim m_sim;
  Options m_options;
  double m_lastRps = 0;
};

std::unique_ptr<RollerPairIO> MakeIntakeRollersIO(bool simulation);
std::unique_ptr<RollerPairIO> MakeFeederIO(bool simulation);
std::unique_ptr<RollerPairIO> MakeIndexerIO(bool simulation);
