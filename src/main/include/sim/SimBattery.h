// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <units/current.h>

/**
 * Collects the supply current of every simulated mechanism motor during a loop (as computed by the
 * Phoenix 6 simulated TalonFXs, so current limits are respected), then applies the loaded battery
 * voltage once (Robot::SimulationPeriodic). The Java simulation set the voltage separately from
 * each mechanism using the physics models' unlimited current, so only the last mechanism's load
 * counted. The swerve drive (simulated by Phoenix) is not included, as before.
 */
namespace sim_battery {
void AddCurrent(units::ampere_t current);
/** Applies the summed load to the simulated battery and resets the sum. */
void Update();
units::ampere_t LastTotalCurrent();
}  // namespace sim_battery
