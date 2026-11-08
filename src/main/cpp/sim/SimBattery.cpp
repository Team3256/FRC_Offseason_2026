// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "sim/SimBattery.h"

#include <frc/simulation/BatterySim.h>
#include <frc/simulation/RoboRioSim.h>

namespace sim_battery {
namespace {
units::ampere_t g_total = 0_A;
units::ampere_t g_last = 0_A;
}  // namespace

void AddCurrent(units::ampere_t current) {
  // Regenerative (negative) supply current is ignored: a real battery barely rises, and letting
  // it raise the simulated battery voltage makes the simulated TalonFXs fault on over-voltage.
  if (current > 0_A) {
    g_total += current;
  }
}

void Update() {
  frc::sim::RoboRioSim::SetVInVoltage(frc::sim::BatterySim::Calculate({g_total}));
  g_last = g_total;
  g_total = 0_A;
}

units::ampere_t LastTotalCurrent() { return g_last; }
}  // namespace sim_battery
