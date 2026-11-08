// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "logging/LoggedTracer.h"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <frc/RobotController.h>

#include "logging/Logger.h"

namespace logging {

namespace {
int64_t g_startUs = -1;
// Few entries (one per subsystem), so a linear scan over pointers beats a hash map.
std::vector<std::pair<const char*, std::unique_ptr<Output<double>>>> g_entries;

Output<double>& EntryFor(const char* name) {
  for (auto& [key, entry] : g_entries) {
    if (key == name) {
      return *entry;
    }
  }
  g_entries.emplace_back(
      name, std::make_unique<Output<double>>(std::string{"LoggedTracer/"} + name + "MS"));
  return *g_entries.back().second;
}
}  // namespace

void LoggedTracer::Reset() {
  g_startUs = static_cast<int64_t>(frc::RobotController::GetFPGATime());
}

void LoggedTracer::Record(const char* epochName) {
  int64_t now = static_cast<int64_t>(frc::RobotController::GetFPGATime());
  EntryFor(epochName).Set(static_cast<double>(now - g_startUs) / 1000.0);
  g_startUs = now;
}

}  // namespace logging
