// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <units/length.h>

/**
 * Robot-wide constants. Subsystem constants live next to each subsystem.
 */
namespace constants {

namespace controller {
inline constexpr int kDriverControllerPort = 0;
inline constexpr int kOperatorControllerPort = 1;

namespace driver {
inline constexpr double kDriverStickDeadband = 0.1;
inline constexpr double kDriverRotationalDeadband = 0.12;
inline constexpr double kOperatorLeftDeadband = 0.0;
inline constexpr double kOperatorRightDeadband = 0.0;
}  // namespace driver
}  // namespace controller

// Naming scheme for FeatureFlags:
// k___Enabled = enables/disables a subsystem
// kUse___ = enables/disables a specific feature (those live in the subsystem constants)
namespace feature_flags {
/** Logging (file + NetworkTables, see logging/Logger.h). NetworkTables is off on the field. */
inline constexpr bool kLoggingEnabled = true;
/** Simulate the PhotonVision camera(s) in simulation so pose estimation is exercised too. */
inline constexpr bool kSimulateVision = true;
}  // namespace feature_flags

namespace simulation {
inline constexpr units::meter_t kDrivebaseWidth = 27_in;
}  // namespace simulation

namespace logging_config {
/** Write logs to the USB stick (/U/wpilogs); falls back to /home/lvuser/wpilogs without one. */
inline constexpr bool kLogToUSB = true;
inline constexpr const char* kUsbLogDir = "/U/wpilogs";
inline constexpr const char* kRioLogDir = "/home/lvuser/wpilogs";
/** Desktop simulation log directory (relative to the working directory). */
inline constexpr const char* kSimLogDir = "logs";
}  // namespace logging_config

}  // namespace constants
