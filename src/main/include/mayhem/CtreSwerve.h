// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <memory>

#include "mayhem/Auto.h"

namespace subsystems {
class CommandSwerveDrivetrain;
}

namespace mayhem {

/**
 * CTRE Phoenix 6 swerve integration (MayhemLib's CtreSwerve.autoFactory): pose and speeds from the
 * drivetrain, output through an ApplyFieldSpeeds request (blue-alliance perspective, desaturated,
 * velocity drive), and the Pigeon 2 as accelerometer and tilt sensor. Module order must match the
 * trajectory's module order (FL, FR, BL, BR), the Tuner X default.
 */
std::unique_ptr<AutoFactory> CtreSwerveAutoFactory(subsystems::CommandSwerveDrivetrain& drivetrain);

}  // namespace mayhem
