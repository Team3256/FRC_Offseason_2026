// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <array>

#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>

/**
 * Trench assist. When the driver heads into a trench, this nudges the robot onto the lane's
 * centerline and squares it up to the walls, so the driver can go through at full speed without
 * clipping the trench structure. A pure function over the driver's requested speeds and the robot
 * pose, in the blue-origin field frame. See the Java version's documentation for the full design.
 */
namespace trench_assist {

/** What the assist decided this loop; weight == 0 means the driver is untouched. */
struct Result {
  frc::ChassisSpeeds speeds;
  double weight;
};

/**
 * @param pose robot pose in the blue-origin field frame
 * @param driverSpeeds the driver's requested speeds, field-relative in the blue-origin frame
 */
Result Apply(const frc::Pose2d& pose, const frc::ChassisSpeeds& driverSpeeds);

/** Lane centerlines, for tests and visualization. */
std::array<double, 2> LaneCenterYs();
/** Trench center x positions, for tests and visualization. */
std::array<double, 2> TrenchCenterXs();

}  // namespace trench_assist
