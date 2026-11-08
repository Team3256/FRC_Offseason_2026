// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <array>
#include <numbers>
#include <string_view>

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/geometry/Rotation3d.h>
#include <frc/geometry/Transform3d.h>
#include <units/angle.h>
#include <units/length.h>

namespace vision {

/** The WPILib default field (2026 welded). Loaded once on first use. */
const frc::AprilTagFieldLayout& AprilTagLayout();

// Camera names, must match names configured on coprocessor
inline constexpr std::string_view kFrontRightCam = "frontRight";
inline constexpr std::string_view kFrontLeftCam = "frontLeft";
inline constexpr std::string_view kBackCam = "middle";

// Robot to camera transforms
inline const frc::Transform3d kRobotToFrontRightCam{-1.997_in, -13.770_in, 8.155_in,
                                                    frc::Rotation3d{0_deg, 25_deg, -90_deg}};
inline const frc::Transform3d kRobotToFrontLeftCam{2.490_in, 13.276_in, 8.408_in,
                                                   frc::Rotation3d{0_deg, -25_deg, 90_deg}};
inline const frc::Transform3d kRobotToBackCam{-13.667_in, 0.0_in, -7.678_in,
                                              frc::Rotation3d{0_deg, -25_deg, 180_deg}};

// Basic filtering thresholds
inline constexpr double kMaxAmbiguity = 0.3;
inline constexpr double kMaxZError = 0.75;

// Standard deviation baselines, for 1 meter distance and 1 tag
// (Adjusted automatically based on distance and # of tags)
inline constexpr double kLinearStdDevBaseline = 0.02;   // Meters
inline constexpr double kAngularStdDevBaseline = 0.99;  // Radians

// Standard deviation multipliers for each camera
// (Adjust to trust some cameras more than others)
inline constexpr std::array<double, 4> kCameraStdDevFactors = {1, 1, 1, 1};

/** Tags that are not on a hub; observations using any of them are trusted 1.3x less. */
inline constexpr std::array<short, 16> kNonHub = {1,  6,  7,  12, 13, 14, 15, 16,
                                                  17, 22, 23, 28, 29, 30, 31, 32};

// Simulated camera (VisionIOSim)
namespace sim {
inline constexpr units::degree_t kHorizontalFov = 80_deg;
inline constexpr units::degree_t kVerticalFov = 64_deg;
inline constexpr units::meter_t kMaxRange = 6_m;
/** Tags seen more than this far off their facing direction are not detected. */
inline constexpr units::degree_t kMaxViewAngle = 70_deg;
inline constexpr double kSingleTagAmbiguity = 0.05;
}  // namespace sim

}  // namespace vision
