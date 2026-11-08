// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <cmath>
#include <numbers>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>

#include "FieldConstants.h"
#include "subsystems/swerve/generated/TunerConstants.h"

namespace swerve_constants {

inline constexpr double deadbandMultiplier = 0.15;
inline constexpr double MaxSpeed = TunerConstants::kSpeedAt12Volts.value();
inline constexpr double MaxAngularRate = 1.5 * std::numbers::pi;
inline constexpr double SlowMaxSpeed = MaxSpeed * 0.3;
inline constexpr double SlowMaxAngular = MaxAngularRate * 0.3;
inline constexpr double SuperSlowMaxSpeed = MaxSpeed * 0.15;

// Physics constants
inline constexpr double kRobotMassLbs = 120;
inline constexpr double kRobotMOI = 36;  // kg m^2

// TODO: adjust each for overshoot
namespace azimuth_targets {
// TODO: tune these jawns
inline constexpr double aziKP = 6.0;
inline constexpr double aziKi = 0.0;
inline constexpr double aziKD = 0.0;
inline constexpr double timeout = 0.3;
inline const frc::Rotation2d bump{units::degree_t{45}};
}  // namespace azimuth_targets

namespace bump_targets {
inline const frc::Pose2d BOTTOM_BUMP{3.3690896034240723_m, 2.46048903465271_m,
                                     frc::Rotation2d{units::degree_t{45.744063302312063968}}};
inline const frc::Pose2d TOP_BUMP{3.4050545692443848_m, 5.535476207733154_m,
                                  frc::Rotation2d{units::degree_t{45.744063302312063968}}};
}  // namespace bump_targets

/**
 * Trench assist: automatically centers the robot in a trench lane (and squares it up to the walls)
 * when the driver heads into one. Distances in meters, speeds in m/s.
 *
 * It only runs in teleop. To turn it off, set kUseTrenchAssist to false, or flip
 * "TrenchAssist/Enabled" on the dashboard (no redeploy needed).
 */
namespace trench_assist {
/** Default for the dashboard toggle; false disables trench assist entirely. */
inline constexpr bool kUseTrenchAssist = false;
/** Half the robot's size (bumpers included); the chassis is square. */
inline constexpr double kRobotHalfLength = 16.5 * 0.0254;
/** Lateral distance from the lane centerline within which the assist is at full strength. */
inline constexpr double kLaneHalfWidth = field_constants::left_trench::openingWidth / 2.0;
/** Extra lateral distance over which the assist fades out (so bump approaches are not hijacked). */
inline constexpr double kCaptureMargin = 8.0 * 0.0254;
/** tan(30 deg): the driver must be heading within this angle of the trench axis. */
inline constexpr double kApproachAngleTan = 0.57735026918962573;
/** Driver speed along the trench axis below which the assist stays out of the way. */
inline constexpr double kMinAssistSpeed = 1.0;
/** Speed range over which the assist fades in, above kMinAssistSpeed. */
inline constexpr double kAssistSpeedBlend = 0.75;
/** The assist starts this far before the trench entrance, plus the lookahead below. */
inline constexpr double kBaseApproachDistance = 0.4;
/** Extra approach distance per m/s of driver speed, so fast drivers are captured earlier. */
inline constexpr double kApproachLookahead = 0.45;
/** The assist is at full strength this many seconds (of travel) before the entrance. */
inline constexpr double kFullStrengthLeadTime = 0.25;
/** Distance after leaving the trench over which the assist fades out. */
inline constexpr double kExitFadeDistance = 0.3;
/** Centering controller: lateral velocity per meter of lateral error. */
inline constexpr double kCenteringKP = 6.0;
inline constexpr double kMaxCenteringSpeed = 3.0;
/** With this much room (or more) between robot and walls, heading is left alone. */
inline constexpr double kHeadingFreeSlack = 0.14;
/** Room between the robot and the walls at or below which heading is fully squared up. */
inline constexpr double kHeadingFullSnapSlack = 0.07;
/** Heading controller used to square the robot to the trench walls (nearest 90 degrees). */
inline constexpr double kHeadingKP = 6.0;
/** Driver turn rate (rad/s) above which the driver keeps control of heading. */
inline constexpr double kDriverTurnOverrideRate = 0.2 * MaxAngularRate;
}  // namespace trench_assist

}  // namespace swerve_constants
