// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "subsystems/swerve/TrenchAssist.h"

#include <cmath>
#include <numbers>

#include <frc/MathUtil.h>

#include "FieldConstants.h"
#include "subsystems/swerve/SwerveConstants.h"
#include "utils/Util.h"

namespace trench_assist {

namespace {
using namespace swerve_constants::trench_assist;
namespace fc = field_constants;

/** One trench lane: the x of its center, the y of its centerline. */
struct Lane {
  double centerX;
  double centerY;
};

constexpr double kTrenchHalfDepth = fc::left_trench::depth / 2.0;
/** Distance from the trench center along x at which the robot body starts overlapping it. */
constexpr double kOverlapHalfDepth = kTrenchHalfDepth + kRobotHalfLength;
constexpr double kRightLaneY = fc::right_trench::openingWidth / 2.0;
constexpr double kLeftLaneY = fc::fieldWidth - fc::left_trench::openingWidth / 2.0;

// Both alliances' trenches, on both sides of the field.
constexpr std::array<Lane, 4> kLanes = {{
    {fc::lines_vertical::hubCenter, kRightLaneY},
    {fc::lines_vertical::hubCenter, kLeftLaneY},
    {fc::lines_vertical::oppHubCenter, kRightLaneY},
    {fc::lines_vertical::oppHubCenter, kLeftLaneY},
}};

/** Smooth 0 to 1 ramp over [0, 1], clamped. */
constexpr double Smoothstep(double t) {
  t = util::Limit(t, 0.0, 1.0);
  return t * t * (3.0 - 2.0 * t);
}

/** How strongly the assist applies for one lane, in [0, 1]. */
double Weight(double x, double y, double driverVx, double driverVy, const Lane& lane) {
  // Only engage for a driver moving along the trench axis at speed.
  const double speed = std::abs(driverVx);
  const double speedGate = Smoothstep((speed - kMinAssistSpeed) / kAssistSpeedBlend);
  if (speedGate <= 0.0) {
    return 0.0;
  }
  const double ratio = std::abs(driverVy) / speed;
  const double angleGate =
      1.0 - Smoothstep((ratio - kApproachAngleTan) / (1.0 - kApproachAngleTan));
  const double intent = speedGate * angleGate;
  if (intent <= 0.0) {
    return 0.0;
  }

  const double dx = x - lane.centerX;
  const double outside = std::abs(dx) - kOverlapHalfDepth;
  const bool approaching = outside > 0.0 && driverVx * dx < 0.0;

  // Lined up with this lane? On approach, judged by where the driver will be at the entrance.
  double lateralError = std::abs(y - lane.centerY);
  if (approaching) {
    lateralError = std::abs(y - lane.centerY + driverVy * outside / speed);
  }
  const double lateralWeight = 1.0 - Smoothstep((lateralError - kLaneHalfWidth) / kCaptureMargin);
  if (lateralWeight <= 0.0) {
    return 0.0;
  }

  double axialWeight;
  if (outside <= 0.0) {
    axialWeight = 1.0;  // robot overlaps the trench
  } else if (approaching) {
    // Heading toward the trench: ramp in over a speed-dependent approach distance.
    const double reach = kBaseApproachDistance + kApproachLookahead * speed;
    const double fullStrength = kFullStrengthLeadTime * speed;
    axialWeight = 1.0 - Smoothstep((outside - fullStrength) / (reach - fullStrength));
  } else {
    // Just left the trench (or driving away from it): fade out quickly.
    axialWeight = 1.0 - Smoothstep(outside / kExitFadeDistance);
  }
  return intent * axialWeight * lateralWeight;
}
}  // namespace

Result Apply(const frc::Pose2d& pose, const frc::ChassisSpeeds& driverSpeeds) {
  const double px = pose.X().value();
  const double py = pose.Y().value();
  const double vxIn = driverSpeeds.vx.value();
  const double vyIn = driverSpeeds.vy.value();
  const Lane* best = nullptr;
  double bestWeight = 0.0;
  for (const auto& lane : kLanes) {
    const double w = Weight(px, py, vxIn, vyIn, lane);
    if (w > bestWeight) {
      bestWeight = w;
      best = &lane;
    }
  }
  if (best == nullptr) {
    return {driverSpeeds, 0.0};
  }

  const double centeringVy =
      util::Limit(-kCenteringKP * (py - best->centerY), -kMaxCenteringSpeed, kMaxCenteringSpeed);
  const double vy = util::Interpolate(vyIn, centeringVy, bestWeight);

  double omega = driverSpeeds.omega.value();
  if (std::abs(omega) < kDriverTurnOverrideRate) {
    // The robot is square, so any multiple of 90 degrees fits equally well. Only turn it if it
    // would not clear the walls where it is (near a diagonal).
    const double heading = pose.Rotation().Radians().value();
    const double footprintHalf =
        kRobotHalfLength * (std::abs(std::cos(heading)) + std::abs(std::sin(heading)));
    const double slack = kLaneHalfWidth - footprintHalf;
    const double snapWeight = 1.0 - Smoothstep((slack - kHeadingFullSnapSlack) /
                                               (kHeadingFreeSlack - kHeadingFullSnapSlack));
    // Java Math.round: round half up
    const double target =
        std::floor(heading / (std::numbers::pi / 2.0) + 0.5) * (std::numbers::pi / 2.0);
    const double squareOmega =
        util::Limit(kHeadingKP * frc::AngleModulus(units::radian_t{target - heading}).value(),
                    -swerve_constants::MaxAngularRate, swerve_constants::MaxAngularRate);
    omega = util::Interpolate(omega, squareOmega, bestWeight * snapWeight);
  }

  return {frc::ChassisSpeeds{driverSpeeds.vx, units::meters_per_second_t{vy},
                             units::radians_per_second_t{omega}},
          bestWeight};
}

std::array<double, 2> LaneCenterYs() { return {kRightLaneY, kLeftLaneY}; }

std::array<double, 2> TrenchCenterXs() {
  return {fc::lines_vertical::hubCenter, fc::lines_vertical::oppHubCenter};
}

}  // namespace trench_assist
