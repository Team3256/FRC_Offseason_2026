// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "subsystems/sotm/ShotCalculator.h"

#include <cmath>
#include <numbers>

#include <frc/geometry/Twist2d.h>

#include "FieldConstants.h"

namespace {
constexpr util::InterpolatingMap<10> kTimeOfFlightHub{{{
    {1.322, 0.88},
    {1.826, 1.0},
    {2.247, 1.01},
    {2.765, 1.1},
    {3.135, 1.0},
    {3.501, 1.02},
    {3.901, 1.09},
    {4.017, 1.12},
    {4.560, 1.2},
    //          {4.857, 1.27},
    //          {5.616, 1.2},
    {5.837, 1.27},
}}};
constexpr util::InterpolatingMap<10> kTimeOfFlightFeed{{{
    {4.38, 1.12},
    {6.11, 1.2},
    {8.0, 1.3},
    {9.42, 1.3},
    {10.8, 1.55},
    {12.834, 1.7},
    // fake data
    {14.0, 1.8},
    {15.0, 1.9},
    {16.0, 2.0},
    {17.0, 2.1},
}}};
}  // namespace

const util::InterpolatingMap<10>& ShotCalculator::TimeOfFlightHub() { return kTimeOfFlightHub; }
const util::InterpolatingMap<10>& ShotCalculator::TimeOfFlightFeed() { return kTimeOfFlightFeed; }

ShotCalculator::ShotCalculator(std::function<frc::Pose2d()> robotPoseSupplier,
                               std::function<frc::ChassisSpeeds()> robotVelocitySupplier,
                               frc::Transform2d robotToShooter)
    : m_robotPoseSupplier{std::move(robotPoseSupplier)},
      m_robotVelocitySupplier{std::move(robotVelocitySupplier)},
      m_robotToShooter{robotToShooter},
      m_target{field_constants::hub::kTarget2d} {}

void ShotCalculator::Periodic() {
  if (!m_robotPoseSupplier || !m_robotVelocitySupplier) {
    return;
  }

  const auto& timeOfFlightMap = (m_target == field_constants::hub::kTarget2d ||
                                 m_target == field_constants::hub::kOppTarget2d)
                                    ? kTimeOfFlightHub
                                    : kTimeOfFlightFeed;

  const frc::Pose2d robotPose = m_robotPoseSupplier();
  const frc::ChassisSpeeds robotVelocity = m_robotVelocitySupplier();
  const double vx = robotVelocity.vx.value();
  const double vy = robotVelocity.vy.value();
  const double omega = robotVelocity.omega.value();

  // Phase delay
  const frc::Pose2d estimatedPose =
      robotPose.Exp(frc::Twist2d{units::meter_t{vx * kPhaseDelay}, units::meter_t{vy * kPhaseDelay},
                                 units::radian_t{omega * kPhaseDelay}});
  const frc::Pose2d shooterPosition = estimatedPose.TransformBy(m_robotToShooter);

  // Distance from shooter to target
  const double shooterToTargetDistance = m_target.Distance(shooterPosition.Translation()).value();

  // Field-relative shooter velocity
  const double robotAngle = estimatedPose.Rotation().Radians().value();
  const double rx = m_robotToShooter.X().value();
  const double ry = m_robotToShooter.Y().value();
  const double shooterVelocityX =
      vx + omega * (ry * std::cos(robotAngle) - rx * std::sin(robotAngle));
  const double shooterVelocityY =
      vy + omega * (rx * std::cos(robotAngle) - ry * std::sin(robotAngle));

  const double sx = shooterPosition.X().value();
  const double sy = shooterPosition.Y().value();
  const double tx = m_target.X().value();
  const double ty = m_target.Y().value();
  double lx = sx;
  double ly = sy;
  double currentDistance = shooterToTargetDistance;  // starting estimate
  for (int i = 0; i < kIterations; i++) {
    const double timeOfFlight = timeOfFlightMap.Get(currentDistance) - kFunnyNum;
    lx = sx + shooterVelocityX * timeOfFlight;
    ly = sy + shooterVelocityY * timeOfFlight;
    currentDistance = std::hypot(tx - lx, ty - ly);
  }
  m_lookaheadPose = frc::Pose2d{units::meter_t{lx}, units::meter_t{ly}, shooterPosition.Rotation()};
  m_hasLookahead = true;

  m_lookaheadLog.Set(m_lookaheadPose);
  m_distanceLog.Set(shooterToTargetDistance);

  m_driveAngle = frc::Rotation2d{units::radian_t{std::atan2(ty - ly, tx - lx)}}.RotateBy(
      frc::Rotation2d{units::radian_t{std::numbers::pi}});
}

double ShotCalculator::GetDistance() const {
  if (!m_hasLookahead) {
    // Before the first Periodic() (the Java version would have thrown here): use the robot pose.
    return m_robotPoseSupplier ? m_robotPoseSupplier()
                                     .TransformBy(m_robotToShooter)
                                     .Translation()
                                     .Distance(m_target)
                                     .value()
                               : 0.0;
  }
  return m_lookaheadPose.Translation().Distance(m_target).value();
}

bool ShotCalculator::IsAimed(const frc::Rotation2d& robotHeading, const frc::Rotation2d& driveAngle,
                             double toleranceRad) {
  return std::abs((robotHeading - driveAngle).Radians().value()) <= toleranceRad;
}
