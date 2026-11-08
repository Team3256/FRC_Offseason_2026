// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <functional>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Transform2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>

#include "logging/Logger.h"
#include "utils/InterpolatingMap.h"

/**
 * Shoot-on-the-move: predicts where the shooter will be when the ball lands (time of flight from a
 * lookup table, iterated to convergence) and from that the distance to shoot at and the angle to
 * face.
 */
class ShotCalculator {
 public:
  ShotCalculator(std::function<frc::Pose2d()> robotPoseSupplier,
                 std::function<frc::ChassisSpeeds()> robotVelocitySupplier,
                 frc::Transform2d robotToShooter);

  /** Call once per loop. */
  void Periodic();

  frc::Rotation2d GetDriveAngle() const { return m_driveAngle; }
  frc::Pose2d GetLookaheadPose() const { return m_lookaheadPose; }
  /** Distance from the lookahead pose to the target [m]. */
  double GetDistance() const;
  void SetTarget(const frc::Translation2d& target) { m_target = target; }
  const frc::Translation2d& GetTarget() const { return m_target; }
  const std::function<frc::Pose2d()>& GetRobotPoseSupplier() const { return m_robotPoseSupplier; }

  static bool IsAimed(const frc::Rotation2d& robotHeading, const frc::Rotation2d& driveAngle,
                      double toleranceRad);

  static constexpr double kPhaseDelay = 0.05;
  static constexpr double kFunnyNum = 0;
  static constexpr int kIterations = 20;

  static const util::InterpolatingMap<10>& TimeOfFlightHub();
  static const util::InterpolatingMap<10>& TimeOfFlightFeed();

 private:
  std::function<frc::Pose2d()> m_robotPoseSupplier;
  std::function<frc::ChassisSpeeds()> m_robotVelocitySupplier;
  frc::Transform2d m_robotToShooter;
  frc::Pose2d m_lookaheadPose;
  bool m_hasLookahead = false;
  frc::Translation2d m_target;
  frc::Rotation2d m_driveAngle;

  logging::Output<frc::Pose2d> m_lookaheadLog{"ShotCalculator/LookaheadPose"};
  logging::Output<double> m_distanceLog{"ShotCalculator/ShooterToTargetDistance"};
};
