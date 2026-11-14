// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <array>
#include <functional>
#include <optional>

#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <ctre/phoenix6/swerve/SwerveDrivetrain.hpp>

#include "mayhem/sim/swerve_requests.hpp"
#include "sim/PhysicsWorld.h"

namespace sim {

/**
 * The drivetrain in simulation. Phoenix's SwerveDrivetrain runs its requests and odometry on a
 * 250 Hz thread that reads status signals on the wall clock, which cannot follow stepped time, so
 * in simulation Drivetrain hands its requests here instead. Every 4 ms of simulated time this
 * updates the pose estimate from the simulated modules and gyro, then applies the current request
 * with Mayhem's SwerveRequestEmulator (Phoenix's request and module math) to the simulated modules.
 * Vision measurements go into the same WPILib pose estimator Phoenix uses, with Phoenix's default
 * standard deviations.
 */
class DrivetrainSim : public DriveHooks {
 public:
  using SwerveDriveState = ctre::phoenix6::swerve::impl::SwerveDrivetrainImpl::SwerveDriveState;

  explicit DrivetrainSim(std::function<frc::Rotation2d()> operatorForward);
  ~DrivetrainSim() override;

  /** The request Phoenix would re-apply every 4 ms (converted now; requests are plain data). */
  void SetControl(const ctre::phoenix6::swerve::requests::SwerveRequest& request);
  void ControlTick(double fpgaSeconds) override;

  const SwerveDriveState& State() const { return m_state; }
  void ResetPose(const frc::Pose2d& pose);
  void SeedFieldCentric(const frc::Rotation2d& rotation);
  void AddVisionMeasurement(const frc::Pose2d& pose, double fpgaSeconds,
                            std::optional<std::array<double, 3>> stdDevs);

 private:
  frc::Rotation2d Gyro() const;
  wpi::array<frc::SwerveModulePosition, 4> Positions() const;
  void UpdateState(double fpgaSeconds);

  std::function<frc::Rotation2d()> m_operatorForward;
  frc::SwerveDriveKinematics<4> m_kinematics;
  frc::SwerveDrivePoseEstimator<4> m_estimator;
  mayhem::sim::swerve::SwerveRequestEmulator m_emulator;
  mayhem::sim::swerve::SwerveRequest m_request = mayhem::sim::swerve::Idle{};
  SwerveDriveState m_state{};
};

}  // namespace sim
