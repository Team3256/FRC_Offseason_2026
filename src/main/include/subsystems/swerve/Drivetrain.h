// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <array>
#include <functional>
#include <memory>
#include <utility>

#include <frc/Notifier.h>
#include <frc/RobotBase.h>
#include <frc/controller/PIDController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <units/time.h>

#include "logging/Logger.h"
#include "subsystems/swerve/generated/TunerConstants.h"

class ShotCalculator;

namespace sim {
class DrivetrainSim;
}

namespace subsystems {

/**
 * The Phoenix 6 swerve drivetrain. Nothing owns it through a scheduler: each loop, RobotContainer
 * picks one control (driver, aim, autonomous) and calls exactly one of the drive methods below.
 *
 * Loop-time note: SwerveDrivetrain::GetState() copies the whole state (including three vectors)
 * under a lock. Robot code reads the pose many times per loop (every trigger condition), so this
 * class keeps one copy per loop: State() returns it, refreshing only after
 * InvalidateStateCache() (called at the start of each robot loop) or anything that moves the pose
 * estimate (pose reset, vision measurement).
 */
class Drivetrain : public TunerConstants::TunerSwerveDrivetrain {
 public:
  using SwerveDriveState = TunerConstants::TunerSwerveDrivetrain::SwerveDriveState;

  template <typename... Arg>
  explicit Drivetrain(Arg&&... args)
      : TunerConstants::TunerSwerveDrivetrain{std::forward<Arg>(args)...} {
    Init();
  }

  ~Drivetrain() override;

  /** Cached drivetrain state, at most one GetState() copy per loop. */
  const SwerveDriveState& State();
  const frc::Pose2d& Pose() { return State().Pose; }
  void InvalidateStateCache() { m_stateValid = false; }

  // Drive methods: call one per loop.
  /** PID straight to {@code target} (blue-origin field frame). */
  void PidToPose(const frc::Pose2d& target);
  /** Points the robot at the shot calculator's drive angle while translating at vx/vy (field). */
  void RotateToLookahead(const ShotCalculator& shotCalculator, double vx, double vy);
  /**
   * Wheel radius characterization (omegaDirection is 1 or -1): spins in place and estimates the
   * effective wheel radius. Starts over if it was not running the loop before; call
   * EndWheelRadiusCharacterization() when done to stop and print the result.
   */
  void WheelRadiusCharacterization(double omegaDirection);
  void EndWheelRadiusCharacterization();

  /**
   * Applies trench assist to the driver's requested speeds (operator-perspective frame, what a
   * FieldCentric request takes), returning speeds in the same frame.
   */
  frc::ChassisSpeeds ApplyTrenchAssist(const frc::ChassisSpeeds& operatorSpeeds);

  frc::ChassisSpeeds GetFieldRelativeSpeeds();
  /** Blue-origin field speeds from Mayhem; request storage lives as long as the drivetrain. */
  void ApplyMayhemSpeeds(frc::ChassisSpeeds speeds);

  /** Call once per loop before any drive method: operator perspective and logging. */
  void Periodic();

  /**
   * Phoenix's SetControl. In simulation the request goes to sim::DrivetrainSim, which applies it
   * every 4 ms of simulated time (Phoenix's odometry thread cannot follow stepped time).
   */
  template <typename Request>
  void SetControl(Request&& request) {
    if (m_sim) {
      SimSetControl(request);
    } else {
      TunerSwerveDrivetrain::SetControl(std::forward<Request>(request));
    }
  }
  void RegisterTelemetry(std::function<void(SwerveDriveState const&)> telemetryFunction) override;

  void ResetPose(frc::Pose2d const& pose) override;
  void SeedFieldCentric(frc::Rotation2d const& rotation = frc::Rotation2d{}) override;

  /** Adds a vision measurement. The timestamp is in the FPGA timebase (converted here). */
  void AddVisionMeasurement(frc::Pose2d visionRobotPose, units::second_t timestamp) override;
  void AddVisionMeasurement(frc::Pose2d visionRobotPose, units::second_t timestamp,
                            std::array<double, 3> visionMeasurementStdDevs) override;

  /** Steps the swerve simulation (normally done by the 5 ms sim thread). */
  void StepSimulation(units::second_t dt);

  /**
   * Stops the odometry and simulation threads and parks the drivetrain on a request it owns.
   * Phoenix's SetControl(request&) keeps a reference that the odometry thread re-applies at
   * 250 Hz, so this must run before any request object owned elsewhere (RobotContainer, the auto
   * factory) is destroyed. Called by ~RobotContainer and by the destructor.
   */
  void Shutdown();

 private:
  ctre::phoenix6::swerve::requests::ApplyFieldSpeeds m_suiteRequest;
  void Init();
  double GyroYawRads();
  double ModulePosition(size_t i);
  void StartSimThread();

  static constexpr units::second_t kSimLoopPeriod = 5_ms;
  std::unique_ptr<frc::Notifier> m_simNotifier;
  void SimSetControl(const ctre::phoenix6::swerve::requests::SwerveRequest& request);
  /** Simulation only: the drivetrain on the physics engine (see sim/DrivetrainSim.h). */
  std::unique_ptr<sim::DrivetrainSim> m_sim;
  std::function<void(SwerveDriveState const&)> m_simTelemetry;
  units::second_t m_lastSimTime;

  SwerveDriveState m_state{};
  bool m_stateValid = false;

  // A drive method that aims ran since the last Periodic() (logged as AutoAlign/Running).
  bool m_aligning = false;

  // wheel radius characterization
  bool m_characterizing = false;
  bool m_characterizedLastLoop = false;
  frc::SlewRateLimiter<units::scalar> m_omegaLimiter{0.5 / 1_s};
  ctre::phoenix6::swerve::requests::RobotCentric m_characterizationRequest =
      ctre::phoenix6::swerve::requests::RobotCentric{}.WithDriveRequestType(
          ctre::phoenix6::swerve::DriveRequestType::OpenLoopVoltage);
  double m_lastGyroYawRads = 0;
  double m_accumGyroYawRads = 0;
  double m_averageWheelPosition = 0;
  std::array<double, 4> m_startWheelPositions{};
  double m_currentEffectiveWheelRadius = 0;

  frc::ChassisSpeeds m_previousSpeeds;
  units::second_t m_previousTime = 0_s;

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  static inline const frc::Rotation2d kBlueAlliancePerspectiveRotation{0_deg};
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  static inline const frc::Rotation2d kRedAlliancePerspectiveRotation{180_deg};
  /* Keep track if we've ever applied the operator perspective before or not */
  bool m_hasAppliedOperatorPerspective = false;

  frc::PIDController m_xController{8.0, 0.0, 0};
  frc::PIDController m_yController{8.0, 0.0, 0};
  frc::PIDController m_headingController{5, 0, 0};

  ctre::phoenix6::swerve::requests::Idle m_idleRequest;
  bool m_shutDown = false;

  ctre::phoenix6::swerve::requests::ApplyFieldSpeeds m_pathApplyFieldSpeeds =
      ctre::phoenix6::swerve::requests::ApplyFieldSpeeds{}.WithDriveRequestType(
          ctre::phoenix6::swerve::DriveRequestType::Velocity);

  logging::Output<frc::Pose2d> m_autoAlignTargetLog{"AutoAlign/Target"};
  logging::Output<bool> m_autoAlignRunningLog{"AutoAlign/Running"};
  logging::Output<frc::Rotation2d> m_autoAlignTargetAngleLog{"AutoAlign/TargetAngle"};
  logging::Output<double> m_trenchWeightLog{"TrenchAssist/Weight"};
  logging::Output<bool> m_trenchActiveLog{"TrenchAssist/Active"};
  logging::Output<double> m_lastGyroYawLog{"Drivetrain/lastGyroYawRads"};
  logging::Output<double> m_accumGyroYawLog{"Drivetrain/accumGyroYawRads"};
  logging::Output<double> m_averageWheelPositionLog{"Drivetrain/averageWheelPosition"};
  logging::Output<std::vector<double>> m_startWheelPositionsLog{"Drivetrain/startWheelPositions"};
  logging::Output<double> m_effectiveWheelRadiusLog{"Drivetrain/currentEffectiveWheelRadius"};
};

}  // namespace subsystems
