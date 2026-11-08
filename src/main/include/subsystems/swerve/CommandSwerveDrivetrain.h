// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <array>
#include <concepts>
#include <functional>
#include <memory>
#include <type_traits>
#include <utility>

#include <frc/Notifier.h>
#include <frc/RobotBase.h>
#include <frc/controller/PIDController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <units/time.h>

#include "logging/Logger.h"
#include "subsystems/swerve/generated/TunerConstants.h"

class ShotCalculator;

namespace subsystems {

/**
 * The Phoenix 6 swerve drivetrain as a command-based subsystem.
 *
 * Loop-time note: SwerveDrivetrain::GetState() copies the whole state (including three vectors)
 * under a lock. Robot code reads the pose many times per loop (every trigger condition), so this
 * class keeps one copy per loop: State() returns it, refreshing only after
 * InvalidateStateCache() (called at the start of each robot loop) or anything that moves the pose
 * estimate (pose reset, vision measurement).
 */
class CommandSwerveDrivetrain : public frc2::SubsystemBase,
                                public TunerConstants::TunerSwerveDrivetrain {
 public:
  using SwerveDriveState = TunerConstants::TunerSwerveDrivetrain::SwerveDriveState;

  template <typename... Arg>
  explicit CommandSwerveDrivetrain(Arg&&... args)
      : frc2::SubsystemBase("CommandSwerveDrivetrain"),
        TunerConstants::TunerSwerveDrivetrain{std::forward<Arg>(args)...} {
    Init();
  }

  ~CommandSwerveDrivetrain() override;

  /** Cached drivetrain state, at most one GetState() copy per loop. */
  const SwerveDriveState& State();
  const frc::Pose2d& Pose() { return State().Pose; }
  void InvalidateStateCache() { m_stateValid = false; }

  /** Returns a command that applies the specified control request to this swerve drivetrain. */
  template <typename RequestSupplier>
    requires std::is_lvalue_reference_v<std::invoke_result_t<RequestSupplier>> &&
             requires(RequestSupplier req, TunerSwerveDrivetrain& drive) {
               drive.SetControl(req());
             }
  frc2::CommandPtr ApplyRequest(RequestSupplier request) {
    return Run([this, request = std::move(request)] { return SetControl(request()); });
  }

  frc2::CommandPtr PidToPose(std::function<frc::Pose2d()> target);

  /** Points the robot at the shot calculator's drive angle while translating at vx/vy. */
  frc2::CommandPtr RotateToLookahead(ShotCalculator& shotCalculator,
                                     std::function<double()> vxSupplier,
                                     std::function<double()> vySupplier);

  /**
   * Applies trench assist to the driver's requested speeds (operator-perspective frame, what a
   * FieldCentric request takes), returning speeds in the same frame.
   */
  frc::ChassisSpeeds ApplyTrenchAssist(const frc::ChassisSpeeds& operatorSpeeds);

  frc::ChassisSpeeds GetFieldRelativeSpeeds();

  /** Runs the wheel radius characterization routine (omegaDirection is 1 or -1). */
  frc2::CommandPtr WheelRadiusCharacterization(double omegaDirection);

  void Periodic() override;

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
  void Init();
  void StartSimThread();

  static constexpr units::second_t kSimLoopPeriod = 5_ms;
  std::unique_ptr<frc::Notifier> m_simNotifier;
  units::second_t m_lastSimTime;

  SwerveDriveState m_state{};
  bool m_stateValid = false;

  // wheel radius characterization
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
  logging::Output<double> m_lastGyroYawLog{"CommandSwerveDrivetrain/lastGyroYawRads"};
  logging::Output<double> m_accumGyroYawLog{"CommandSwerveDrivetrain/accumGyroYawRads"};
  logging::Output<double> m_averageWheelPositionLog{"CommandSwerveDrivetrain/averageWheelPosition"};
  logging::Output<std::vector<double>> m_startWheelPositionsLog{
      "CommandSwerveDrivetrain/startWheelPositions"};
  logging::Output<double> m_effectiveWheelRadiusLog{
      "CommandSwerveDrivetrain/currentEffectiveWheelRadius"};
};

}  // namespace subsystems
