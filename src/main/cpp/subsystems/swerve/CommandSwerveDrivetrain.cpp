// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "subsystems/swerve/CommandSwerveDrivetrain.h"

#include <cmath>
#include <cstdio>
#include <numbers>
#include <string>

#include <frc/DriverStation.h>
#include <frc/MathUtil.h>
#include <frc/RobotController.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/geometry/Translation2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <ctre/phoenix6/Utils.hpp>

#include "logging/LoggedTracer.h"
#include "subsystems/sotm/ShotCalculator.h"
#include "subsystems/swerve/SwerveConstants.h"
#include "subsystems/swerve/TrenchAssist.h"
#include "utils/DSReport.h"
#include "utils/PhoenixUtil.h"

namespace subsystems {

using namespace ctre::phoenix6;

void CommandSwerveDrivetrain::Init() {
  // Publish the trench assist switch so it shows up on the dashboard (read in ApplyTrenchAssist)
  frc::SmartDashboard::SetDefaultBoolean("TrenchAssist/Enabled",
                                         swerve_constants::trench_assist::kUseTrenchAssist);
  // Dashboard alerts for any swerve device that drops off the CANivore.
  static constexpr std::array<const char*, 4> kModuleNames = {"front left", "front right",
                                                              "back left", "back right"};
  for (size_t i = 0; i < GetModules().size() && i < kModuleNames.size(); i++) {
    const std::string module = std::string{"Swerve "} + kModuleNames[i];
    phoenix_util::WatchDevice(GetModule(i).GetDriveMotor(), module + " drive motor");
    phoenix_util::WatchDevice(GetModule(i).GetSteerMotor(), module + " steer motor");
    phoenix_util::WatchDevice(GetModule(i).GetEncoder(), module + " CANcoder");
  }
  phoenix_util::WatchDevice(GetPigeon2(), "Pigeon 2");
  if (frc::RobotBase::IsSimulation()) {
    StartSimThread();
  }
}

CommandSwerveDrivetrain::~CommandSwerveDrivetrain() { Shutdown(); }

void CommandSwerveDrivetrain::Shutdown() {
  if (m_shutDown) {
    return;
  }
  m_shutDown = true;
  if (m_simNotifier) {
    m_simNotifier->Stop();
  }
  GetOdometryThread().Stop();
  SetControl(m_idleRequest);
}

const CommandSwerveDrivetrain::SwerveDriveState& CommandSwerveDrivetrain::State() {
  if (!m_stateValid) {
    m_state = GetState();
    m_stateValid = true;
  }
  return m_state;
}

frc::ChassisSpeeds CommandSwerveDrivetrain::GetFieldRelativeSpeeds() {
  const auto& st = State();
  return frc::ChassisSpeeds::FromRobotRelativeSpeeds(st.Speeds, st.Pose.Rotation());
}

void CommandSwerveDrivetrain::ResetPose(frc::Pose2d const& pose) {
  TunerSwerveDrivetrain::ResetPose(pose);
  InvalidateStateCache();
}

void CommandSwerveDrivetrain::SeedFieldCentric(frc::Rotation2d const& rotation) {
  TunerSwerveDrivetrain::SeedFieldCentric(rotation);
  InvalidateStateCache();
}

void CommandSwerveDrivetrain::AddVisionMeasurement(frc::Pose2d visionRobotPose,
                                                   units::second_t timestamp) {
  TunerSwerveDrivetrain::AddVisionMeasurement(std::move(visionRobotPose),
                                              utils::FPGAToCurrentTime(timestamp));
  InvalidateStateCache();
}

void CommandSwerveDrivetrain::AddVisionMeasurement(frc::Pose2d visionRobotPose,
                                                   units::second_t timestamp,
                                                   std::array<double, 3> visionMeasurementStdDevs) {
  TunerSwerveDrivetrain::AddVisionMeasurement(
      std::move(visionRobotPose), utils::FPGAToCurrentTime(timestamp), visionMeasurementStdDevs);
  InvalidateStateCache();
}

frc2::CommandPtr CommandSwerveDrivetrain::PidToPose(std::function<frc::Pose2d()> target) {
  if (!target) {
    std::printf("**** pidToPose disabled because of null target\n");
    return frc2::cmd::None();
  }
  m_xController.SetTolerance(.02);
  m_yController.SetTolerance(.02);
  m_headingController.SetTolerance(0.1 * std::numbers::pi / 180.0);
  return Run([this, target = std::move(target)] {
           const frc::Pose2d t = target();
           m_autoAlignTargetLog.Set(t);
           m_autoAlignRunningLog.Set(true);
           const auto& pose = State().Pose;
           SetControl(m_pathApplyFieldSpeeds.WithSpeeds(frc::ChassisSpeeds{
               units::meters_per_second_t{m_xController.Calculate(pose.X().value(), t.X().value())},
               units::meters_per_second_t{m_yController.Calculate(pose.Y().value(), t.Y().value())},
               units::radians_per_second_t{m_headingController.Calculate(
                   pose.Rotation().Radians().value(), t.Rotation().Radians().value())}}));
         })
      .FinallyDo([this] { m_autoAlignRunningLog.Set(false); });
}

frc::ChassisSpeeds CommandSwerveDrivetrain::ApplyTrenchAssist(
    const frc::ChassisSpeeds& operatorSpeeds) {
  // Only for the driver: never while a path is being followed in auto, and it can be switched
  // off from the dashboard ("TrenchAssist/Enabled") or with kUseTrenchAssist.
  if (!frc::DriverStation::IsTeleopEnabled() ||
      !frc::SmartDashboard::GetBoolean("TrenchAssist/Enabled",
                                       swerve_constants::trench_assist::kUseTrenchAssist)) {
    return operatorSpeeds;
  }
  // Operator perspective -> blue-origin field frame, which the pose and field geometry use.
  const frc::Rotation2d operatorForward = GetOperatorForwardDirection();
  const frc::Translation2d fieldVelocity = frc::Translation2d{
      units::meter_t{operatorSpeeds.vx.value()},
      units::meter_t{
          operatorSpeeds.vy.value()}}.RotateBy(operatorForward);
  const auto result = trench_assist::Apply(
      State().Pose, frc::ChassisSpeeds{units::meters_per_second_t{fieldVelocity.X().value()},
                                       units::meters_per_second_t{fieldVelocity.Y().value()},
                                       operatorSpeeds.omega});

  m_trenchWeightLog.Set(result.weight);
  m_trenchActiveLog.Set(result.weight > 0.0);
  if (result.weight <= 0.0) {
    return operatorSpeeds;
  }
  const frc::Translation2d assisted = frc::Translation2d{
      units::meter_t{result.speeds.vx.value()},
      units::meter_t{
          result.speeds.vy.value()}}.RotateBy(-operatorForward);
  return frc::ChassisSpeeds{units::meters_per_second_t{assisted.X().value()},
                            units::meters_per_second_t{assisted.Y().value()}, result.speeds.omega};
}

frc2::CommandPtr CommandSwerveDrivetrain::RotateToLookahead(ShotCalculator& shotCalculator,
                                                            std::function<double()> vxSupplier,
                                                            std::function<double()> vySupplier) {
  m_headingController.EnableContinuousInput(-std::numbers::pi, std::numbers::pi);
  m_headingController.SetTolerance(5.0 * std::numbers::pi / 180.0);
  return Run([this, &shotCalculator, vxSupplier = std::move(vxSupplier),
              vySupplier = std::move(vySupplier)] {
           const frc::Rotation2d driveAngle = shotCalculator.GetDriveAngle();
           const double omega = m_headingController.Calculate(
               State().Pose.Rotation().Radians().value(), driveAngle.Radians().value());
           m_autoAlignTargetAngleLog.Set(driveAngle);
           m_autoAlignRunningLog.Set(true);
           SetControl(m_pathApplyFieldSpeeds.WithSpeeds(frc::ChassisSpeeds{
               units::meters_per_second_t{vxSupplier()}, units::meters_per_second_t{vySupplier()},
               units::radians_per_second_t{omega}}));
         })
      .FinallyDo([this] { m_autoAlignRunningLog.Set(false); });
}

void CommandSwerveDrivetrain::Periodic() {
  /*
   * Periodically try to apply the operator perspective. If we haven't applied it before, apply it
   * regardless of DS state (corrects the perspective if the robot code restarts mid-match).
   * Otherwise only while disabled, so driving behavior doesn't change until an explicit disable.
   */
  if (!m_hasAppliedOperatorPerspective || frc::DriverStation::IsDisabled()) {
    if (auto allianceColor = frc::DriverStation::GetAlliance()) {
      SetOperatorPerspectiveForward(*allianceColor == frc::DriverStation::Alliance::kRed
                                        ? kRedAlliancePerspectiveRotation
                                        : kBlueAlliancePerspectiveRotation);
      m_hasAppliedOperatorPerspective = true;
    }
  }

  const auto& st = State();
  m_previousSpeeds = frc::ChassisSpeeds::FromRobotRelativeSpeeds(st.Speeds, st.RawHeading);
  m_previousTime = st.Timestamp;

  m_lastGyroYawLog.Set(m_lastGyroYawRads);
  m_accumGyroYawLog.Set(m_accumGyroYawRads);
  m_averageWheelPositionLog.Set(m_averageWheelPosition);
  m_startWheelPositionsLog.Set(std::span<const double>{m_startWheelPositions});
  m_effectiveWheelRadiusLog.Set(m_currentEffectiveWheelRadius);

  logging::LoggedTracer::Record("CommandSwerveDrivetrain");
}

void CommandSwerveDrivetrain::StepSimulation(units::second_t dt) {
  /* use the measured time delta, get battery voltage from WPILib */
  UpdateSimState(dt, frc::RobotController::GetBatteryVoltage());
}

void CommandSwerveDrivetrain::StartSimThread() {
  m_lastSimTime = utils::GetCurrentTime();
  /* Run simulation at a faster rate so PID gains behave more reasonably */
  m_simNotifier = std::make_unique<frc::Notifier>([this] {
    const units::second_t currentTime = utils::GetCurrentTime();
    const auto deltaTime = currentTime - m_lastSimTime;
    m_lastSimTime = currentTime;
    StepSimulation(deltaTime);
  });
  m_simNotifier->StartPeriodic(kSimLoopPeriod);
}

frc2::CommandPtr CommandSwerveDrivetrain::WheelRadiusCharacterization(double omegaDirection) {
  /* wheel radius characterization schtuffs */
  auto gyroYawRads = [this] {
    return GetPigeon2().GetYaw().GetValueAsDouble() * std::numbers::pi / 180.0;
  };
  auto omegaLimiter = std::make_shared<frc::SlewRateLimiter<units::scalar>>(0.5 / 1_s);
  auto characterizationReq = std::make_shared<swerve::requests::RobotCentric>(
      swerve::requests::RobotCentric{}.WithDriveRequestType(
          swerve::DriveRequestType::OpenLoopVoltage));
  constexpr double kCharacterizationSpeed = 1.5;

  auto modulePosition = [this](size_t i) {
    return GetModule(i).GetPosition(true).distance.value() *
           TunerConstants::kDriveRotationsPerMeter;
  };

  auto initialize = RunOnce([this, gyroYawRads, omegaLimiter, modulePosition] {
    m_lastGyroYawRads = gyroYawRads();
    m_accumGyroYawRads = 0;
    m_currentEffectiveWheelRadius = 0;
    m_averageWheelPosition = 0;
    for (size_t i = 0; i < GetModules().size(); i++) {
      m_startWheelPositions[i] = modulePosition(i);
    }
    omegaLimiter->Reset(0);
  });

  auto executeEnd = RunEnd(
      [this, gyroYawRads, omegaLimiter, characterizationReq, modulePosition, omegaDirection] {
        SetControl(characterizationReq->WithRotationalRate(units::radians_per_second_t{
            omegaLimiter->Calculate(kCharacterizationSpeed * omegaDirection)}));
        m_accumGyroYawRads +=
            frc::AngleModulus(units::radian_t{gyroYawRads() - m_lastGyroYawRads}).value();
        m_lastGyroYawRads = gyroYawRads();
        m_averageWheelPosition = 0;
        for (size_t i = 0; i < GetModules().size(); i++) {
          m_averageWheelPosition += std::abs(modulePosition(i) - m_startWheelPositions[i]);
        }
        m_averageWheelPosition = m_averageWheelPosition / 4.0;
        m_currentEffectiveWheelRadius =
            (m_accumGyroYawRads * TunerConstants::kDriveRadius) / m_averageWheelPosition;
        std::printf("Average Wheel Position: %f\n", m_averageWheelPosition);
      },
      [this, characterizationReq] {
        SetControl(characterizationReq->WithRotationalRate(0_rad_per_s));
        if (std::abs(m_accumGyroYawRads) <= std::numbers::pi * 2.0) {
          std::printf("not enough data for characterization %f\navgWheelPos: %fradians\n",
                      m_accumGyroYawRads, m_averageWheelPosition);
        } else {
          std::printf(
              "effective wheel radius: %f inches\naccumGryoYawRads: %fradians\navgWheelPos: "
              "%fradians\n",
              m_currentEffectiveWheelRadius, m_accumGyroYawRads, m_averageWheelPosition);
        }
      });

  return frc2::cmd::Sequence(std::move(initialize), std::move(executeEnd));
}

}  // namespace subsystems

subsystems::CommandSwerveDrivetrain TunerConstants::CreateDrivetrain() {
  return subsystems::CommandSwerveDrivetrain{DrivetrainConstants, FrontLeft, FrontRight, BackLeft,
                                             BackRight};
}
