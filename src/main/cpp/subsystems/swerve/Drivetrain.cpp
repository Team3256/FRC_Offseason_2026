// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "subsystems/swerve/Drivetrain.h"

#include <cmath>
#include <cstdio>
#include <numbers>
#include <string>

#include <frc/DriverStation.h>
#include <frc/MathUtil.h>
#include <frc/RobotController.h>
#include <frc/geometry/Translation2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <ctre/phoenix6/Utils.hpp>

#include "logging/LoggedTracer.h"
#include "sim/DrivetrainSim.h"
#include "subsystems/sotm/ShotCalculator.h"
#include "subsystems/swerve/SwerveConstants.h"
#include "subsystems/swerve/TrenchAssist.h"
#include "utils/DSReport.h"
#include "utils/PhoenixUtil.h"

namespace subsystems {

using namespace ctre::phoenix6;

void Drivetrain::Init() {
  m_headingController.EnableContinuousInput(-std::numbers::pi, std::numbers::pi);
  using namespace ctre::phoenix6::swerve;
  m_suiteRequest.WithDriveRequestType(DriveRequestType::Velocity)
      .WithForwardPerspective(requests::ForwardPerspectiveValue::BlueAlliance)
      .WithDesaturateWheelSpeeds(true);
  m_suiteRequest.WheelForceFeedforwardsX.assign(4, 0_N);
  m_suiteRequest.WheelForceFeedforwardsY.assign(4, 0_N);
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
    // The physics engine drives the simulated drivetrain on simulated time; Phoenix's odometry
    // thread and device simulation would run on the wall clock (see sim/DrivetrainSim.h).
    GetOdometryThread().Stop();
    m_sim =
        std::make_unique<::sim::DrivetrainSim>([this] { return GetOperatorForwardDirection(); });
  }
}

Drivetrain::~Drivetrain() { Shutdown(); }

void Drivetrain::Shutdown() {
  if (m_shutDown) {
    return;
  }
  m_shutDown = true;
  if (m_simNotifier) {
    m_simNotifier->Stop();
  }
  GetOdometryThread().Stop();
  SetControl(m_idleRequest);
  m_sim.reset();
}

const Drivetrain::SwerveDriveState& Drivetrain::State() {
  if (!m_stateValid) {
    m_state = m_sim ? m_sim->State() : GetState();
    m_stateValid = true;
  }
  return m_state;
}

void Drivetrain::SimSetControl(const swerve::requests::SwerveRequest& request) {
  m_sim->SetControl(request);
}

void Drivetrain::RegisterTelemetry(std::function<void(SwerveDriveState const&)> telemetryFunction) {
  if (m_sim) {
    m_simTelemetry = std::move(telemetryFunction);
  } else {
    TunerSwerveDrivetrain::RegisterTelemetry(std::move(telemetryFunction));
  }
}

frc::ChassisSpeeds Drivetrain::GetFieldRelativeSpeeds() {
  const auto& st = State();
  return frc::ChassisSpeeds::FromRobotRelativeSpeeds(st.Speeds, st.Pose.Rotation());
}

void Drivetrain::ApplyMayhemSpeeds(frc::ChassisSpeeds speeds) {
  m_suiteRequest.Speeds = speeds;
  SetControl(m_suiteRequest);
}

void Drivetrain::ResetPose(frc::Pose2d const& pose) {
  if (m_sim) {
    m_sim->ResetPose(pose);
  } else {
    TunerSwerveDrivetrain::ResetPose(pose);
  }
  InvalidateStateCache();
}

void Drivetrain::SeedFieldCentric(frc::Rotation2d const& rotation) {
  if (m_sim) {
    m_sim->SeedFieldCentric(rotation);
  } else {
    TunerSwerveDrivetrain::SeedFieldCentric(rotation);
  }
  InvalidateStateCache();
}

void Drivetrain::AddVisionMeasurement(frc::Pose2d visionRobotPose, units::second_t timestamp) {
  if (m_sim) {
    m_sim->AddVisionMeasurement(visionRobotPose, timestamp.value(), std::nullopt);
  } else {
    TunerSwerveDrivetrain::AddVisionMeasurement(std::move(visionRobotPose),
                                                utils::FPGAToCurrentTime(timestamp));
  }
  InvalidateStateCache();
}

void Drivetrain::AddVisionMeasurement(frc::Pose2d visionRobotPose, units::second_t timestamp,
                                      std::array<double, 3> visionMeasurementStdDevs) {
  if (m_sim) {
    m_sim->AddVisionMeasurement(visionRobotPose, timestamp.value(), visionMeasurementStdDevs);
  } else {
    TunerSwerveDrivetrain::AddVisionMeasurement(
        std::move(visionRobotPose), utils::FPGAToCurrentTime(timestamp), visionMeasurementStdDevs);
  }
  InvalidateStateCache();
}

void Drivetrain::PidToPose(const frc::Pose2d& target) {
  m_autoAlignTargetLog.Set(target);
  m_aligning = true;
  const auto& pose = State().Pose;
  SetControl(m_pathApplyFieldSpeeds.WithSpeeds(frc::ChassisSpeeds{
      units::meters_per_second_t{m_xController.Calculate(pose.X().value(), target.X().value())},
      units::meters_per_second_t{m_yController.Calculate(pose.Y().value(), target.Y().value())},
      units::radians_per_second_t{m_headingController.Calculate(
          pose.Rotation().Radians().value(), target.Rotation().Radians().value())}}));
}

frc::ChassisSpeeds Drivetrain::ApplyTrenchAssist(const frc::ChassisSpeeds& operatorSpeeds) {
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

void Drivetrain::RotateToLookahead(const ShotCalculator& shotCalculator, double vx, double vy) {
  const frc::Rotation2d driveAngle = shotCalculator.GetDriveAngle();
  const double omega = m_headingController.Calculate(State().Pose.Rotation().Radians().value(),
                                                     driveAngle.Radians().value());
  m_autoAlignTargetAngleLog.Set(driveAngle);
  m_aligning = true;
  SetControl(m_pathApplyFieldSpeeds.WithSpeeds(
      frc::ChassisSpeeds{units::meters_per_second_t{vx}, units::meters_per_second_t{vy},
                         units::radians_per_second_t{omega}}));
}

void Drivetrain::Periodic() {
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

  // A drive method that ran last loop and not since has been replaced by another control.
  m_autoAlignRunningLog.Set(m_aligning);
  m_aligning = false;
  if (m_characterizing && !m_characterizedLastLoop) {
    EndWheelRadiusCharacterization();
  }
  m_characterizedLastLoop = false;

  const auto& st = State();
  if (m_simTelemetry) {
    m_simTelemetry(st);
  }
  m_previousSpeeds = frc::ChassisSpeeds::FromRobotRelativeSpeeds(st.Speeds, st.RawHeading);
  m_previousTime = st.Timestamp;

  m_lastGyroYawLog.Set(m_lastGyroYawRads);
  m_accumGyroYawLog.Set(m_accumGyroYawRads);
  m_averageWheelPositionLog.Set(m_averageWheelPosition);
  m_startWheelPositionsLog.Set(std::span<const double>{m_startWheelPositions});
  m_effectiveWheelRadiusLog.Set(m_currentEffectiveWheelRadius);

  logging::LoggedTracer::Record("Drivetrain");
}

void Drivetrain::StepSimulation(units::second_t dt) {
  /* use the measured time delta, get battery voltage from WPILib */
  UpdateSimState(dt, frc::RobotController::GetBatteryVoltage());
}

void Drivetrain::StartSimThread() {
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

double Drivetrain::GyroYawRads() {
  return GetPigeon2().GetYaw().GetValueAsDouble() * std::numbers::pi / 180.0;
}

double Drivetrain::ModulePosition(size_t i) {
  return GetModule(i).GetPosition(true).distance.value() * TunerConstants::kDriveRotationsPerMeter;
}

void Drivetrain::WheelRadiusCharacterization(double omegaDirection) {
  /* wheel radius characterization schtuffs */
  constexpr double kCharacterizationSpeed = 1.5;
  m_characterizedLastLoop = true;
  if (!m_characterizing) {
    m_characterizing = true;
    m_lastGyroYawRads = GyroYawRads();
    m_accumGyroYawRads = 0;
    m_currentEffectiveWheelRadius = 0;
    m_averageWheelPosition = 0;
    for (size_t i = 0; i < GetModules().size(); i++) {
      m_startWheelPositions[i] = ModulePosition(i);
    }
    m_omegaLimiter.Reset(0);
    return;
  }
  SetControl(m_characterizationRequest.WithRotationalRate(units::radians_per_second_t{
      m_omegaLimiter.Calculate(kCharacterizationSpeed * omegaDirection)}));
  m_accumGyroYawRads +=
      frc::AngleModulus(units::radian_t{GyroYawRads() - m_lastGyroYawRads}).value();
  m_lastGyroYawRads = GyroYawRads();
  m_averageWheelPosition = 0;
  for (size_t i = 0; i < GetModules().size(); i++) {
    m_averageWheelPosition += std::abs(ModulePosition(i) - m_startWheelPositions[i]);
  }
  m_averageWheelPosition = m_averageWheelPosition / 4.0;
  m_currentEffectiveWheelRadius =
      (m_accumGyroYawRads * TunerConstants::kDriveRadius) / m_averageWheelPosition;
  std::printf("Average Wheel Position: %f\n", m_averageWheelPosition);
}

void Drivetrain::EndWheelRadiusCharacterization() {
  if (!m_characterizing) {
    return;
  }
  m_characterizing = false;
  SetControl(m_characterizationRequest.WithRotationalRate(0_rad_per_s));
  if (std::abs(m_accumGyroYawRads) <= std::numbers::pi * 2.0) {
    std::printf("not enough data for characterization %f\navgWheelPos: %fradians\n",
                m_accumGyroYawRads, m_averageWheelPosition);
  } else {
    std::printf(
        "effective wheel radius: %f inches\naccumGryoYawRads: %fradians\navgWheelPos: "
        "%fradians\n",
        m_currentEffectiveWheelRadius, m_accumGyroYawRads, m_averageWheelPosition);
  }
}

}  // namespace subsystems

subsystems::Drivetrain TunerConstants::CreateDrivetrain() {
  return subsystems::Drivetrain{DrivetrainConstants, FrontLeft, FrontRight, BackLeft, BackRight};
}
