// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "sim/DrivetrainSim.h"

#include <utility>

#include "mayhem/sim/phoenix6.hpp"
#include "subsystems/swerve/generated/TunerConstants.h"

namespace sim {
namespace {
constexpr double kOdometryPeriod = 1.0 / 250;

frc::Translation2d Location(const decltype(TunerConstants::FrontLeft)& m) {
  return {units::meter_t{m.LocationX.value()}, units::meter_t{m.LocationY.value()}};
}

mayhem::sim::swerve::DrivetrainConstants Constants() {
  mayhem::sim::swerve::DrivetrainConstants c;
  for (const auto* m : {&TunerConstants::FrontLeft, &TunerConstants::FrontRight,
                        &TunerConstants::BackLeft, &TunerConstants::BackRight}) {
    c.modulePositions.push_back({m->LocationX.value(), m->LocationY.value()});
  }
  const auto& m = TunerConstants::FrontLeft;
  c.speedAt12Volts = TunerConstants::kSpeedAt12Volts.value();
  c.driveGearing = m.DriveMotorGearRatio;
  c.steerGearing = m.SteerMotorGearRatio;
  c.couplingRatio = m.CouplingGearRatio;
  c.wheelRadius = m.WheelRadius.value();
  // Voltage closed loops; the drive request does not enable FOC (no Phoenix Pro license).
  c.enableFoc = false;
  c.driveMotor = mayhem::sim::DcMotor::KrakenX60();
  return c;
}
}  // namespace

DrivetrainSim::DrivetrainSim(std::function<frc::Rotation2d()> operatorForward)
    : m_operatorForward{std::move(operatorForward)},
      m_kinematics{Location(TunerConstants::FrontLeft), Location(TunerConstants::FrontRight),
                   Location(TunerConstants::BackLeft), Location(TunerConstants::BackRight)},
      m_estimator{m_kinematics,    Gyro(),         Positions(), PhysicsWorld::Get().TruePose(),
                  {0.1, 0.1, 0.1}, {0.9, 0.9, 0.9}},
      m_emulator{Constants()} {
  m_state.ModuleStates.resize(4);
  m_state.ModuleTargets.resize(4);
  m_state.ModulePositions.resize(4);
  PhysicsWorld::Get().SetDriveHooks(this);
  UpdateState(0);
}

DrivetrainSim::~DrivetrainSim() {
  if (PhysicsWorld::Exists()) {
    PhysicsWorld::Get().SetDriveHooks(nullptr);
  }
}

frc::Rotation2d DrivetrainSim::Gyro() const {
  return frc::Rotation2d{units::radian_t{PhysicsWorld::Get().Swerve().GyroYaw()}};
}

wpi::array<frc::SwerveModulePosition, 4> DrivetrainSim::Positions() const {
  auto& swerve = PhysicsWorld::Get().Swerve();
  wpi::array<frc::SwerveModulePosition, 4> out{wpi::empty_array};
  for (size_t i = 0; i < 4; i++) {
    out[i] = {units::meter_t{swerve.ModuleDrivePosition(i)},
              frc::Rotation2d{units::radian_t{swerve.ModuleSteerAngle(i)}}};
  }
  return out;
}

void DrivetrainSim::SetControl(const ctre::phoenix6::swerve::requests::SwerveRequest& request) {
  if (auto converted = mayhem::sim::FromRequest(request)) {
    m_request = std::move(*converted);
  }
}

void DrivetrainSim::UpdateState(double fpgaSeconds) {
  auto& swerve = PhysicsWorld::Get().Swerve();
  wpi::array<frc::SwerveModuleState, 4> states{wpi::empty_array};
  for (size_t i = 0; i < 4; i++) {
    states[i] = {units::meters_per_second_t{swerve.ModuleDriveVelocity(i)},
                 frc::Rotation2d{units::radian_t{swerve.ModuleSteerAngle(i)}}};
    m_state.ModuleStates[i] = states[i];
  }
  const auto positions = Positions();
  for (size_t i = 0; i < 4; i++) {
    m_state.ModulePositions[i] = positions[i];
  }
  const auto& targets = m_emulator.TargetStates();
  for (size_t i = 0; i < 4 && i < targets.size(); i++) {
    m_state.ModuleTargets[i] = {units::meters_per_second_t{targets[i].speed},
                                frc::Rotation2d{units::radian_t{targets[i].angle}}};
  }
  m_state.Pose = m_estimator.GetEstimatedPosition();
  m_state.Speeds = m_kinematics.ToChassisSpeeds(states);
  m_state.RawHeading = Gyro();
  m_state.Timestamp = units::second_t{fpgaSeconds};
  m_state.OdometryPeriod = units::second_t{kOdometryPeriod};
  m_state.SuccessfulDaqs++;
}

void DrivetrainSim::ControlTick(double fpgaSeconds) {
  m_estimator.UpdateWithTime(units::second_t{fpgaSeconds}, Gyro(), Positions());
  UpdateState(fpgaSeconds);
  mayhem::sim::swerve::ControlParameters params;
  params.poseX = m_state.Pose.X().value();
  params.poseY = m_state.Pose.Y().value();
  params.heading = m_state.Pose.Rotation().Radians().value();
  params.operatorForwardDirection = m_operatorForward().Radians().value();
  params.timestamp = fpgaSeconds;
  params.updatePeriod = kOdometryPeriod;
  params.currentSpeeds = {m_state.Speeds.vx.value(), m_state.Speeds.vy.value(),
                          m_state.Speeds.omega.value()};
  m_emulator.Apply(m_request, PhysicsWorld::Get().Swerve(), params);
}

void DrivetrainSim::ResetPose(const frc::Pose2d& pose) {
  // The robot was placed at this pose (unless a test keeps its own placement).
  PhysicsWorld::Get().OnOdometryReset(pose);
  m_estimator.ResetPosition(Gyro(), Positions(), pose);
  m_state.Pose = pose;
}

void DrivetrainSim::SeedFieldCentric(const frc::Rotation2d& rotation) {
  // Phoenix: the robot's heading becomes the operator's forward direction plus `rotation`.
  const frc::Pose2d pose{m_estimator.GetEstimatedPosition().Translation(),
                         m_operatorForward() + rotation};
  m_estimator.ResetPosition(Gyro(), Positions(), pose);
  m_state.Pose = pose;
}

void DrivetrainSim::AddVisionMeasurement(const frc::Pose2d& pose, double fpgaSeconds,
                                         std::optional<std::array<double, 3>> stdDevs) {
  if (stdDevs) {
    m_estimator.AddVisionMeasurement(
        pose, units::second_t{fpgaSeconds},
        wpi::array<double, 3>{(*stdDevs)[0], (*stdDevs)[1], (*stdDevs)[2]});
  } else {
    m_estimator.AddVisionMeasurement(pose, units::second_t{fpgaSeconds});
  }
  m_state.Pose = m_estimator.GetEstimatedPosition();
}

}  // namespace sim
