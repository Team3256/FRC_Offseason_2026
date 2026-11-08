// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <array>

#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveModulePosition.h>
#include <frc/kinematics/SwerveModuleState.h>
#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StringTopic.h>
#include <networktables/StructArrayTopic.h>
#include <networktables/StructTopic.h>

#include "subsystems/swerve/CommandSwerveDrivetrain.h"

/** Publishes the swerve drive state (called by Phoenix from its odometry thread). */
class Telemetry {
 public:
  /** @param maxSpeed Maximum speed in meters per second */
  explicit Telemetry(double maxSpeed);

  /** Accept the swerve drive state and telemeterize it to SmartDashboard and SignalLogger. */
  void Telemeterize(const subsystems::CommandSwerveDrivetrain::SwerveDriveState& state);

 private:
  double m_maxSpeed;

  /* What to publish over networktables for telemetry */
  nt::NetworkTableInstance m_inst = nt::NetworkTableInstance::GetDefault();

  /* Robot swerve drive state */
  std::shared_ptr<nt::NetworkTable> m_driveStateTable = m_inst.GetTable("DriveState");
  nt::StructPublisher<frc::Pose2d> m_drivePose =
      m_driveStateTable->GetStructTopic<frc::Pose2d>("Pose").Publish();
  nt::StructPublisher<frc::ChassisSpeeds> m_driveSpeeds =
      m_driveStateTable->GetStructTopic<frc::ChassisSpeeds>("Speeds").Publish();
  nt::StructArrayPublisher<frc::SwerveModuleState> m_driveModuleStates =
      m_driveStateTable->GetStructArrayTopic<frc::SwerveModuleState>("ModuleStates").Publish();
  nt::StructArrayPublisher<frc::SwerveModuleState> m_driveModuleTargets =
      m_driveStateTable->GetStructArrayTopic<frc::SwerveModuleState>("ModuleTargets").Publish();
  nt::StructArrayPublisher<frc::SwerveModulePosition> m_driveModulePositions =
      m_driveStateTable->GetStructArrayTopic<frc::SwerveModulePosition>("ModulePositions")
          .Publish();
  nt::DoublePublisher m_driveTimestamp = m_driveStateTable->GetDoubleTopic("Timestamp").Publish();
  nt::DoublePublisher m_driveOdometryFrequency =
      m_driveStateTable->GetDoubleTopic("OdometryFrequency").Publish();

  /* Robot pose for field positioning */
  std::shared_ptr<nt::NetworkTable> m_table = m_inst.GetTable("Pose");
  nt::DoubleArrayPublisher m_fieldPub = m_table->GetDoubleArrayTopic("robotPose").Publish();
  nt::StringPublisher m_fieldTypePub = m_table->GetStringTopic(".type").Publish();

  /* Mechanisms to represent the swerve module states */
  std::array<frc::Mechanism2d, 4> m_moduleMechanisms{frc::Mechanism2d{1, 1}, frc::Mechanism2d{1, 1},
                                                     frc::Mechanism2d{1, 1},
                                                     frc::Mechanism2d{1, 1}};
  /* A direction and length changing ligament for speed representation */
  std::array<frc::MechanismLigament2d*, 4> m_moduleSpeeds{};
  /* A direction changing and length constant ligament for module direction */
  std::array<frc::MechanismLigament2d*, 4> m_moduleDirections{};

  std::array<double, 3> m_poseArray{};
  std::array<double, 8> m_moduleStatesArray{};
  std::array<double, 8> m_moduleTargetsArray{};
};
