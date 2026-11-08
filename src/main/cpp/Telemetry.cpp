// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "Telemetry.h"

#include <string>

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/util/Color.h>
#include <frc/util/Color8Bit.h>
#include <ctre/phoenix6/SignalLogger.hpp>

using namespace ctre::phoenix6;

Telemetry::Telemetry(double maxSpeed) : m_maxSpeed{maxSpeed} {
  SignalLogger::Start();
  for (size_t i = 0; i < 4; ++i) {
    m_moduleSpeeds[i] = m_moduleMechanisms[i]
                            .GetRoot("RootSpeed", 0.5, 0.5)
                            ->Append<frc::MechanismLigament2d>("Speed", 0.5, 0_deg);
    m_moduleDirections[i] = m_moduleMechanisms[i]
                                .GetRoot("RootDirection", 0.5, 0.5)
                                ->Append<frc::MechanismLigament2d>(
                                    "Direction", 0.1, 0_deg, 0, frc::Color8Bit{frc::Color::kWhite});
    // (Java re-put these every call; the dashboard keeps the same object, so once is enough)
    frc::SmartDashboard::PutData("Module " + std::to_string(i), &m_moduleMechanisms[i]);
  }
}

void Telemetry::Telemeterize(const subsystems::CommandSwerveDrivetrain::SwerveDriveState& state) {
  /* Telemeterize the swerve drive state */
  m_drivePose.Set(state.Pose);
  m_driveSpeeds.Set(state.Speeds);
  m_driveModuleStates.Set(state.ModuleStates);
  m_driveModuleTargets.Set(state.ModuleTargets);
  m_driveModulePositions.Set(state.ModulePositions);
  m_driveTimestamp.Set(state.Timestamp.value());
  m_driveOdometryFrequency.Set(1.0 / state.OdometryPeriod.value());

  /* Also write to log file */
  m_poseArray[0] = state.Pose.X().value();
  m_poseArray[1] = state.Pose.Y().value();
  m_poseArray[2] = state.Pose.Rotation().Degrees().value();
  for (size_t i = 0; i < 4 && i < state.ModuleStates.size(); ++i) {
    m_moduleStatesArray[i * 2 + 0] = state.ModuleStates[i].angle.Radians().value();
    m_moduleStatesArray[i * 2 + 1] = state.ModuleStates[i].speed.value();
    m_moduleTargetsArray[i * 2 + 0] = state.ModuleTargets[i].angle.Radians().value();
    m_moduleTargetsArray[i * 2 + 1] = state.ModuleTargets[i].speed.value();
  }
  SignalLogger::WriteDoubleArray("DriveState/Pose", m_poseArray);
  SignalLogger::WriteDoubleArray("DriveState/ModuleStates", m_moduleStatesArray);
  SignalLogger::WriteDoubleArray("DriveState/ModuleTargets", m_moduleTargetsArray);
  SignalLogger::WriteDouble("DriveState/OdometryPeriod", state.OdometryPeriod.value(), "seconds");

  /* Telemeterize the pose to a Field2d */
  m_fieldTypePub.Set("Field2d");
  m_fieldPub.Set(m_poseArray);

  /* Telemeterize the module states to a Mechanism2d */
  for (size_t i = 0; i < 4 && i < state.ModuleStates.size(); ++i) {
    m_moduleSpeeds[i]->SetAngle(state.ModuleStates[i].angle.Degrees());
    m_moduleDirections[i]->SetAngle(state.ModuleStates[i].angle.Degrees());
    m_moduleSpeeds[i]->SetLength(state.ModuleStates[i].speed.value() / (2 * m_maxSpeed));
  }
}
