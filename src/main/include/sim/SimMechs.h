// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <frc/smartdashboard/Mechanism2d.h>
#include <frc/smartdashboard/MechanismLigament2d.h>
#include <frc/smartdashboard/MechanismRoot2d.h>
#include <units/angle.h>
#include <units/length.h>

/** Mechanism2d drawing of the robot's mechanisms for simulation ("RobotSim" on the dashboard). */
class SimMechs {
 public:
  static SimMechs& GetInstance();

  void UpdateRollers(units::degree_t delta);
  void UpdateFeeder(units::degree_t delta);
  void UpdateShooterPivot(units::degree_t angle);
  void UpdateIndexer(units::degree_t delta);
  void UpdateShooterWheel(units::degree_t delta);
  void UpdateLinearSlide(units::meter_t height);
  void PublishToNT();

  frc::Mechanism2d mech;

  double ShooterPivotAngleDegrees() const { return m_shooterPivotViz->GetAngle(); }
  double LinearSlideLengthMeters() const { return m_linearSlideViz->GetLength(); }

 private:
  SimMechs();
  frc::MechanismLigament2d* m_groundIntakeRollerViz;
  frc::MechanismLigament2d* m_feederViz;
  frc::MechanismLigament2d* m_indexerViz;
  frc::MechanismLigament2d* m_shooterPivotViz;
  frc::MechanismLigament2d* m_shooterWheelViz;
  frc::MechanismLigament2d* m_linearSlideViz;
};
