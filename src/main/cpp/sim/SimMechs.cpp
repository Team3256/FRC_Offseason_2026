// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "sim/SimMechs.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/util/Color.h>
#include <frc/util/Color8Bit.h>

#include "Constants.h"
#include "subsystems/linearslide/LinearSlideConstants.h"

namespace {
constexpr double kWidth = units::meter_t{constants::simulation::kDrivebaseWidth}.value();
constexpr double In(double in) { return in * 0.0254; }
}  // namespace

SimMechs::SimMechs() : mech{kWidth, 1.0} {
  using frc::MechanismLigament2d;
  m_groundIntakeRollerViz = mech.GetRoot("Ground Intake", kWidth / 2 - 0.15, In(2))
                                ->Append<MechanismLigament2d>("Intake Roller", In(2), 0_deg, 3,
                                                              frc::Color8Bit{frc::Color::kRed});
  m_feederViz = mech.GetRoot("Feeder", kWidth / 2, In(5))
                    ->Append<MechanismLigament2d>("Feeder", In(2), 0_deg, 3,
                                                  frc::Color8Bit{frc::Color::kMagenta});
  m_indexerViz = mech.GetRoot("Indexer", kWidth / 2 - 0.15, In(2))
                     ->Append<MechanismLigament2d>("Indexer", In(2), 0_deg, 5,
                                                   frc::Color8Bit{frc::Color::kGreen});
  auto* pivotRoot = mech.GetRoot("Shooter Pivot", kWidth / 2 - 0.05, In(25));
  m_shooterPivotViz = pivotRoot->Append<MechanismLigament2d>("Shooter Pivot", In(10), 0_deg, 7,
                                                             frc::Color8Bit{frc::Color::kCyan});
  m_shooterWheelViz = pivotRoot->Append<MechanismLigament2d>("Shooter Wheel", In(4), 0_deg, 4,
                                                             frc::Color8Bit{frc::Color::kPink});
  const double slideMin = units::meter_t{linear_slide::sim::kMinLength}.value();
  m_linearSlideViz = mech.GetRoot("Linear Slide", kWidth / 2, slideMin)
                         ->Append<MechanismLigament2d>("Linear Slide", slideMin + In(6), 180_deg, 4,
                                                       frc::Color8Bit{frc::Color::kOrange});
}

SimMechs& SimMechs::GetInstance() {
  static SimMechs instance;
  return instance;
}

void SimMechs::UpdateRollers(units::degree_t x) {
  m_groundIntakeRollerViz->SetAngle(units::degree_t{m_groundIntakeRollerViz->GetAngle()} + x);
}
void SimMechs::UpdateFeeder(units::degree_t x) {
  m_feederViz->SetAngle(units::degree_t{m_feederViz->GetAngle()} + x);
}
void SimMechs::UpdateShooterPivot(units::degree_t angle) { m_shooterPivotViz->SetAngle(angle); }
void SimMechs::UpdateIndexer(units::degree_t x) {
  m_indexerViz->SetAngle(units::degree_t{m_indexerViz->GetAngle()} + x);
}
void SimMechs::UpdateShooterWheel(units::degree_t delta) {
  m_shooterWheelViz->SetAngle(units::degree_t{m_shooterWheelViz->GetAngle()} + delta);
}
void SimMechs::UpdateLinearSlide(units::meter_t height) {
  m_linearSlideViz->SetLength(height.value());
}
void SimMechs::PublishToNT() { frc::SmartDashboard::PutData("RobotSim", &mech); }
