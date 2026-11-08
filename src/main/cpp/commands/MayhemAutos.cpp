// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "commands/MayhemAutos.h"

#include <numbers>

#include <frc2/command/Commands.h>

#include "subsystems/Superstructure.h"
#include "subsystems/sotm/ShotCalculator.h"
#include "subsystems/swerve/CommandSwerveDrivetrain.h"

using SS = Superstructure::StructureState;

frc2::CommandPtr MayhemAutosBase::AlignAndShoot() {
  auto* drivetrain = m_drivetrain;
  auto* shotCalculator = m_shotCalculator;
  return frc2::cmd::Parallel(m_drivetrain->RotateToLookahead(
                                 *m_shotCalculator, [] { return 0.0; }, [] { return 0.0; }),
                             frc2::cmd::WaitUntil([drivetrain, shotCalculator] {
                               return ShotCalculator::IsAimed(drivetrain->State().Pose.Rotation(),
                                                              shotCalculator->GetDriveAngle(),
                                                              3.0 * std::numbers::pi / 180.0);
                             }).AndThen(m_superstructure.SetState(SS::SHOOT)))
      .WithTimeout(3.5_s);
}

std::shared_ptr<mayhem::AutoRoutine> SweepMayhemAutos::BottomTrenchPart1Only() {
  auto routine = m_factory.NewRoutine("OutpostTrenchSweepBumpPt1");
  auto& pt1 = routine->Trajectory("OutpostTrenchSweepBumpPt1");
  routine->Active().OnTrue(pt1.ResetOdometry().AndThen(pt1.Cmd()));
  pt1.AtTime("intake").OnTrue(m_superstructure.SetState(SS::INTAKE));
  pt1.AtTime("idle").OnTrue(m_superstructure.SetState(SS::IDLE));
  pt1.Done().OnTrue(AlignAndShoot());
  return routine;
}

std::shared_ptr<mayhem::AutoRoutine> SweepMayhemAutos::BottomTrenchPart1And2() {
  auto routine = m_factory.NewRoutine("OutpostTrenchSweepPart1And2");
  auto& pt1 = routine->Trajectory("OutpostTrenchSweepBumpPt1");
  auto& pt2 = routine->Trajectory("OutpostTrenchSweepBumpPt2");
  routine->Active().OnTrue(pt1.ResetOdometry().AndThen(pt1.Cmd()));
  pt1.AtTime("intake").OnTrue(m_superstructure.SetState(SS::INTAKE));
  pt1.AtTime("idle").OnTrue(m_superstructure.SetState(SS::IDLE));
  pt1.Done().OnTrue(AlignAndShoot().AndThen(pt2.Cmd()));
  pt2.AtTime("intake2").OnTrue(m_superstructure.SetState(SS::INTAKE));
  pt2.AtTime("idle2").OnTrue(m_superstructure.SetState(SS::IDLE));
  pt2.Done().OnTrue(AlignAndShoot());
  return routine;
}

std::shared_ptr<mayhem::AutoRoutine> SweepMayhemAutos::TopTrenchPart1Only() {
  auto routine = m_factory.NewRoutine("DepotTrenchPart1Only");
  auto& pt1 = routine->Trajectory("DepotTrenchSweepBumpPt1");
  routine->Active().OnTrue(pt1.ResetOdometry().AndThen(pt1.Cmd()));
  pt1.AtTime("intake").OnTrue(m_superstructure.SetState(SS::INTAKE));
  pt1.AtTime("idle").OnTrue(m_superstructure.SetState(SS::IDLE));
  pt1.Done().OnTrue(AlignAndShoot());
  return routine;
}

std::shared_ptr<mayhem::AutoRoutine> SweepMayhemAutos::TopTrenchPart1And2() {
  auto routine = m_factory.NewRoutine("DepotTrenchPart1And2");
  auto& pt1 = routine->Trajectory("DepotTrenchSweepBumpPt1");
  auto& pt2 = routine->Trajectory("DepotTrenchSweepBump2xPt2");
  routine->Active().OnTrue(pt1.ResetOdometry().AndThen(pt1.Cmd()));
  pt1.AtTime("intake").OnTrue(m_superstructure.SetState(SS::INTAKE));
  pt1.AtTime("idle").OnTrue(m_superstructure.SetState(SS::IDLE));
  pt1.Done().OnTrue(AlignAndShoot().AndThen(pt2.Cmd()));
  pt2.AtTime("intake2").OnTrue(m_superstructure.SetState(SS::INTAKE));
  pt2.AtTime("idle2").OnTrue(m_superstructure.SetState(SS::IDLE));
  pt2.Done().OnTrue(AlignAndShoot());
  return routine;
}

std::shared_ptr<mayhem::AutoRoutine> SweepDepotMayhemAutos::TopTrenchSweepDepot() {
  auto routine = m_factory.NewRoutine("topTrenchSweepDepot");
  auto& part1 = routine->Trajectory("DepotTrenchSweepDepotPt");
  auto& part2 = routine->Trajectory("DepotTrenchSweepDepotPt2");
  // The Choreo version also loaded "topTrenchSweepDepotPt3", but that path was deleted, and
  // MayhemLib throws if a trajectory file is missing, so it is left out here.
  routine->Active().OnTrue(part1.ResetOdometry().AndThen(part1.Cmd()));
  part1.AtTime("intake").OnTrue(m_superstructure.SetState(SS::INTAKE));
  part1.AtTime("idle").OnTrue(m_superstructure.SetState(SS::IDLE));
  part1.Done().OnTrue(AlignAndShoot().AndThen(part2.Cmd()));
  part2.AtTime("intake").OnTrue(m_superstructure.SetState(SS::INTAKE));
  part2.Done().OnTrue(AlignAndShoot());
  return routine;
}

namespace {
std::shared_ptr<mayhem::AutoRoutine> DepotOnly(mayhem::AutoFactory& factory, Superstructure& ss,
                                               const char* name) {
  auto routine = factory.NewRoutine(name);
  auto& traj = routine->Trajectory(name);
  routine->Active().OnTrue(traj.ResetOdometry().AndThen(traj.Cmd()));
  traj.AtTime("intake").OnTrue(ss.SetState(SS::INTAKE));
  traj.AtTime("idle").OnTrue(ss.SetState(SS::IDLE));
  traj.AtTime("shoot").OnTrue(ss.SetState(SS::SHOOT));
  return routine;
}
}  // namespace

std::shared_ptr<mayhem::AutoRoutine> DepotOnlyMayhemAutos::TrenchDepotOnlyMayhemAuto() {
  return DepotOnly(m_factory, m_superstructure, "trenchDepotOnly");
}
std::shared_ptr<mayhem::AutoRoutine> DepotOnlyMayhemAutos::BumpDepotOnlyMayhemAuto() {
  return DepotOnly(m_factory, m_superstructure, "bumpDepotOnly");
}
std::shared_ptr<mayhem::AutoRoutine> DepotOnlyMayhemAutos::HubDepotOnlyMayhemAuto() {
  return DepotOnly(m_factory, m_superstructure, "hubDepotOnly");
}
