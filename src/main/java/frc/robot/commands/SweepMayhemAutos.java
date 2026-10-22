// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.StructureState;
import mayhemlib.auto.AutoFactory;
import mayhemlib.auto.AutoRoutine;
import mayhemlib.auto.AutoTrajectory;

public class SweepMayhemAutos {

  private final AutoFactory m_factory;
  private final Superstructure m_superstructure;

  public SweepMayhemAutos(AutoFactory factory, Superstructure superstructure) {
    m_factory = factory;
    m_superstructure = superstructure;
  }

  // 1. Bottom Trench: Part 1 only
  public AutoRoutine bottomTrenchPart1Only() {
    AutoRoutine routine = m_factory.newRoutine("bottomTrenchPart1Only");

    AutoTrajectory pt1 = routine.trajectory("bottomTrenchSweepBumpPt1");

    routine.active().onTrue(pt1.resetOdometry().andThen(pt1.cmd()));

    pt1.atTime("intake").onTrue(m_superstructure.setState(StructureState.INTAKE));
    pt1.atTime("idle").onTrue(m_superstructure.setState(StructureState.IDLE));
    pt1.atTime("shoot").onTrue(m_superstructure.setState(StructureState.SHOOT));

    return routine;
  }

  // 2. Bottom Trench: Parts 1 and 2
  public AutoRoutine bottomTrenchPart1And2() {
    AutoRoutine routine = m_factory.newRoutine("bottomTrenchPart1And2");

    AutoTrajectory pt1 = routine.trajectory("bottomTrenchSweepBumpPt1");
    AutoTrajectory pt2 = routine.trajectory("bottomTrenchSweepBumpPt2");

    routine.active().onTrue(pt1.resetOdometry().andThen(pt1.cmd()));

    pt1.atTime("intake").onTrue(m_superstructure.setState(StructureState.INTAKE));
    pt1.atTime("idle").onTrue(m_superstructure.setState(StructureState.IDLE));
    pt1.atTime("shoot").onTrue(m_superstructure.setState(StructureState.SHOOT));

    pt1.done().onTrue(Commands.waitSeconds(3).andThen(pt2.cmd()));

    pt2.atTime("intake2").onTrue(m_superstructure.setState(StructureState.INTAKE));
    pt2.atTime("idle2").onTrue(m_superstructure.setState(StructureState.IDLE));
    pt2.atTime("shoot2").onTrue(m_superstructure.setState(StructureState.SHOOT));

    return routine;
  }

  // 3. Top Trench: Part 1 only
  public AutoRoutine topTrenchPart1Only() {
    AutoRoutine routine = m_factory.newRoutine("topTrenchPart1Only");

    AutoTrajectory pt1 = routine.trajectory("topTrenchSweepBump2xPt1");

    routine.active().onTrue(pt1.resetOdometry().andThen(pt1.cmd()));

    pt1.atTime("intake").onTrue(m_superstructure.setState(StructureState.INTAKE));
    pt1.atTime("idle").onTrue(m_superstructure.setState(StructureState.IDLE));
    pt1.atTime("shoot").onTrue(m_superstructure.setState(StructureState.SHOOT));

    return routine;
  }

  // 4. Top Trench: Parts 1 and 2
  public AutoRoutine topTrenchPart1And2() {
    AutoRoutine routine = m_factory.newRoutine("topTrenchPart1And2");

    AutoTrajectory pt1 = routine.trajectory("topTrenchSweepBump2xPt1");
    AutoTrajectory pt2 = routine.trajectory("topTrenchSweepBump2xPt2");

    routine.active().onTrue(pt1.resetOdometry().andThen(pt1.cmd()));

    pt1.atTime("intake").onTrue(m_superstructure.setState(StructureState.INTAKE));
    pt1.atTime("idle").onTrue(m_superstructure.setState(StructureState.IDLE));
    pt1.atTime("shoot").onTrue(m_superstructure.setState(StructureState.SHOOT));

    pt1.done().onTrue(Commands.waitSeconds(3).andThen(pt2.cmd()));

    pt2.atTime("intake2").onTrue(m_superstructure.setState(StructureState.INTAKE));
    pt2.atTime("idle2").onTrue(m_superstructure.setState(StructureState.IDLE));
    pt2.atTime("shoot2").onTrue(m_superstructure.setState(StructureState.SHOOT));

    return routine;
  }
}
