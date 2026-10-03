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

/**
 * Mayhem versions of the trench sweep autos. The paths live in {@code src/main/deploy/mayhem} (open
 * that folder as a project in the Mayhem app); the routine logic is the same as the Choreo version
 * in {@link AutoRoutines}.
 */
public class SweepMayhemAutos {

  private final AutoFactory m_factory;
  private final Superstructure m_superstructure;

  public SweepMayhemAutos(AutoFactory factory, Superstructure superstructure) {
    m_factory = factory;
    m_superstructure = superstructure;
  }

  public AutoRoutine topTrenchSweepAuto() {
    final AutoRoutine routine = m_factory.newRoutine("topTrenchSweepAuto");
    final AutoTrajectory topTrenchSweepAuto = routine.trajectory("topTrenchSweepBump");
    routine.active().onTrue(topTrenchSweepAuto.resetOdometry().andThen(topTrenchSweepAuto.cmd()));
    topTrenchSweepAuto.atTime("intake").onTrue(m_superstructure.setState(StructureState.INTAKE));
    topTrenchSweepAuto.atTime("idle").onTrue(m_superstructure.setState(StructureState.IDLE));
    topTrenchSweepAuto.atTime("shoot").onTrue(m_superstructure.setState(StructureState.SHOOT));

    return routine;
  }

  public AutoRoutine bottomTrenchSweepAuto() {
    final AutoRoutine routine = m_factory.newRoutine("bottomTrenchSweepAuto");
    final AutoTrajectory bottomTrenchSweepAuto = routine.trajectory("bottomTrenchSweepBump");
    routine
        .active()
        .onTrue(bottomTrenchSweepAuto.resetOdometry().andThen(bottomTrenchSweepAuto.cmd()));
    bottomTrenchSweepAuto.atTime("intake").onTrue(m_superstructure.setState(StructureState.INTAKE));
    bottomTrenchSweepAuto.atTime("idle").onTrue(m_superstructure.setState(StructureState.IDLE));
    bottomTrenchSweepAuto.atTime("shoot").onTrue(m_superstructure.setState(StructureState.SHOOT));

    return routine;
  }

  public AutoRoutine topTrenchSweep2xAuto() {
    final AutoRoutine routine = m_factory.newRoutine("topTrenchSweep2xAuto");
    final AutoTrajectory topTrenchSweep2xAutoPt1 = routine.trajectory("topTrenchSweepBump2xPt1");
    final AutoTrajectory topTrenchSweep2xAutoPt2 = routine.trajectory("topTrenchSweepBump2xPt2");
    routine
        .active()
        .onTrue(topTrenchSweep2xAutoPt1.resetOdometry().andThen(topTrenchSweep2xAutoPt1.cmd()));
    topTrenchSweep2xAutoPt1
        .atTime("intake")
        .onTrue(m_superstructure.setState(StructureState.INTAKE));
    topTrenchSweep2xAutoPt1.atTime("idle").onTrue(m_superstructure.setState(StructureState.IDLE));
    topTrenchSweep2xAutoPt1.atTime("shoot").onTrue(m_superstructure.setState(StructureState.SHOOT));

    topTrenchSweep2xAutoPt1
        .done()
        .onTrue(Commands.waitSeconds(1).andThen(topTrenchSweep2xAutoPt2.cmd()));

    topTrenchSweep2xAutoPt2
        .atTime("intake2")
        .onTrue(m_superstructure.setState(StructureState.INTAKE));
    topTrenchSweep2xAutoPt2.atTime("idle2").onTrue(m_superstructure.setState(StructureState.IDLE));
    topTrenchSweep2xAutoPt2
        .atTime("shoot2")
        .onTrue(m_superstructure.setState(StructureState.SHOOT));

    return routine;
  }

  public AutoRoutine bottomTrenchSweep2xAuto() {
    final AutoRoutine routine = m_factory.newRoutine("bottomTrenchSweep2xAuto");
    final AutoTrajectory bottomTrenchSweep2xAutoPt1 =
        routine.trajectory("bottomTrenchSweepBump2xPt1");
    final AutoTrajectory bottomTrenchSweep2xAutoPt2 =
        routine.trajectory("bottomTrenchSweepBump2xPt2");
    routine
        .active()
        .onTrue(
            bottomTrenchSweep2xAutoPt1.resetOdometry().andThen(bottomTrenchSweep2xAutoPt1.cmd()));
    bottomTrenchSweep2xAutoPt1
        .atTime("intake")
        .onTrue(m_superstructure.setState(StructureState.INTAKE));
    bottomTrenchSweep2xAutoPt1
        .atTime("idle")
        .onTrue(m_superstructure.setState(StructureState.IDLE));
    bottomTrenchSweep2xAutoPt1
        .atTime("shoot")
        .onTrue(m_superstructure.setState(StructureState.SHOOT));

    bottomTrenchSweep2xAutoPt1
        .done()
        .onTrue(Commands.waitSeconds(1).andThen(bottomTrenchSweep2xAutoPt2.cmd()));

    bottomTrenchSweep2xAutoPt2
        .atTime("intake2")
        .onTrue(m_superstructure.setState(StructureState.INTAKE));
    bottomTrenchSweep2xAutoPt2
        .atTime("idle2")
        .onTrue(m_superstructure.setState(StructureState.IDLE));
    bottomTrenchSweep2xAutoPt2
        .atTime("shoot2")
        .onTrue(m_superstructure.setState(StructureState.SHOOT));

    return routine;
  }
}
