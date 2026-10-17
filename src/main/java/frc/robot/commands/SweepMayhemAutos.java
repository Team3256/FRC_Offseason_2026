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

  public AutoRoutine deepFriendlyBottomSweepAuto() {
    final AutoRoutine routine = m_factory.newRoutine("deepFriendlyBottomSweepAuto");
    final AutoTrajectory deepFriendlyBottomSweepAuto =
        routine.trajectory("deepFriendlyBottomTrenchSweepBump");
    routine
        .active()
        .onTrue(
            deepFriendlyBottomSweepAuto.resetOdometry().andThen(deepFriendlyBottomSweepAuto.cmd()));
    deepFriendlyBottomSweepAuto
        .atTime("intake")
        .onTrue(m_superstructure.setState(StructureState.INTAKE));
    deepFriendlyBottomSweepAuto
        .atTime("idle")
        .onTrue(m_superstructure.setState(StructureState.IDLE));
    deepFriendlyBottomSweepAuto
        .atTime("shoot")
        .onTrue(m_superstructure.setState(StructureState.SHOOT));

    return routine;
  }

  public AutoRoutine deepFriendlyTopSweepAuto() {
    final AutoRoutine routine = m_factory.newRoutine("deepFriendlyTopSweepAuto");
    final AutoTrajectory deepFriendlyTopSweepAuto =
        routine.trajectory("deepFriendlyTopTrenchSweepBump");
    routine
        .active()
        .onTrue(deepFriendlyTopSweepAuto.resetOdometry().andThen(deepFriendlyTopSweepAuto.cmd()));
    deepFriendlyTopSweepAuto
        .atTime("intake")
        .onTrue(m_superstructure.setState(StructureState.INTAKE));
    deepFriendlyTopSweepAuto.atTime("idle").onTrue(m_superstructure.setState(StructureState.IDLE));
    deepFriendlyTopSweepAuto
        .atTime("shoot")
        .onTrue(m_superstructure.setState(StructureState.SHOOT));

    return routine;
  }

  public AutoRoutine shallowFriendlyBottomSweepAuto() {
    final AutoRoutine routine = m_factory.newRoutine("shallowFriendlyBottomSweepAuto");
    final AutoTrajectory shallowFriendlyBottomSweepAuto =
        routine.trajectory("shallowFriendlyBottomTrenchSweepBump");
    routine
        .active()
        .onTrue(
            shallowFriendlyBottomSweepAuto
                .resetOdometry()
                .andThen(shallowFriendlyBottomSweepAuto.cmd()));
    shallowFriendlyBottomSweepAuto
        .atTime("intake")
        .onTrue(m_superstructure.setState(StructureState.INTAKE));
    shallowFriendlyBottomSweepAuto
        .atTime("idle")
        .onTrue(m_superstructure.setState(StructureState.IDLE));
    shallowFriendlyBottomSweepAuto
        .atTime("shoot")
        .onTrue(m_superstructure.setState(StructureState.SHOOT));

    return routine;
  }

  public AutoRoutine shallowFriendlyTopSweepAuto() {
    final AutoRoutine routine = m_factory.newRoutine("shallowFriendlyTopSweepAuto");
    final AutoTrajectory shallowFriendlyTopSweepAuto =
        routine.trajectory("shallowFriendlyTopTrenchSweepBump");
    routine
        .active()
        .onTrue(
            shallowFriendlyTopSweepAuto.resetOdometry().andThen(shallowFriendlyTopSweepAuto.cmd()));
    shallowFriendlyTopSweepAuto
        .atTime("intake")
        .onTrue(m_superstructure.setState(StructureState.INTAKE));
    shallowFriendlyTopSweepAuto
        .atTime("idle")
        .onTrue(m_superstructure.setState(StructureState.IDLE));
    shallowFriendlyTopSweepAuto
        .atTime("shoot")
        .onTrue(m_superstructure.setState(StructureState.SHOOT));

    return routine;
  }

  public AutoRoutine deepAggressiveBottomSweepAuto() {
    final AutoRoutine routine = m_factory.newRoutine("deepAggressiveBottomSweepAuto");
    final AutoTrajectory deepAggressiveBottomSweepAuto =
        routine.trajectory("deepAggressiveBottomTrenchSweepBump");
    routine
        .active()
        .onTrue(
            deepAggressiveBottomSweepAuto
                .resetOdometry()
                .andThen(deepAggressiveBottomSweepAuto.cmd()));
    deepAggressiveBottomSweepAuto
        .atTime("intake")
        .onTrue(m_superstructure.setState(StructureState.INTAKE));
    deepAggressiveBottomSweepAuto
        .atTime("idle")
        .onTrue(m_superstructure.setState(StructureState.IDLE));
    deepAggressiveBottomSweepAuto
        .atTime("shoot")
        .onTrue(m_superstructure.setState(StructureState.SHOOT));

    return routine;
  }

  public AutoRoutine deepAggressiveTopSweepAuto() {
    final AutoRoutine routine = m_factory.newRoutine("deepAggressiveTopSweepAuto");
    final AutoTrajectory deepAggressiveTopSweepAuto =
        routine.trajectory("deepAggressiveTopTrenchSweepBump");
    routine
        .active()
        .onTrue(
            deepAggressiveTopSweepAuto.resetOdometry().andThen(deepAggressiveTopSweepAuto.cmd()));
    deepAggressiveTopSweepAuto
        .atTime("intake")
        .onTrue(m_superstructure.setState(StructureState.INTAKE));
    deepAggressiveTopSweepAuto
        .atTime("idle")
        .onTrue(m_superstructure.setState(StructureState.IDLE));
    deepAggressiveTopSweepAuto
        .atTime("shoot")
        .onTrue(m_superstructure.setState(StructureState.SHOOT));

    return routine;
  }

  public AutoRoutine shallowAggressiveBottomSweepAuto() {
    final AutoRoutine routine = m_factory.newRoutine("shallowAggressiveBottomSweepAuto");
    final AutoTrajectory shallowAggressiveBottomSweepAuto =
        routine.trajectory("shallowAggressiveBottomTrenchSweepAuto");
    routine
        .active()
        .onTrue(
            shallowAggressiveBottomSweepAuto
                .resetOdometry()
                .andThen(shallowAggressiveBottomSweepAuto.cmd()));
    shallowAggressiveBottomSweepAuto
        .atTime("intake")
        .onTrue(m_superstructure.setState(StructureState.INTAKE));
    shallowAggressiveBottomSweepAuto
        .atTime("idle")
        .onTrue(m_superstructure.setState(StructureState.IDLE));
    shallowAggressiveBottomSweepAuto
        .atTime("shoot")
        .onTrue(m_superstructure.setState(StructureState.SHOOT));

    return routine;
  }

  public AutoRoutine shallowAggressiveTopSweepAuto() {
    final AutoRoutine routine = m_factory.newRoutine("shallowAggressiveTopSweepAuto");
    final AutoTrajectory shallowAggressiveTopSweepAuto =
        routine.trajectory("shallowAggressiveTopTrenchSweepAuto");
    routine
        .active()
        .onTrue(
            shallowAggressiveTopSweepAuto
                .resetOdometry()
                .andThen(shallowAggressiveTopSweepAuto.cmd()));
    shallowAggressiveTopSweepAuto
        .atTime("intake")
        .onTrue(m_superstructure.setState(StructureState.INTAKE));
    shallowAggressiveTopSweepAuto
        .atTime("idle")
        .onTrue(m_superstructure.setState(StructureState.IDLE));
    shallowAggressiveTopSweepAuto
        .atTime("shoot")
        .onTrue(m_superstructure.setState(StructureState.SHOOT));

    return routine;
  }
}
