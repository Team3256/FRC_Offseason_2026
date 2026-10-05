// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.StructureState;
import mayhemlib.auto.AutoFactory;
import mayhemlib.auto.AutoRoutine;
import mayhemlib.auto.AutoTrajectory;

/**
 * Mayhem version of the top trench sweep depot auto. The paths live in {@code
 * src/main/deploy/mayhem} (open that folder as a project in the Mayhem app); the routine logic is
 * the same as the Choreo version in {@link AutoRoutines}.
 */
public class DepotOnlyMayhemAutos {

  private final AutoFactory m_factory;
  private final Superstructure m_superstructure;

  public DepotOnlyMayhemAutos(AutoFactory factory, Superstructure superstructure) {
    m_factory = factory;
    m_superstructure = superstructure;
  }

  public AutoRoutine trenchDepotOnlyMayhemAuto() {
    final AutoRoutine routine = m_factory.newRoutine("trenchDepotOnly");
    final AutoTrajectory trenchDepotOnlyMayhemAuto = routine.trajectory("trenchDepotOnly");

    routine
        .active()
        .onTrue(trenchDepotOnlyMayhemAuto.resetOdometry().andThen(trenchDepotOnlyMayhemAuto.cmd()));

    trenchDepotOnlyMayhemAuto
        .atTime("intake")
        .onTrue(m_superstructure.setState(StructureState.INTAKE));
    trenchDepotOnlyMayhemAuto.atTime("idle").onTrue(m_superstructure.setState(StructureState.IDLE));
    trenchDepotOnlyMayhemAuto
        .atTime("shoot")
        .onTrue(m_superstructure.setState(StructureState.SHOOT));

    return routine;
  }

  public AutoRoutine bumpDepotOnlyMayhemAuto() {
    final AutoRoutine routine = m_factory.newRoutine("bumpDepotOnly");
    final AutoTrajectory bumpDepotOnlyMayhemAuto = routine.trajectory("bumpDepotOnly");

    routine
        .active()
        .onTrue(bumpDepotOnlyMayhemAuto.resetOdometry().andThen(bumpDepotOnlyMayhemAuto.cmd()));

    bumpDepotOnlyMayhemAuto
        .atTime("intake")
        .onTrue(m_superstructure.setState(StructureState.INTAKE));
    bumpDepotOnlyMayhemAuto.atTime("idle").onTrue(m_superstructure.setState(StructureState.IDLE));
    bumpDepotOnlyMayhemAuto.atTime("shoot").onTrue(m_superstructure.setState(StructureState.SHOOT));

    return routine;
  }

  public AutoRoutine hubDepotOnlyMayhemAuto() {
    final AutoRoutine routine = m_factory.newRoutine("hubDepotOnly");
    final AutoTrajectory hubDepotOnlyMayhemAuto = routine.trajectory("hubDepotOnly");

    routine
        .active()
        .onTrue(hubDepotOnlyMayhemAuto.resetOdometry().andThen(hubDepotOnlyMayhemAuto.cmd()));

    hubDepotOnlyMayhemAuto
        .atTime("intake")
        .onTrue(m_superstructure.setState(StructureState.INTAKE));
    hubDepotOnlyMayhemAuto.atTime("idle").onTrue(m_superstructure.setState(StructureState.IDLE));
    hubDepotOnlyMayhemAuto.atTime("shoot").onTrue(m_superstructure.setState(StructureState.SHOOT));

    return routine;
  }
}
