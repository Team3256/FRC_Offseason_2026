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
public class FollowAutos {

  private final AutoFactory m_factory;
  private final Superstructure m_superstructure;

  public FollowAutos(AutoFactory factory, Superstructure superstructure) {
    m_factory = factory;
    m_superstructure = superstructure;
  }

  public AutoRoutine trenchDepotOnlyMayhemAutos() {
    final AutoRoutine routine = m_factory.newRoutine("followTrench");
    final AutoTrajectory followTrenchAuto = routine.trajectory("followTrench");
    // The Choreo version also loads "topTrenchSweepDepotPt3", but that path was deleted, and
    // MayhemLib throws if a trajectory file is missing, so it is left out here.

    routine
            .active()
            .onTrue(followTrenchAuto.resetOdometry().andThen(followTrenchAuto.cmd()));

    followTrenchAuto
            .atTime("intake")
            .onTrue(m_superstructure.setState(StructureState.INTAKE));
    followTrenchAuto.atTime("idle").onTrue(m_superstructure.setState(StructureState.IDLE));
    followTrenchAuto.atTime("shoot").onTrue(m_superstructure.setState(StructureState.SHOOT));



    return routine;
  }
}