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
 * Mayhem versions of the egg autos. The paths live in {@code src/main/deploy/mayhem} (open that
 * folder as a project in the Mayhem app); the routine logic is the same as the Choreo version in
 * {@link AutoRoutines}.
 */
public class EggMayhemAutos {

  private final AutoFactory m_factory;
  private final Superstructure m_superstructure;

  public EggMayhemAutos(AutoFactory factory, Superstructure superstructure) {
    m_factory = factory;
    m_superstructure = superstructure;
  }

  public AutoRoutine egg() {
    final AutoRoutine routine = m_factory.newRoutine("egg");
    final AutoTrajectory eggAuto = routine.trajectory("egg");
    routine.active().onTrue(eggAuto.resetOdometry().andThen(eggAuto.cmd()));

    eggAuto.atTime("intake").onTrue(m_superstructure.setState(StructureState.INTAKE));
    eggAuto.atTime("jitter").onTrue(m_superstructure.setState(StructureState.JITTER_AND_SHOOT));
    eggAuto.atTime("shoot").onTrue(m_superstructure.setState(StructureState.SHOOT));

    return routine;
  }

  public AutoRoutine eggTwice() {
    final AutoRoutine routine = m_factory.newRoutine("eggTwice");
    final AutoTrajectory eggTwiceAuto = routine.trajectory("eggtwice");
    routine.active().onTrue(eggTwiceAuto.resetOdometry().andThen(eggTwiceAuto.cmd()));

    eggTwiceAuto.atTime("intake").onTrue(m_superstructure.setState(StructureState.INTAKE));
    eggTwiceAuto
        .atTime("jitter")
        .onTrue(m_superstructure.setState(StructureState.JITTER_AND_SHOOT));
    eggTwiceAuto.atTime("shoot").onTrue(m_superstructure.setState(StructureState.SHOOT));

    return routine;
  }
}
