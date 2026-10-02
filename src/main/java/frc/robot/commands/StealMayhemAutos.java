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
 * Mayhem versions of the steal autos in {@link AutoRoutines}. The paths live in {@code
 * src/main/deploy/mayhem} (open that folder as a project in the Mayhem app); the routine logic is
 * the same as the Choreo version.
 */
public class StealMayhemAutos {

  private final AutoFactory m_factory;
  private final Superstructure m_superstructure;

  public StealMayhemAutos(AutoFactory factory, Superstructure superstructure) {
    m_factory = factory;
    m_superstructure = superstructure;
  }

  public AutoRoutine stealAuto() {
    final AutoRoutine routine = m_factory.newRoutine("stealAuto");
    final AutoTrajectory stealAuto = routine.trajectory("steal");
    final AutoTrajectory stealP2 = routine.trajectory("stealp2");

    routine.active().onTrue(stealAuto.resetOdometry().andThen(stealAuto.cmd()));

    stealAuto.atTime("Intake 1").onTrue(m_superstructure.setState(StructureState.INTAKE));

    stealAuto.doneDelayed(2).onTrue(stealP2.cmd());

    stealP2
        .atTime("Intake 5")
        .onTrue(
            m_superstructure
                .setState(StructureState.IDLE)
                .andThen(m_superstructure.setState(StructureState.REV)));

    stealP2
        .atTime("Shoot")
        .onTrue(
            Commands.waitSeconds(1.5)
                .andThen(m_superstructure.setState(StructureState.SHOOT))
                .andThen(Commands.waitSeconds(1))
                .andThen(m_superstructure.setState(StructureState.JITTER_AND_SHOOT)));

    return routine;
  }
}
