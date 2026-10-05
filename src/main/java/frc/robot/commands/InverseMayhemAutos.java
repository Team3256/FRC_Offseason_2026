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
 * Mayhem version of the bottom inverse auto. The paths live in {@code src/main/deploy/mayhem} (open
 * that folder as a project in the Mayhem app); the routine logic is the same as the Choreo version
 * in {@link AutoRoutines}.
 */
public class InverseMayhemAutos {

  private final AutoFactory m_factory;
  private final Superstructure m_superstructure;

  public InverseMayhemAutos(AutoFactory factory, Superstructure superstructure) {
    m_factory = factory;
    m_superstructure = superstructure;
  }

  public AutoRoutine bottomInverseAuto() {
    final AutoRoutine routine = m_factory.newRoutine("bottomInverseAuto");
    final AutoTrajectory bottomInverseAuto = routine.trajectory("bottomInverseAuto");
    final AutoTrajectory bottomInverseAutopt2 = routine.trajectory("bottomInverseAutopt2");
    final AutoTrajectory bottomInverseAutopt3 = routine.trajectory("bottomInverseAutopt3");

    routine.active().onTrue(bottomInverseAuto.resetOdometry().andThen(bottomInverseAuto.cmd()));

    bottomInverseAuto.atTime("Intake").onTrue(m_superstructure.setState(StructureState.INTAKE));
    bottomInverseAuto.atTime("StopIntake").onTrue(m_superstructure.setState(StructureState.IDLE));
    bottomInverseAuto
        .atTime("Shoot")
        .onTrue(
            m_superstructure
                .setState(StructureState.SHOOT)
                .andThen(Commands.waitSeconds(1))
                .andThen(m_superstructure.setState(StructureState.JITTER_AND_SHOOT))
                .andThen(Commands.waitSeconds(3)));

    bottomInverseAuto.done().onTrue(bottomInverseAutopt2.cmd());

    bottomInverseAutopt2.atTime("Intake").onTrue(m_superstructure.setState(StructureState.INTAKE));
    bottomInverseAutopt2
        .atTime("StopIntake")
        .onTrue(m_superstructure.setState(StructureState.IDLE));
    bottomInverseAutopt2
        .atTime("Shoot")
        .onTrue(
            m_superstructure
                .setState(StructureState.SHOOT)
                .andThen(Commands.waitSeconds(1))
                .andThen(m_superstructure.setState(StructureState.JITTER_AND_SHOOT))
                .andThen(Commands.waitSeconds(3)));

    bottomInverseAutopt2.done().onTrue(bottomInverseAutopt3.cmd());

    bottomInverseAutopt3.atTime("Intake").onTrue(m_superstructure.setState(StructureState.INTAKE));

    return routine;
  }

  public AutoRoutine topInverseAuto() {
    final AutoRoutine routine = m_factory.newRoutine("topInverseAuto");
    final AutoTrajectory topInverseAuto = routine.trajectory("topInverseAuto");
    final AutoTrajectory topInverseAutopt2 = routine.trajectory("topInverseAutopt2");
    final AutoTrajectory topInverseAutopt3 = routine.trajectory("topInverseAutopt3");

    routine.active().onTrue(topInverseAuto.resetOdometry().andThen(topInverseAuto.cmd()));

    topInverseAuto.atTime("Intake").onTrue(m_superstructure.setState(StructureState.INTAKE));
    topInverseAuto.atTime("StopIntake").onTrue(m_superstructure.setState(StructureState.IDLE));
    topInverseAuto
        .atTime("Shoot")
        .onTrue(
            m_superstructure
                .setState(StructureState.SHOOT)
                .andThen(Commands.waitSeconds(1))
                .andThen(m_superstructure.setState(StructureState.JITTER_AND_SHOOT))
                .andThen(Commands.waitSeconds(3)));

    topInverseAuto.done().onTrue(topInverseAutopt2.cmd());

    topInverseAutopt2.atTime("Intake").onTrue(m_superstructure.setState(StructureState.INTAKE));
    topInverseAutopt2
        .atTime("StopIntake")
        .onTrue(m_superstructure.setState(StructureState.IDLE));
    topInverseAutopt2
        .atTime("Shoot")
        .onTrue(
            m_superstructure
                .setState(StructureState.SHOOT)
                .andThen(Commands.waitSeconds(1))
                .andThen(m_superstructure.setState(StructureState.JITTER_AND_SHOOT))
                .andThen(Commands.waitSeconds(3)));

    topInverseAutopt2.done().onTrue(topInverseAutopt3.cmd());

    topInverseAutopt3.atTime("Intake").onTrue(m_superstructure.setState(StructureState.INTAKE));

    return routine;
  }
}
