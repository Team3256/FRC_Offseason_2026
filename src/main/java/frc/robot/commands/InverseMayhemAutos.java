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

  public AutoRoutine bottomInverseAutoDeep() {
    final AutoRoutine routine = m_factory.newRoutine("bottomInverseAutoDeep");
    final AutoTrajectory bottomInverseAutoDeep = routine.trajectory("bottomInverseAutoDeep");
    final AutoTrajectory bottomInverseAutoDeeppt2 = routine.trajectory("bottomInverseAutoDeeppt2");
    final AutoTrajectory bottomInverseAutoDeeppt3 = routine.trajectory("bottomInverseAutoDeeppt3");

    routine.active().onTrue(bottomInverseAutoDeep.resetOdometry().andThen(bottomInverseAutoDeep.cmd()));

    bottomInverseAutoDeep.atTime("Intake").onTrue(m_superstructure.setState(StructureState.INTAKE));
    bottomInverseAutoDeep.atTime("StopIntake").onTrue(m_superstructure.setState(StructureState.IDLE));
    bottomInverseAutoDeep
        .atTime("Shoot")
        .onTrue(
            m_superstructure
                .setState(StructureState.SHOOT)
                .andThen(Commands.waitSeconds(1))
                .andThen(m_superstructure.setState(StructureState.JITTER_AND_SHOOT))
                .andThen(Commands.waitSeconds(3)));

    bottomInverseAutoDeep.done().onTrue(bottomInverseAutoDeeppt2.cmd());

    bottomInverseAutoDeeppt2.atTime("Intake").onTrue(m_superstructure.setState(StructureState.INTAKE));
    bottomInverseAutoDeeppt2
        .atTime("StopIntake")
        .onTrue(m_superstructure.setState(StructureState.IDLE));
    bottomInverseAutoDeeppt2
        .atTime("Shoot")
        .onTrue(
            m_superstructure
                .setState(StructureState.SHOOT)
                .andThen(Commands.waitSeconds(1))
                .andThen(m_superstructure.setState(StructureState.JITTER_AND_SHOOT))
                .andThen(Commands.waitSeconds(3)));

    bottomInverseAutoDeeppt2.done().onTrue(bottomInverseAutoDeeppt3.cmd());

    bottomInverseAutoDeeppt3.atTime("Intake").onTrue(m_superstructure.setState(StructureState.INTAKE));

    return routine;
  }

  public AutoRoutine bottomInverseAutoAggressive() {
    final AutoRoutine routine = m_factory.newRoutine("bottomInverseAutoAggressive");
    final AutoTrajectory bottomInverseAutoAggressive = routine.trajectory("bottomInverseAutoAggressive");
    final AutoTrajectory bottomInverseAutoAggressivept2 = routine.trajectory("bottomInverseAutoAggressivept2");
    final AutoTrajectory bottomInverseAutoAggressivept3 = routine.trajectory("bottomInverseAutoAggressivept3");

    routine.active().onTrue(bottomInverseAutoAggressive.resetOdometry().andThen(bottomInverseAutoAggressive.cmd()));

    bottomInverseAutoAggressive.atTime("Intake").onTrue(m_superstructure.setState(StructureState.INTAKE));
    bottomInverseAutoAggressive.atTime("StopIntake").onTrue(m_superstructure.setState(StructureState.IDLE));
    bottomInverseAutoAggressive
        .atTime("Shoot")
        .onTrue(
            m_superstructure
                .setState(StructureState.SHOOT)
                .andThen(Commands.waitSeconds(1))
                .andThen(m_superstructure.setState(StructureState.JITTER_AND_SHOOT))
                .andThen(Commands.waitSeconds(3)));

    bottomInverseAutoAggressive.done().onTrue(bottomInverseAutoAggressivept2.cmd());

    bottomInverseAutoAggressivept2.atTime("Intake").onTrue(m_superstructure.setState(StructureState.INTAKE));
    bottomInverseAutoAggressivept2
        .atTime("StopIntake")
        .onTrue(m_superstructure.setState(StructureState.IDLE));
    bottomInverseAutoAggressivept2
        .atTime("Shoot")
        .onTrue(
            m_superstructure
                .setState(StructureState.SHOOT)
                .andThen(Commands.waitSeconds(1))
                .andThen(m_superstructure.setState(StructureState.JITTER_AND_SHOOT))
                .andThen(Commands.waitSeconds(3)));

    bottomInverseAutoAggressivept2.done().onTrue(bottomInverseAutoAggressivept3.cmd());

    bottomInverseAutoAggressivept3.atTime("Intake").onTrue(m_superstructure.setState(StructureState.INTAKE));

    return routine;
  }

  public AutoRoutine bottomInverseAutoDeepAggressive() {
    final AutoRoutine routine = m_factory.newRoutine("bottomInverseAutoDeepAggressive");
    final AutoTrajectory bottomInverseAutoDeepAggressive = routine.trajectory("bottomInverseAutoDeepAggressive");
    final AutoTrajectory bottomInverseAutoDeepAggressivept2 = routine.trajectory("bottomInverseAutoDeepAggressivept2");
    final AutoTrajectory bottomInverseAutoDeepAggressivept3 = routine.trajectory("bottomInverseAutoDeepAggressivept3");

    routine.active().onTrue(bottomInverseAutoDeepAggressive.resetOdometry().andThen(bottomInverseAutoDeepAggressive.cmd()));

    bottomInverseAutoDeepAggressive.atTime("Intake").onTrue(m_superstructure.setState(StructureState.INTAKE));
    bottomInverseAutoDeepAggressive.atTime("StopIntake").onTrue(m_superstructure.setState(StructureState.IDLE));
    bottomInverseAutoDeepAggressive
        .atTime("Shoot")
        .onTrue(
            m_superstructure
                .setState(StructureState.SHOOT)
                .andThen(Commands.waitSeconds(1))
                .andThen(m_superstructure.setState(StructureState.JITTER_AND_SHOOT))
                .andThen(Commands.waitSeconds(3)));

    bottomInverseAutoDeepAggressive.done().onTrue(bottomInverseAutoDeepAggressivept2.cmd());

    bottomInverseAutoDeepAggressivept2.atTime("Intake").onTrue(m_superstructure.setState(StructureState.INTAKE));
    bottomInverseAutoDeepAggressivept2
        .atTime("StopIntake")
        .onTrue(m_superstructure.setState(StructureState.IDLE));
    bottomInverseAutoDeepAggressivept2
        .atTime("Shoot")
        .onTrue(
            m_superstructure
                .setState(StructureState.SHOOT)
                .andThen(Commands.waitSeconds(1))
                .andThen(m_superstructure.setState(StructureState.JITTER_AND_SHOOT))
                .andThen(Commands.waitSeconds(3)));

    bottomInverseAutoDeepAggressivept2.done().onTrue(bottomInverseAutoDeepAggressivept3.cmd());

    bottomInverseAutoDeepAggressivept3.atTime("Intake").onTrue(m_superstructure.setState(StructureState.INTAKE));

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
    topInverseAutopt2.atTime("StopIntake").onTrue(m_superstructure.setState(StructureState.IDLE));
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

    public AutoRoutine topInverseAutoDeep() {
    final AutoRoutine routine = m_factory.newRoutine("topInverseAutoDeep");
    final AutoTrajectory topInverseAutoDeep = routine.trajectory("topInverseAutoDeep");
    final AutoTrajectory topInverseAutoDeeppt2 = routine.trajectory("topInverseAutoDeeppt2");
    final AutoTrajectory topInverseAutoDeeppt3 = routine.trajectory("topInverseAutoDeeppt3");

    routine.active().onTrue(topInverseAutoDeep.resetOdometry().andThen(topInverseAutoDeep.cmd()));

    topInverseAutoDeep.atTime("Intake").onTrue(m_superstructure.setState(StructureState.INTAKE));
    topInverseAutoDeep.atTime("StopIntake").onTrue(m_superstructure.setState(StructureState.IDLE));
    topInverseAutoDeep
        .atTime("Shoot")
        .onTrue(
            m_superstructure
                .setState(StructureState.SHOOT)
                .andThen(Commands.waitSeconds(1))
                .andThen(m_superstructure.setState(StructureState.JITTER_AND_SHOOT))
                .andThen(Commands.waitSeconds(3)));

    topInverseAutoDeep.done().onTrue(topInverseAutoDeeppt2.cmd());

    topInverseAutoDeeppt2.atTime("Intake").onTrue(m_superstructure.setState(StructureState.INTAKE));
    topInverseAutoDeeppt2
        .atTime("StopIntake")
        .onTrue(m_superstructure.setState(StructureState.IDLE));
    topInverseAutoDeeppt2
        .atTime("Shoot")
        .onTrue(
            m_superstructure
                .setState(StructureState.SHOOT)
                .andThen(Commands.waitSeconds(1))
                .andThen(m_superstructure.setState(StructureState.JITTER_AND_SHOOT))
                .andThen(Commands.waitSeconds(3)));

    topInverseAutoDeeppt2.done().onTrue(topInverseAutoDeeppt3.cmd());

    topInverseAutoDeeppt3.atTime("Intake").onTrue(m_superstructure.setState(StructureState.INTAKE));

    return routine;
  }

  public AutoRoutine topInverseAutoAggressive() {
    final AutoRoutine routine = m_factory.newRoutine("topInverseAutoAggressive");
    final AutoTrajectory topInverseAutoAggressive = routine.trajectory("topInverseAutoAggressive");
    final AutoTrajectory topInverseAutoAggressivept2 = routine.trajectory("topInverseAutoAggressivept2");
    final AutoTrajectory topInverseAutoAggressivept3 = routine.trajectory("topInverseAutoAggressivept3");

    routine.active().onTrue(topInverseAutoAggressive.resetOdometry().andThen(topInverseAutoAggressive.cmd()));

    topInverseAutoAggressive.atTime("Intake").onTrue(m_superstructure.setState(StructureState.INTAKE));
    topInverseAutoAggressive.atTime("StopIntake").onTrue(m_superstructure.setState(StructureState.IDLE));
    topInverseAutoAggressive
        .atTime("Shoot")
        .onTrue(
            m_superstructure
                .setState(StructureState.SHOOT)
                .andThen(Commands.waitSeconds(1))
                .andThen(m_superstructure.setState(StructureState.JITTER_AND_SHOOT))
                .andThen(Commands.waitSeconds(3)));

    topInverseAutoAggressive.done().onTrue(topInverseAutoAggressivept2.cmd());

    topInverseAutoAggressivept2.atTime("Intake").onTrue(m_superstructure.setState(StructureState.INTAKE));
    topInverseAutoAggressivept2
        .atTime("StopIntake")
        .onTrue(m_superstructure.setState(StructureState.IDLE));
    topInverseAutoAggressivept2
        .atTime("Shoot")
        .onTrue(
            m_superstructure
                .setState(StructureState.SHOOT)
                .andThen(Commands.waitSeconds(1))
                .andThen(m_superstructure.setState(StructureState.JITTER_AND_SHOOT))
                .andThen(Commands.waitSeconds(3)));

    topInverseAutoAggressivept2.done().onTrue(topInverseAutoAggressivept3.cmd());

    topInverseAutoAggressivept3.atTime("Intake").onTrue(m_superstructure.setState(StructureState.INTAKE));

    return routine;
  }


  public AutoRoutine topInverseAutoDeepAggressive() {
    final AutoRoutine routine = m_factory.newRoutine("topInverseAutoDeepAggressive");
    final AutoTrajectory topInverseAutoDeepAggressive = routine.trajectory("topInverseAutoDeepAggressive");
    final AutoTrajectory topInverseAutoDeepAggressivept2 = routine.trajectory("topInverseAutoDeepAggressivept2");
    final AutoTrajectory topInverseAutoDeepAggressivept3 = routine.trajectory("topInverseAutoDeepAggressivept3");

    routine.active().onTrue(topInverseAutoDeepAggressive.resetOdometry().andThen(topInverseAutoDeepAggressive.cmd()));

    topInverseAutoDeepAggressive.atTime("Intake").onTrue(m_superstructure.setState(StructureState.INTAKE));
    topInverseAutoDeepAggressive.atTime("StopIntake").onTrue(m_superstructure.setState(StructureState.IDLE));
    topInverseAutoDeepAggressive
        .atTime("Shoot")
        .onTrue(
            m_superstructure
                .setState(StructureState.SHOOT)
                .andThen(Commands.waitSeconds(1))
                .andThen(m_superstructure.setState(StructureState.JITTER_AND_SHOOT))
                .andThen(Commands.waitSeconds(3)));

    topInverseAutoDeepAggressive.done().onTrue(topInverseAutoDeepAggressivept2.cmd());

    topInverseAutoDeepAggressivept2.atTime("Intake").onTrue(m_superstructure.setState(StructureState.INTAKE));
    topInverseAutoDeepAggressivept2
        .atTime("StopIntake")
        .onTrue(m_superstructure.setState(StructureState.IDLE));
    topInverseAutoDeepAggressivept2
        .atTime("Shoot")
        .onTrue(
            m_superstructure
                .setState(StructureState.SHOOT)
                .andThen(Commands.waitSeconds(1))
                .andThen(m_superstructure.setState(StructureState.JITTER_AND_SHOOT))
                .andThen(Commands.waitSeconds(3)));

    topInverseAutoDeepAggressivept2.done().onTrue(topInverseAutoDeepAggressivept3.cmd());

    topInverseAutoDeepAggressivept3.atTime("Intake").onTrue(m_superstructure.setState(StructureState.INTAKE));

    return routine;
  }
}
