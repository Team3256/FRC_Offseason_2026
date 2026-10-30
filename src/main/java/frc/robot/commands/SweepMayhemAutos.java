// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.StructureState;
import frc.robot.subsystems.sotm.ShotCalculator;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import mayhemlib.auto.AutoFactory;
import mayhemlib.auto.AutoRoutine;
import mayhemlib.auto.AutoTrajectory;

public class SweepMayhemAutos {

  private final AutoFactory m_factory;
  private final Superstructure m_superstructure;
  private final CommandSwerveDrivetrain m_drivetrain;
  private final ShotCalculator m_shotCalculator;

  public SweepMayhemAutos(
      AutoFactory factory,
      Superstructure superstructure,
      CommandSwerveDrivetrain drivetrain,
      ShotCalculator shotCalculator) {
    m_factory = factory;
    m_superstructure = superstructure;
    m_drivetrain = drivetrain;
    m_shotCalculator = shotCalculator;
  }

  private Command alignAndShoot() {
    return Commands.parallel(
            m_drivetrain.rotateToLookahead(m_shotCalculator, () -> 0.0, () -> 0.0),
            Commands.waitUntil(
                    () ->
                        ShotCalculator.isAimed(
                            m_drivetrain.getState().Pose.getRotation(),
                            m_shotCalculator.getDriveAngle(),
                            Math.toRadians(3)))
                .andThen(m_superstructure.setState(StructureState.SHOOT)))
        .withTimeout(3.5);
  }

  // 1. Bottom Trench: Part 1 only
  public AutoRoutine bottomTrenchPart1Only() {
    AutoRoutine routine = m_factory.newRoutine("OutpostTrenchSweepBumpPt1");

    AutoTrajectory pt1 = routine.trajectory("OutpostTrenchSweepBumpPt1");

    routine.active().onTrue(pt1.resetOdometry().andThen(pt1.cmd()));

    pt1.atTime("intake").onTrue(m_superstructure.setState(StructureState.INTAKE));
    pt1.atTime("idle").onTrue(m_superstructure.setState(StructureState.IDLE));

    pt1.done().onTrue(alignAndShoot());

    return routine;
  }

  // 2. Bottom Trench: Parts 1 and 2
  public AutoRoutine bottomTrenchPart1And2() {
    AutoRoutine routine = m_factory.newRoutine("OutpostTrenchSweepPart1And2");

    AutoTrajectory pt1 = routine.trajectory("OutpostTrenchSweepBumpPt1");
    AutoTrajectory pt2 = routine.trajectory("OutpostTrenchSweepBumpPt2");

    routine.active().onTrue(pt1.resetOdometry().andThen(pt1.cmd()));

    pt1.atTime("intake").onTrue(m_superstructure.setState(StructureState.INTAKE));
    pt1.atTime("idle").onTrue(m_superstructure.setState(StructureState.IDLE));

    pt1.done().onTrue(alignAndShoot().andThen(pt2.cmd()));

    pt2.atTime("intake2").onTrue(m_superstructure.setState(StructureState.INTAKE));
    pt2.atTime("idle2").onTrue(m_superstructure.setState(StructureState.IDLE));

    pt2.done().onTrue(alignAndShoot());

    return routine;
  }

  // 3. Top Trench: Part 1 only
  public AutoRoutine topTrenchPart1Only() {
    AutoRoutine routine = m_factory.newRoutine("DepotTrenchPart1Only");

    AutoTrajectory pt1 = routine.trajectory("DepotTrenchSweepBumpPt1");

    routine.active().onTrue(pt1.resetOdometry().andThen(pt1.cmd()));

    pt1.atTime("intake").onTrue(m_superstructure.setState(StructureState.INTAKE));
    pt1.atTime("idle").onTrue(m_superstructure.setState(StructureState.IDLE));

    pt1.done().onTrue(alignAndShoot());

    return routine;
  }

  // 4. Top Trench: Parts 1 and 2
  public AutoRoutine topTrenchPart1And2() {
    AutoRoutine routine = m_factory.newRoutine("DepotTrenchPart1And2");

    AutoTrajectory pt1 = routine.trajectory("DepotTrenchSweepBumpPt1");
    AutoTrajectory pt2 = routine.trajectory("DepotTrenchSweepBump2xPt2");

    routine.active().onTrue(pt1.resetOdometry().andThen(pt1.cmd()));

    pt1.atTime("intake").onTrue(m_superstructure.setState(StructureState.INTAKE));
    pt1.atTime("idle").onTrue(m_superstructure.setState(StructureState.IDLE));

    pt1.done().onTrue(alignAndShoot().andThen(pt2.cmd()));

    pt2.atTime("intake2").onTrue(m_superstructure.setState(StructureState.INTAKE));
    pt2.atTime("idle2").onTrue(m_superstructure.setState(StructureState.IDLE));

    pt2.done().onTrue(alignAndShoot());

    return routine;
  }
}
