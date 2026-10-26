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

/**
 * Mayhem version of the top trench sweep depot auto. The paths live in {@code
 * src/main/deploy/mayhem} (open that folder as a project in the Mayhem app); the routine logic is
 * the same as the Choreo version in {@link AutoRoutines}.
 */
public class SweepDepotMayhemAutos {

  private final AutoFactory m_factory;
  private final Superstructure m_superstructure;
  private final CommandSwerveDrivetrain m_drivetrain;
  private final ShotCalculator m_shotCalculator;

  public SweepDepotMayhemAutos(
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

  public AutoRoutine topTrenchSweepDepot() {
    final AutoRoutine routine = m_factory.newRoutine("topTrenchSweepDepot");
    final AutoTrajectory topTrenchSweepDepotAuto = routine.trajectory("DepotTrenchSweepDepotPt");
    final AutoTrajectory topTrenchSweepDepotPt2 = routine.trajectory("DepotTrenchSweepDepotPt2");
    // The Choreo version also loads "topTrenchSweepDepotPt3", but that path was deleted, and
    // MayhemLib throws if a trajectory file is missing, so it is left out here.

    routine
        .active()
        .onTrue(topTrenchSweepDepotAuto.resetOdometry().andThen(topTrenchSweepDepotAuto.cmd()));

    topTrenchSweepDepotAuto
        .atTime("intake")
        .onTrue(m_superstructure.setState(StructureState.INTAKE));
    topTrenchSweepDepotAuto.atTime("idle").onTrue(m_superstructure.setState(StructureState.IDLE));

    topTrenchSweepDepotAuto.done().onTrue(alignAndShoot().andThen(topTrenchSweepDepotPt2.cmd()));

    topTrenchSweepDepotPt2
        .atTime("intake")
        .onTrue(m_superstructure.setState(StructureState.INTAKE));
    topTrenchSweepDepotPt2.done().onTrue(alignAndShoot());

    return routine;
  }
}
