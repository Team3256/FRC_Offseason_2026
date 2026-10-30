// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import choreo.Choreo;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.StructureState;
import frc.robot.subsystems.sotm.ShotCalculator;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import java.util.ArrayList;
import java.util.List;

public class AutoRoutines {

  private final AutoFactory m_factory;

  // Subsystems
  private final Superstructure m_superstructure;
  private final CommandSwerveDrivetrain m_drivetrain;
  private final ShotCalculator m_shotCalculator;
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

  public AutoRoutines(
      AutoFactory factory,
      CommandSwerveDrivetrain drivetrain,
      Superstructure superstructure,
      ShotCalculator shotCalculator) {
    m_factory = factory;
    m_drivetrain = drivetrain;
    m_superstructure = superstructure;
    m_shotCalculator = shotCalculator;
  }

  public AutoRoutine topTrenchSweepAuto() {
    final AutoRoutine routine = m_factory.newRoutine("topTrenchSweepAuto");
    final AutoTrajectory topTrenchSweepAuto = routine.trajectory("topTrenchSweepBump");

    routine.active().onTrue(topTrenchSweepAuto.resetOdometry().andThen(topTrenchSweepAuto.cmd()));
    topTrenchSweepAuto.atTime("intake").onTrue(m_superstructure.setState(StructureState.INTAKE));
    topTrenchSweepAuto.atTime("idle").onTrue(m_superstructure.setState(StructureState.IDLE));

    topTrenchSweepAuto.done().onTrue(alignAndShoot());

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

    bottomTrenchSweepAuto.done().onTrue(alignAndShoot());

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
    // topTrenchSweep2xAutoPt1.atTime("shoot").onTrue(m_superstructure.setState(StructureState.SHOOT));

    topTrenchSweep2xAutoPt1.done().onTrue(alignAndShoot().andThen(topTrenchSweep2xAutoPt2.cmd()));

    topTrenchSweep2xAutoPt2
        .atTime("intake2")
        .onTrue(m_superstructure.setState(StructureState.INTAKE));
    topTrenchSweep2xAutoPt2.atTime("idle2").onTrue(m_superstructure.setState(StructureState.IDLE));

    topTrenchSweep2xAutoPt2.done().onTrue(alignAndShoot());

    return routine;
  }

  public AutoRoutine topTrenchSweepDepot() {
    final AutoRoutine routine = m_factory.newRoutine("topTrenchSweepDepot");
    final AutoTrajectory topTrenchSweepDepotAuto = routine.trajectory("topTrenchSweepDepotPt1");
    final AutoTrajectory topTrenchSweepDepotPt2 = routine.trajectory("topTrenchSweepDepotPt2");

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
        .done()
        .onTrue(alignAndShoot().andThen(bottomTrenchSweep2xAutoPt2.cmd()));

    bottomTrenchSweep2xAutoPt2
        .atTime("intake2")
        .onTrue(m_superstructure.setState(StructureState.INTAKE));
    bottomTrenchSweep2xAutoPt2
        .atTime("idle2")
        .onTrue(m_superstructure.setState(StructureState.IDLE));

    bottomTrenchSweep2xAutoPt2.done().onTrue(alignAndShoot());

    return routine;
  }

  public Pose2d getInitialPose(String trajectoryName) {
    var trajectory = Choreo.loadTrajectory(trajectoryName);
    Pose2d initialPose = trajectory.get().getInitialPose(false).get();
    return initialPose;
  }

  public void updateField2d(Field2d field2d, List<String> trajectoryNames) {
    ArrayList<Pose2d> poseList = new ArrayList<>();
    for (String t : trajectoryNames) {
      var trajectory = Choreo.loadTrajectory(t);

      if (trajectory.isEmpty()) {
        poseList.add(new Pose2d(0, 0, Rotation2d.kZero));
      } else {
        var poses = trajectory.get().getPoses();
        for (int i = 0; i < poses.length; i += 5) {
          poseList.add(poses[i]);
        }
      }
    }

    field2d.setRobotPose(poseList.get(0));
    System.out.println(poseList.size());

    if (poseList.size() == 1) {
      return;
    }

    field2d.getObject("traj").setPoses(poseList);
  }

  private boolean isRedAlliance() {
    return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
  }
}
