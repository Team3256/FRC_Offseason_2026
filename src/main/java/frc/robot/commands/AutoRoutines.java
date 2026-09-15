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
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.StructureState;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import java.util.ArrayList;
import java.util.List;

public class AutoRoutines {

  private final AutoFactory m_factory;

  // subsystems
  private final Superstructure m_superstructure;
  private final CommandSwerveDrivetrain m_drivetrain;

  public AutoRoutines(
      AutoFactory factory, CommandSwerveDrivetrain drivetrain, Superstructure superstructure) {
    m_factory = factory;
    m_drivetrain = drivetrain; // subsystems
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
