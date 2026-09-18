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
