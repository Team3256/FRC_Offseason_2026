// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import java.util.ArrayList;
import java.util.List;
import mayhemlib.auto.AutoFactory;
import mayhemlib.auto.AutoRoutine;
import mayhemlib.auto.AutoTrajectory;
import mayhemlib.trajectory.TrajectoryLoader;

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

  public AutoRoutine testAutoRoutine() {
    final AutoRoutine routine = m_factory.newRoutine("Test Auto Routine");
    final AutoTrajectory traj = routine.trajectory("DepotAggroFirstLeg");
    final AutoTrajectory traj2 = routine.trajectory("DepotAggroSecondLeg");
    final AutoTrajectory traj3 = routine.trajectory("DepotHalfLeg");

    routine.active().onTrue(traj.resetOdometry().andThen(traj.cmd()));
    traj.done().onTrue(traj2.cmd());
    traj2.done().onTrue(traj3.cmd());
    return routine;
  }

  public Pose2d getInitialPose(String trajectoryName) {

    var trajectory = TrajectoryLoader.load(trajectoryName);

    Pose2d initialPose = trajectory.initialPose();

    return initialPose;
  }

  public void updateField2d(Field2d field2d, List<String> trajectoryNames) {

    ArrayList<Pose2d> poseList = new ArrayList<>();
    for (String t : trajectoryNames) {
      var trajectory = TrajectoryLoader.load(t);

      if (trajectory == null) {
        poseList.add(new Pose2d(0, 0, Rotation2d.kZero));
      } else {

        var samples = trajectory.samples();
        //        var poses = trajectory.get().getPoses();

        for (int i = 0; i < samples.size(); i += 5) {
          poseList.add(samples.get(i).getPose());
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
