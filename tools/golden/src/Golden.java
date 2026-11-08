// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.FieldConstants;
import frc.robot.subsystems.sotm.ShotCalculator;
import frc.robot.subsystems.swerve.TrenchAssist;
import java.io.File;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import mayhemlib.follow.DriveCommand;
import mayhemlib.follow.FollowerConfig;
import mayhemlib.recovery.BridgePlanner;
import mayhemlib.recovery.RecoveryConfig;
import mayhemlib.recovery.UnbeachConfig;
import mayhemlib.runner.TrajectoryRunner;
import mayhemlib.trajectory.MayhemTrajectory;
import mayhemlib.trajectory.TrajectoryEvent;
import mayhemlib.trajectory.TrajectoryLoader;
import mayhemlib.trajectory.TrajectorySample;

/**
 * Generates golden data from the ORIGINAL Java implementation (the MayhemLib jar from the
 * vendordep and the robot's Java TrenchAssist / ShotCalculator / FieldConstants) so the C++ port
 * can be checked for identical behaviour. See tools/golden/README.md.
 *
 * <p>Args: <deploy dir> <output dir>
 */
public final class Golden {
  private static String d(double v) {
    return Double.toString(v);
  }

  public static void main(String[] args) throws IOException {
    String deploy = args[0];
    File out = new File(args[1]);
    out.mkdirs();
    System.setProperty("deploy.dir", deploy);

    trenchAssist(new File(out, "trench_assist.csv"));
    shotCalculator(new File(out, "shot_calculator.csv"));
    fieldConstants(new File(out, "field_constants.csv"));

    File[] files =
        new File(deploy, "mayhem").listFiles((dir, name) -> name.endsWith(TrajectoryLoader.EXTENSION));
    java.util.Arrays.sort(files);
    try (PrintWriter samples = new PrintWriter(new File(out, "mayhem_samples.csv"));
        PrintWriter planner = new PrintWriter(new File(out, "mayhem_planner.csv"));
        PrintWriter runner = new PrintWriter(new File(out, "mayhem_runner.csv"))) {
      samples.println("traj,flipped,t,x,y,heading,vx,vy,omega,ax,ay,alpha,nf,fx0,fx1,fx2,fx3,fy0,fy1,fy2,fy3");
      planner.println(
          "traj,tNow,px,py,ph,vx,vy,w,found,joinTime,duration,midx,midy,midh,midvx,midvy,midw");
      runner.println(
          "traj,flipped,scenario,k,now,px,py,ph,cvx,cvy,cw,fx0,fx1,fx2,fx3,fy0,fy1,fy2,fy3,state,"
              + "trajTime,rate,posErr,headErr,bridges,events");
      Random rng = new Random(3256);
      for (File f : files) {
        String name = f.getName().substring(0, f.getName().length() - TrajectoryLoader.EXTENSION.length());
        MayhemTrajectory blue = TrajectoryLoader.load(name);
        for (boolean flip : new boolean[] {false, true}) {
          MayhemTrajectory t = flip ? blue.flipped() : blue;
          double T = t.totalTime();
          for (int i = 0; i <= 60; i++) {
            double time = -0.1 + (T + 0.2) * i / 60.0;
            TrajectorySample s = t.sampleAt(time);
            StringBuilder sb = new StringBuilder();
            sb.append(name).append(',').append(flip ? 1 : 0).append(',').append(d(time));
            for (double v : new double[] {s.x, s.y, s.heading, s.vx, s.vy, s.omega, s.ax, s.ay, s.alpha}) {
              sb.append(',').append(d(v));
            }
            sb.append(',').append(s.fx.length);
            for (int k = 0; k < 4; k++) sb.append(',').append(d(k < s.fx.length ? s.fx[k] : 0));
            for (int k = 0; k < 4; k++) sb.append(',').append(d(k < s.fy.length ? s.fy[k] : 0));
            samples.println(sb);
          }
          for (String scenario : new String[] {"nominal", "shove", "unbeach"}) {
            if (scenario.equals("unbeach") && flip) {
              continue;
            }
            runScenario(runner, name, t, flip, scenario);
          }
        }
        // planner, blue only
        BridgePlanner bp = new BridgePlanner(robotRecoveryConfig());
        for (int i = 0; i < 25; i++) {
          double tNow = rng.nextDouble() * blue.totalTime();
          TrajectorySample here = blue.sampleAt(tNow);
          Pose2d pose =
              new Pose2d(
                  here.x + (rng.nextDouble() - 0.5) * 1.6,
                  here.y + (rng.nextDouble() - 0.5) * 1.6,
                  new Rotation2d(here.heading + (rng.nextDouble() - 0.5) * 1.2));
          ChassisSpeeds v =
              new ChassisSpeeds(
                  (rng.nextDouble() - 0.5) * 4, (rng.nextDouble() - 0.5) * 4, (rng.nextDouble() - 0.5) * 3);
          var res = bp.plan(blue, tNow, pose, v);
          StringBuilder sb = new StringBuilder();
          sb.append(name).append(',').append(d(tNow)).append(',').append(d(pose.getX())).append(',')
              .append(d(pose.getY())).append(',').append(d(pose.getRotation().getRadians())).append(',')
              .append(d(v.vxMetersPerSecond)).append(',').append(d(v.vyMetersPerSecond)).append(',')
              .append(d(v.omegaRadiansPerSecond)).append(',');
          if (res.bridge.isPresent()) {
            var b = res.bridge.get();
            TrajectorySample m = b.sampleAt(b.duration() / 2);
            sb.append("1,").append(d(b.joinTime)).append(',').append(d(b.duration()));
            for (double val : new double[] {m.x, m.y, m.heading, m.vx, m.vy, m.omega}) {
              sb.append(',').append(d(val));
            }
          } else {
            sb.append("0,0,0,0,0,0,0,0,0");
          }
          planner.println(sb);
        }
      }
    }
  }

  static RecoveryConfig robotRecoveryConfig() {
    RecoveryConfig rc = new RecoveryConfig();
    rc.persistenceLoops = 10; // same as RobotContainer
    return rc;
  }

  /**
   * Closed loop with a simple plant: the robot moves at 92% of the commanded field speeds. In
   * "shove" the robot is knocked 0.55 m / -0.25 m at step 60 (with an accelerometer spike) so bump
   * recovery plans a bridge; in "unbeach" the IMU reports an 11 degree pitch for steps 50-89.
   */
  static void runScenario(
      PrintWriter w, String name, MayhemTrajectory traj, boolean flip, String scenario) {
    TrajectoryRunner.EventListener listener;
    List<String> fired = new ArrayList<>();
    listener =
        new TrajectoryRunner.EventListener() {
          @Override
          public void onEvent(TrajectoryEvent e) {
            fired.add(e.name);
          }

          @Override
          public void onZoneEnd(TrajectoryEvent e) {
            fired.add("end:" + e.name);
          }
        };
    TrajectoryRunner r =
        new TrajectoryRunner(traj, new FollowerConfig(), robotRecoveryConfig(), listener, null);
    if (scenario.equals("unbeach")) {
      r.withUnbeach(new UnbeachConfig());
    }
    final double dt = 0.02;
    final double gain = 0.92;
    Pose2d start = traj.initialPose();
    double x = start.getX() + 0.05;
    double y = start.getY() - 0.03;
    double h = start.getRotation().getRadians() + 0.02;
    ChassisSpeeds measured = new ChassisSpeeds();
    double now = 10.0;
    r.start(now);
    for (int k = 0; k < 1500 && !r.isFinished(); k++) {
      now = 10.0 + (k + 1) * dt;
      double accel = Double.NaN;
      double pitch = Double.NaN;
      double roll = Double.NaN;
      if (scenario.equals("shove") && k == 60) {
        x += 0.55;
        y -= 0.25;
        accel = 2.5;
      }
      if (scenario.equals("unbeach")) {
        pitch = (k >= 50 && k < 90) ? 11.0 : 0.5;
        roll = 0.0;
      }
      fired.clear();
      DriveCommand dc = r.update(now, new Pose2d(x, y, new Rotation2d(h)), measured, accel, pitch, roll);
      StringBuilder sb = new StringBuilder();
      sb.append(name).append(',').append(flip ? 1 : 0).append(',').append(scenario).append(',')
          .append(k).append(',').append(d(now)).append(',').append(d(x)).append(',').append(d(y))
          .append(',').append(d(h)).append(',').append(d(dc.fieldSpeeds.vxMetersPerSecond))
          .append(',').append(d(dc.fieldSpeeds.vyMetersPerSecond)).append(',')
          .append(d(dc.fieldSpeeds.omegaRadiansPerSecond));
      for (int i = 0; i < 4; i++) sb.append(',').append(d(dc.wheelForceX[i]));
      for (int i = 0; i < 4; i++) sb.append(',').append(d(dc.wheelForceY[i]));
      sb.append(',').append(r.state().name()).append(',').append(d(r.trajectoryTime())).append(',')
          .append(d(r.clockRate())).append(',').append(d(r.positionError())).append(',')
          .append(d(r.headingError())).append(',').append(r.bridgesPlanned()).append(',')
          .append(String.join("|", fired));
      w.println(sb);
      x += dc.fieldSpeeds.vxMetersPerSecond * gain * dt;
      y += dc.fieldSpeeds.vyMetersPerSecond * gain * dt;
      h += dc.fieldSpeeds.omegaRadiansPerSecond * gain * dt;
      measured =
          new ChassisSpeeds(
              dc.fieldSpeeds.vxMetersPerSecond * gain,
              dc.fieldSpeeds.vyMetersPerSecond * gain,
              dc.fieldSpeeds.omegaRadiansPerSecond * gain);
    }
  }

  static void trenchAssist(File f) throws IOException {
    Random rng = new Random(1234);
    double[] lanesY = TrenchAssist.laneCenterYs();
    double[] trenchX = TrenchAssist.trenchCenterXs();
    try (PrintWriter w = new PrintWriter(f)) {
      w.println("px,py,ph,vx,vy,w,ovx,ovy,ow,weight");
      for (int i = 0; i < 4000; i++) {
        double px, py;
        if (i % 5 == 0) {
          px = rng.nextDouble() * FieldConstants.fieldLength;
          py = rng.nextDouble() * FieldConstants.fieldWidth;
        } else {
          px = trenchX[rng.nextInt(2)] + (rng.nextDouble() - 0.5) * 5.0;
          py = lanesY[rng.nextInt(2)] + (rng.nextDouble() - 0.5) * 2.0;
        }
        double ph = (rng.nextDouble() - 0.5) * 4 * Math.PI;
        double vx = (rng.nextDouble() - 0.5) * 10;
        double vy = (rng.nextDouble() - 0.5) * (i % 3 == 0 ? 8 : 2);
        double w0 = (rng.nextDouble() - 0.5) * (i % 4 == 0 ? 8 : 0.8);
        var res = TrenchAssist.apply(new Pose2d(px, py, new Rotation2d(ph)), new ChassisSpeeds(vx, vy, w0));
        w.println(
            String.join(
                ",",
                d(px), d(py), d(ph), d(vx), d(vy), d(w0),
                d(res.speeds().vxMetersPerSecond), d(res.speeds().vyMetersPerSecond),
                d(res.speeds().omegaRadiansPerSecond), d(res.weight())));
      }
    }
  }

  static void shotCalculator(File f) throws IOException {
    Random rng = new Random(99);
    Translation2d[] targets = {
      FieldConstants.Hub.topCenterPoint.toTranslation2d(),
      FieldConstants.Hub.oppTopCenterPoint.toTranslation2d(),
      new Translation2d(1.5, 6.8),
      new Translation2d(1.5, 1.5),
      new Translation2d(FieldConstants.fieldLength - 1.5, FieldConstants.fieldWidth - 6.8),
    };
    try (PrintWriter w = new PrintWriter(f)) {
      w.println("px,py,ph,vx,vy,w,tx,ty,lx,ly,lh,distance,driveAngle,tf");
      for (int i = 0; i < 2000; i++) {
        double px = rng.nextDouble() * FieldConstants.fieldLength;
        double py = rng.nextDouble() * FieldConstants.fieldWidth;
        double ph = (rng.nextDouble() - 0.5) * 2 * Math.PI;
        double vx = (rng.nextDouble() - 0.5) * 8;
        double vy = (rng.nextDouble() - 0.5) * 8;
        double w0 = (rng.nextDouble() - 0.5) * 6;
        Translation2d target = targets[rng.nextInt(targets.length)];
        Pose2d pose = new Pose2d(px, py, new Rotation2d(ph));
        ChassisSpeeds speeds = new ChassisSpeeds(vx, vy, w0);
        // the robot code uses a zero robot-to-shooter transform; also cover a non-zero one
        Transform2d robotToShooter =
            i % 2 == 0 ? new Transform2d(0, 0, Rotation2d.kZero) : new Transform2d(0.2, -0.1, new Rotation2d(0.3));
        ShotCalculator sc = new ShotCalculator(() -> pose, () -> speeds, robotToShooter);
        sc.setTarget(target);
        sc.periodic();
        Pose2d la = sc.getLookaheadPose();
        w.println(
            String.join(
                ",",
                d(px), d(py), d(ph), d(vx), d(vy), d(w0), d(target.getX()), d(target.getY()),
                d(la.getX()), d(la.getY()), d(la.getRotation().getRadians()), d(sc.getDistance()),
                d(sc.getDriveAngle().getRadians())) + "," + (i % 2));
      }
    }
  }

  static void fieldConstants(File f) throws IOException {
    try (PrintWriter w = new PrintWriter(f)) {
      w.println("name,value");
      w.println("fieldLength," + d(FieldConstants.fieldLength));
      w.println("fieldWidth," + d(FieldConstants.fieldWidth));
      w.println("aprilTagCount," + FieldConstants.aprilTagCount);
      w.println("hubCenter," + d(FieldConstants.LinesVertical.hubCenter));
      w.println("oppHubCenter," + d(FieldConstants.LinesVertical.oppHubCenter));
      w.println("oppAllianceZone," + d(FieldConstants.LinesVertical.oppAllianceZone));
      w.println("starting," + d(FieldConstants.LinesVertical.starting));
      w.println("rightTrenchOpenStart," + d(FieldConstants.LinesHorizontal.rightTrenchOpenStart));
      w.println("leftTrenchOpenEnd," + d(FieldConstants.LinesHorizontal.leftTrenchOpenEnd));
      w.println("rightBumpStart," + d(FieldConstants.LinesHorizontal.rightBumpStart));
      w.println("leftBumpStart," + d(FieldConstants.LinesHorizontal.leftBumpStart));
      w.println("hubTopX," + d(FieldConstants.Hub.topCenterPoint.getX()));
      w.println("hubTopY," + d(FieldConstants.Hub.topCenterPoint.getY()));
      w.println("oppHubTopX," + d(FieldConstants.Hub.oppTopCenterPoint.getX()));
      w.println("towerCenterY," + d(FieldConstants.Tower.centerPoint.getY()));
      w.println("oppTowerCenterX," + d(FieldConstants.Tower.oppCenterPoint.getX()));
      w.println("oppTowerCenterY," + d(FieldConstants.Tower.oppCenterPoint.getY()));
      w.println("outpostCenterY," + d(FieldConstants.Outpost.centerPoint.getY()));
    }
  }
}
