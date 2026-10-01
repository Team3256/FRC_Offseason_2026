// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.FieldConstants;
import frc.robot.subsystems.swerve.SwerveConstants.TrenchAssistConstants;
import org.junit.jupiter.api.Test;

/** Drives a simple kinematic robot through every trench with the driver only pushing the stick. */
class TrenchAssistTest {
  private static final double kDt = 0.005;
  // The drivetrain does not track a command instantly
  private static final double kDriveLag = 0.25;
  private static final double kRobotHalfWidth = Units.inchesToMeters(16.5);
  // Room each side of the robot when it is dead center in the lane; touching a wall is not a
  // crash, so a centimeter of tolerance keeps wall contact from tripping float rounding
  private static final double kClearance =
      FieldConstants.LeftTrench.openingWidth / 2.0 - kRobotHalfWidth + 0.01;

  private static ChassisSpeeds driver(double vx, double vy) {
    return new ChassisSpeeds(vx, vy, 0);
  }

  /**
   * Drives the robot at constant stick input past one trench; returns the worst lateral offset and
   * the worst heading error (from 0 or 180 degrees) while the robot overlaps that trench.
   */
  private static double[] worstInTrench(
      double startX,
      double startY,
      double startHeading,
      double vx,
      double vy,
      double trenchX,
      double laneY) {
    Pose2d pose = new Pose2d(startX, startY, Rotation2d.fromRadians(startHeading));
    ChassisSpeeds actual = new ChassisSpeeds(vx, 0, 0);
    double worstOffset = 0;
    double worstHeading = 0;
    for (int i = 0; i < 4000; i++) {
      ChassisSpeeds cmd = TrenchAssist.apply(pose, driver(vx, vy)).speeds();
      double alpha = kDt / (kDriveLag + kDt);
      actual =
          new ChassisSpeeds(
              actual.vxMetersPerSecond + alpha * (cmd.vxMetersPerSecond - actual.vxMetersPerSecond),
              actual.vyMetersPerSecond + alpha * (cmd.vyMetersPerSecond - actual.vyMetersPerSecond),
              actual.omegaRadiansPerSecond
                  + alpha * (cmd.omegaRadiansPerSecond - actual.omegaRadiansPerSecond));
      // The field perimeter stops the robot, like it would a real driver
      double y =
          MathUtil.clamp(
              pose.getY() + actual.vyMetersPerSecond * kDt,
              kRobotHalfWidth,
              FieldConstants.fieldWidth - kRobotHalfWidth);
      pose =
          new Pose2d(
              pose.getX() + actual.vxMetersPerSecond * kDt,
              y,
              pose.getRotation().plus(Rotation2d.fromRadians(actual.omegaRadiansPerSecond * kDt)));

      // Only the part where the robot is actually between the walls matters
      if (Math.abs(pose.getX() - trenchX)
          <= FieldConstants.LeftTrench.depth / 2.0 + TrenchAssistConstants.kRobotHalfLength) {
        worstOffset = Math.max(worstOffset, Math.abs(pose.getY() - laneY));
        double heading = pose.getRotation().getRadians();
        worstHeading =
            Math.max(
                worstHeading,
                Math.abs(MathUtil.angleModulus(heading - Math.round(heading / Math.PI) * Math.PI)));
      }
    }
    return new double[] {worstOffset, worstHeading};
  }

  @Test
  void centersFromAnyOffsetAndDirection() {
    double reach = 6.0;
    for (double centerX : TrenchAssist.trenchCenterXs()) {
      for (double laneY : TrenchAssist.laneCenterYs()) {
        for (double offset : new double[] {-0.5, -0.25, 0.25, 0.5}) {
          for (double heading : new double[] {0, 0.5, -0.5, Math.PI, Math.PI + 0.4}) {
            for (double dir : new double[] {1, -1}) {
              for (double speed : new double[] {1.5, 3.0, 5.0}) {
                double[] worst =
                    worstInTrench(
                        centerX - dir * reach,
                        laneY + offset,
                        heading,
                        dir * speed,
                        0,
                        centerX,
                        laneY);
                String where =
                    String.format(
                        "x=%.2f lane=%.2f off=%.2f hdg=%.2f dir=%.0f v=%.1f",
                        centerX, laneY, offset, heading, dir, speed);
                assertTrue(worst[0] < kClearance * 0.5, where + ": offset " + worst[0] + " m");
                assertTrue(
                    worst[1] < Math.toRadians(15), where + ": heading error " + worst[1] + " rad");
              }
            }
          }
        }
      }
    }
  }

  @Test
  void centersWhenDriverApproachesAtAnAngle() {
    double vx = 3.5;
    for (double centerX : TrenchAssist.trenchCenterXs()) {
      for (double laneY : TrenchAssist.laneCenterYs()) {
        // Sloppy aim (+/- 0.3 m) is fine up to ~16 degrees off the trench axis; at ~27 degrees the
        // driver has to be within +/- 0.1 m, since the capture funnel is only so wide
        for (double vy : new double[] {-1.75, -1.0, -0.5, 0.5, 1.0, 1.75}) {
          double maxAimError = Math.abs(vy) > 1.0 ? 0.1 : 0.3;
          for (double aimError : new double[] {-maxAimError, 0, maxAimError}) {
            double startY = laneY - vy * 5.0 / vx + aimError;
            if (startY < kRobotHalfWidth || startY > FieldConstants.fieldWidth - kRobotHalfWidth) {
              continue;
            }
            double[] worst = worstInTrench(centerX - 5.0, startY, 0, vx, vy, centerX, laneY);
            String where = String.format("vy=%.2f lane=%.2f aim error=%.1f", vy, laneY, aimError);
            assertTrue(worst[0] < kClearance, where + ": offset " + worst[0] + " m");
          }
        }
      }
    }
  }

  @Test
  void doesNothingAwayFromTrenches() {
    ChassisSpeeds in = new ChassisSpeeds(4.0, 1.0, 0.3);
    // Middle of the neutral zone, and on the bump between the trenches
    for (Pose2d pose :
        new Pose2d[] {
          new Pose2d(FieldConstants.fieldLength / 2.0, 4.0, Rotation2d.kZero),
          new Pose2d(FieldConstants.LinesVertical.hubCenter - 1.0, 2.4, Rotation2d.kZero),
          new Pose2d(FieldConstants.LinesVertical.hubCenter - 1.0, 5.6, Rotation2d.kZero),
          new Pose2d(FieldConstants.LinesVertical.hubCenter, 4.0, Rotation2d.kZero),
        }) {
      var out = TrenchAssist.apply(pose, in);
      assertEquals(0.0, out.weight());
      assertEquals(in.vyMetersPerSecond, out.speeds().vyMetersPerSecond);
      assertEquals(in.omegaRadiansPerSecond, out.speeds().omegaRadiansPerSecond);
    }
  }

  @Test
  void leavesDriverAloneWhenSlowOrDrivingAway() {
    double laneY = TrenchAssist.laneCenterYs()[0];
    double x = FieldConstants.LinesVertical.hubCenter - 1.5;
    var slow = TrenchAssist.apply(new Pose2d(x, laneY + 0.3, Rotation2d.kZero), driver(0.3, 0.2));
    assertEquals(0.0, slow.weight());
    var away = TrenchAssist.apply(new Pose2d(x, laneY + 0.3, Rotation2d.kZero), driver(-4.0, 0.2));
    assertEquals(0.0, away.weight());
  }

  @Test
  void driverKeepsHeadingControlWhenTurning() {
    double laneY = TrenchAssist.laneCenterYs()[0];
    var pose =
        new Pose2d(FieldConstants.LinesVertical.hubCenter, laneY, Rotation2d.fromDegrees(40));
    ChassisSpeeds turning = new ChassisSpeeds(2.0, 0, 2.0);
    assertEquals(2.0, TrenchAssist.apply(pose, turning).speeds().omegaRadiansPerSecond);
    // Not turning: it squares the robot up (40 degrees ccw of 0 -> turn clockwise)
    assertTrue(TrenchAssist.apply(pose, driver(2.0, 0)).speeds().omegaRadiansPerSecond < 0);
  }

  @Test
  void staysOutOfTheWayWhenShootingFromUnderTheTrench() {
    double laneY = TrenchAssist.laneCenterYs()[1];
    double centerX = TrenchAssist.trenchCenterXs()[0];
    // Off-center and crooked, which is fine when the driver is parked or nudging around
    for (double xOffset : new double[] {-1.5, -0.5, 0.0, 0.4}) {
      var pose =
          new Pose2d(centerX + xOffset, laneY - 0.15, Rotation2d.fromDegrees(25));
      for (ChassisSpeeds stick :
          new ChassisSpeeds[] {
            driver(0, 0), driver(0.4, 0), driver(-0.8, 0.1), driver(0.2, 0.9), driver(0, -1.0)
          }) {
        var out = TrenchAssist.apply(pose, stick);
        assertEquals(0.0, out.weight(), "x offset " + xOffset + " stick " + stick);
        assertEquals(stick.vyMetersPerSecond, out.speeds().vyMetersPerSecond);
        assertEquals(stick.omegaRadiansPerSecond, out.speeds().omegaRadiansPerSecond);
      }
    }
  }

  @Test
  void releasesSmoothlyAsTheDriverSlowsInsideTheTrench() {
    double laneY = TrenchAssist.laneCenterYs()[0];
    var pose =
        new Pose2d(TrenchAssist.trenchCenterXs()[0], laneY + 0.1, Rotation2d.kZero);
    double previous = 1.0;
    for (double vx = 3.0; vx >= 0.0; vx -= 0.1) {
      double weight = TrenchAssist.apply(pose, driver(vx, 0)).weight();
      assertTrue(weight <= previous + 1e-9, "weight should not rise as the driver slows");
      assertTrue(previous - weight < 0.2, "weight dropped abruptly at vx=" + vx);
      previous = weight;
    }
    assertEquals(0.0, previous);
  }
}
