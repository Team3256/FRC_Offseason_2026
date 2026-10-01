// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.sotm;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.FieldConstants;
import org.junit.jupiter.api.Test;

class ShotCalculatorSolveTest {
  private static final Translation2d kHub = FieldConstants.Hub.topCenterPoint.toTranslation2d();
  private static final double kStartDistance = 3.0;

  /** Runs one solve with the robot {@code kStartDistance} meters from the hub along -x. */
  private static ShotCalculator solve(
      Rotation2d heading, ChassisSpeeds fieldVelocity, Transform2d robotToShooter) {
    Pose2d pose = new Pose2d(kHub.minus(new Translation2d(kStartDistance, 0)), heading);
    ShotCalculator calculator =
        new ShotCalculator(() -> pose, () -> fieldVelocity, robotToShooter);
    calculator.periodic();
    return calculator;
  }

  private static Transform2d noOffset() {
    return new Transform2d(0, 0, Rotation2d.kZero);
  }

  @Test
  void distanceIsSafeBeforeFirstPeriodic() {
    ShotCalculator calculator =
        new ShotCalculator(Pose2d::new, ChassisSpeeds::new, noOffset());
    calculator.getDistance();
  }

  @Test
  void stationaryRobotAimsStraightAtTarget() {
    ShotCalculator calculator = solve(Rotation2d.kZero, new ChassisSpeeds(), noOffset());

    assertEquals(kStartDistance, calculator.getDistance(), 1e-9);
    // The shooter faces out the back, so the robot heading is opposite the bearing to the target.
    assertEquals(Math.PI, Math.abs(calculator.getDriveAngle().getRadians()), 1e-9);
  }

  @Test
  void leadDoesNotDependOnRobotHeading() {
    // Same field-relative velocity, different headings: the shot solution must be identical.
    ChassisSpeeds towardHub = new ChassisSpeeds(2.0, 0.5, 0);
    ShotCalculator facingForward = solve(Rotation2d.kZero, towardHub, noOffset());

    for (double degrees : new double[] {90, 180, -90}) {
      ShotCalculator rotated = solve(Rotation2d.fromDegrees(degrees), towardHub, noOffset());
      assertEquals(facingForward.getDistance(), rotated.getDistance(), 1e-9, degrees + " deg");
      assertEquals(
          facingForward.getDriveAngle().getRadians(),
          rotated.getDriveAngle().getRadians(),
          1e-9,
          degrees + " deg");
    }
  }

  @Test
  void drivingTowardTargetShortensEffectiveDistance() {
    ShotCalculator calculator =
        solve(Rotation2d.kZero, new ChassisSpeeds(2.0, 0, 0), noOffset());

    assertTrue(calculator.getDistance() < kStartDistance - 1.5);
  }

  @Test
  void drivingSidewaysAimsAgainstTheMotion() {
    ShotCalculator calculator =
        solve(Rotation2d.kZero, new ChassisSpeeds(0, 2.0, 0), noOffset());

    // Robot drifts +y, so the ball must be aimed toward -y: the bearing to the target rotates
    // toward -y, and the heading (opposite the bearing) sits just short of pi.
    double headingOffFromBack = calculator.getDriveAngle().minus(Rotation2d.kPi).getRadians();
    assertTrue(headingOffFromBack < 0, "headingOffFromBack=" + headingOffFromBack);
  }

  @Test
  void spinningOffsetShooterLeadsWithItsOwnVelocity() {
    // Shooter 1 m to the robot's left. Spinning CCW at 1 rad/s moves it toward -x (away from the
    // hub, which is at +x), so the lookahead must land behind where the shooter is now.
    Transform2d leftOfCenter = new Transform2d(0, 1.0, Rotation2d.kZero);
    ShotCalculator calculator = solve(Rotation2d.kZero, new ChassisSpeeds(0, 0, 1.0), leftOfCenter);

    double robotX = kHub.getX() - kStartDistance;
    assertTrue(calculator.getLookaheadPose().getX() < robotX - 0.5);
  }
}
