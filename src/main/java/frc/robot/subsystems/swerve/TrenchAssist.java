// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve;

import static frc.robot.subsystems.swerve.SwerveConstants.TrenchAssistConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.FieldConstants;

/**
 * Trench assist. When the driver heads into a trench, this nudges the robot onto the lane's
 * centerline and squares it up to the walls, so the driver can go through at full speed without
 * clipping the trench structure. It needs no driver input: it is a pure function over the driver's
 * requested speeds and the robot pose.
 *
 * <p>Everything here is in the blue-origin field frame (the same frame as the drivetrain pose). The
 * driver keeps full control of x speed. Lateral speed and heading are blended toward the
 * centering/squaring targets by a weight in [0, 1] that ramps up on approach, is 1 while the robot
 * overlaps the trench, and fades out after it leaves.
 *
 * <p>The assist only engages for a driver actually driving through: moving along the trench axis at
 * speed. A robot that is stopped or creeping under the trench (to shoot, say), or strafing, is left
 * completely alone, and so is a driver who is turning the robot themselves. Heading is only touched
 * when the robot would not clear the walls at its current angle, so a robot that is sideways (or at
 * any other angle that fits) keeps it.
 */
public final class TrenchAssist {
  private TrenchAssist() {}

  /** One trench lane: the x of its center, the y of its centerline. */
  private record Lane(double centerX, double centerY) {}

  private static final double kTrenchHalfDepth = FieldConstants.LeftTrench.depth / 2.0;

  /** Distance from the trench center along x at which the robot body starts overlapping it. */
  private static final double kOverlapHalfDepth = kTrenchHalfDepth + kRobotHalfLength;

  private static final double kRightLaneY = FieldConstants.RightTrench.openingWidth / 2.0;
  private static final double kLeftLaneY =
      FieldConstants.fieldWidth - FieldConstants.LeftTrench.openingWidth / 2.0;

  // Both alliances' trenches, on both sides of the field.
  private static final Lane[] kLanes = {
    new Lane(FieldConstants.LinesVertical.hubCenter, kRightLaneY),
    new Lane(FieldConstants.LinesVertical.hubCenter, kLeftLaneY),
    new Lane(FieldConstants.LinesVertical.oppHubCenter, kRightLaneY),
    new Lane(FieldConstants.LinesVertical.oppHubCenter, kLeftLaneY),
  };

  /** What the assist decided this loop; {@code weight == 0} means the driver is untouched. */
  public record Result(ChassisSpeeds speeds, double weight) {}

  /**
   * Applies trench assist to the driver's requested speeds.
   *
   * @param pose robot pose in the blue-origin field frame
   * @param driverSpeeds the driver's requested speeds, field-relative in the blue-origin frame
   * @return the speeds to command (equal to the input when the assist is not engaged)
   */
  public static Result apply(Pose2d pose, ChassisSpeeds driverSpeeds) {
    Lane best = null;
    double bestWeight = 0.0;
    for (Lane lane : kLanes) {
      double w =
          weight(
              pose.getX(),
              pose.getY(),
              driverSpeeds.vxMetersPerSecond,
              driverSpeeds.vyMetersPerSecond,
              lane);
      if (w > bestWeight) {
        bestWeight = w;
        best = lane;
      }
    }
    if (best == null) {
      return new Result(driverSpeeds, 0.0);
    }

    double centeringVy =
        MathUtil.clamp(
            -kCenteringKP * (pose.getY() - best.centerY()),
            -kMaxCenteringSpeed,
            kMaxCenteringSpeed);
    double vy = MathUtil.interpolate(driverSpeeds.vyMetersPerSecond, centeringVy, bestWeight);

    double omega = driverSpeeds.omegaRadiansPerSecond;
    if (Math.abs(omega) < kDriverTurnOverrideRate) {
      // The robot is square, so any multiple of 90 degrees fits equally well. Only turn it if it
      // would not clear the walls where it is (near a diagonal), so a robot that is sideways to
      // shoot is left alone.
      double heading = pose.getRotation().getRadians();
      double footprintHalf =
          kRobotHalfLength * (Math.abs(Math.cos(heading)) + Math.abs(Math.sin(heading)));
      double slack = kLaneHalfWidth - footprintHalf;
      double snapWeight =
          1.0
              - smoothstep(
                  (slack - kHeadingFullSnapSlack) / (kHeadingFreeSlack - kHeadingFullSnapSlack));
      double target = Math.round(heading / (Math.PI / 2.0)) * (Math.PI / 2.0);
      double squareOmega =
          MathUtil.clamp(
              kHeadingKP * MathUtil.angleModulus(target - heading),
              -SwerveConstants.MaxAngularRate,
              SwerveConstants.MaxAngularRate);
      omega = MathUtil.interpolate(omega, squareOmega, bestWeight * snapWeight);
    }

    return new Result(new ChassisSpeeds(driverSpeeds.vxMetersPerSecond, vy, omega), bestWeight);
  }

  /** How strongly the assist applies for one lane, in [0, 1]. */
  private static double weight(double x, double y, double driverVx, double driverVy, Lane lane) {
    // Is the driver actually driving through? Robots also stop under the trench to shoot, creep
    // around in it, or strafe, and the assist must stay out of the way then. So it only engages
    // for a driver moving along the trench axis at speed, and lets go as they slow or turn aside.
    double speed = Math.abs(driverVx);
    double speedGate = smoothstep((speed - kMinAssistSpeed) / kAssistSpeedBlend);
    if (speedGate <= 0.0) {
      return 0.0;
    }
    double ratio = Math.abs(driverVy) / speed;
    double angleGate = 1.0 - smoothstep((ratio - kApproachAngleTan) / (1.0 - kApproachAngleTan));
    double intent = speedGate * angleGate;
    if (intent <= 0.0) {
      return 0.0;
    }

    double dx = x - lane.centerX();
    double outside = Math.abs(dx) - kOverlapHalfDepth;
    boolean approaching = outside > 0.0 && driverVx * dx < 0.0;

    // Is the driver lined up with this lane? Judged by where they are heading: on approach, how far
    // off the centerline they will be when they reach the entrance. So a driver heading for the
    // bump next to the trench, or just passing by, is not pulled in, while one coming in at a
    // slight angle is.
    double lateralError = Math.abs(y - lane.centerY());
    if (approaching) {
      lateralError = Math.abs(y - lane.centerY() + driverVy * outside / speed);
    }
    double lateralWeight = 1.0 - smoothstep((lateralError - kLaneHalfWidth) / kCaptureMargin);
    if (lateralWeight <= 0.0) {
      return 0.0;
    }

    double axialWeight;
    if (outside <= 0.0) {
      axialWeight = 1.0; // robot overlaps the trench
    } else if (approaching) {
      // Heading toward the trench: ramp in over a speed-dependent approach distance.
      double reach = kBaseApproachDistance + kApproachLookahead * speed;
      double fullStrength = kFullStrengthLeadTime * speed;
      axialWeight = 1.0 - smoothstep((outside - fullStrength) / (reach - fullStrength));
    } else {
      // Just left the trench (or driving away from it): fade out quickly.
      axialWeight = 1.0 - smoothstep(outside / kExitFadeDistance);
    }
    return intent * axialWeight * lateralWeight;
  }

  /** Smooth 0 to 1 ramp over [0, 1], clamped. */
  private static double smoothstep(double t) {
    t = MathUtil.clamp(t, 0.0, 1.0);
    return t * t * (3.0 - 2.0 * t);
  }

  /** Lane centerlines, for tests and visualization. */
  public static double[] laneCenterYs() {
    return new double[] {kRightLaneY, kLeftLaneY};
  }

  /** Trench center x positions, for tests and visualization. */
  public static double[] trenchCenterXs() {
    return new double[] {
      FieldConstants.LinesVertical.hubCenter, FieldConstants.LinesVertical.oppHubCenter
    };
  }
}
