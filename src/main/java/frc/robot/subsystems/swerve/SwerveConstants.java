// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a 
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.robot.FieldConstants;
import frc.robot.subsystems.swerve.generated.TunerConstants;

public final class SwerveConstants {
  // LinearVelocity is a vector, so we need to get the magnitude
  public static final double deadbandMultiplier = 0.15;
  public static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.magnitude();
  public static final double MaxAngularRate = 1.5 * Math.PI;
  public static final double SlowMaxSpeed = MaxSpeed * 0.3;
  public static final double SlowMaxAngular = MaxAngularRate * 0.3;

  public static final double SuperSlowMaxSpeed = MaxSpeed * 0.15;

  // Physics constants
  public static final Mass robotMass = Pounds.of(120);
  public static final MomentOfInertia robotMOI = KilogramSquareMeters.of(36);

  // Module Locations
  public static final Translation2d frontLeft =
      new Translation2d(TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY);
  public static final Translation2d frontRight =
      new Translation2d(TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY);
  public static final Translation2d backLeft =
      new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY);
  public static final Translation2d backRight =
      new Translation2d(TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY);

  // TODO: adjust each for overshoot
  public static final class AzimuthTargets {

    // TODO: tune these jawns
    public static final double aziKP = 6.0;
    public static final double aziKi = 0.0;
    public static final double aziKD = 0.0;
    public static final double timeout = 0.3;
    public static final Rotation2d bump = new Rotation2d(Math.toRadians(45));
  }

  public static final class BumpTargets {

    public static Pose2d BOTTOM_BUMP =
        new Pose2d(
            3.3690896034240723, 2.46048903465271, Rotation2d.fromDegrees(45.744063302312063968));

    public static Pose2d TOP_BUMP =
        new Pose2d(
            3.4050545692443848, 5.535476207733154, Rotation2d.fromDegrees(45.744063302312063968));
  }

  /**
   * Trench assist: automatically centers the robot in a trench lane (and squares it up to the
   * walls) when the driver heads into one. Distances in meters, speeds in m/s.
   *
   * <p>It only runs in teleop. To turn it off, set {@link #kUseTrenchAssist} to false, or flip
   * "TrenchAssist/Enabled" on the dashboard (no redeploy needed).
   */
  public static final class TrenchAssistConstants {
    /** Default for the dashboard toggle; false disables trench assist entirely. */
    public static final boolean kUseTrenchAssist = true;

    /** Half the robot's length (bumpers included) along the trench axis. */
    public static final double kRobotHalfLength = Units.inchesToMeters(16.5);

    /** Lateral distance from the lane centerline within which the assist is at full strength. */
    public static final double kLaneHalfWidth = FieldConstants.LeftTrench.openingWidth / 2.0;

    /**
     * Extra lateral distance over which the assist fades out (so bump approaches are not hijacked).
     */
    public static final double kCaptureMargin = Units.inchesToMeters(8.0);

    /**
     * Angle off the trench axis the driver can approach from and still be captured: the lateral
     * capture band widens by tan(angle) per meter of distance from the trench.
     */
    public static final double kApproachAngleTan = Math.tan(Math.toRadians(30.0));

    /**
     * Driver speed along the trench axis below which the assist stays out of the way. Robots shoot
     * from under the trench, so a stopped or creeping driver must keep full control.
     */
    public static final double kMinAssistSpeed = 1.0;

    /** Speed range over which the assist fades in, above {@link #kMinAssistSpeed}. */
    public static final double kAssistSpeedBlend = 0.75;

    /** The assist starts this far before the trench entrance, plus the lookahead below. */
    public static final double kBaseApproachDistance = 1.0;

    /** Extra approach distance per m/s of driver speed, so fast drivers are captured earlier. */
    public static final double kApproachLookahead = 1.0;

    /** The assist is at full strength this many seconds (of travel) before the entrance. */
    public static final double kFullStrengthLeadTime = 0.6;

    /** Distance after leaving the trench over which the assist fades out. */
    public static final double kExitFadeDistance = 0.3;

    /** Centering controller: lateral velocity per meter of lateral error. */
    public static final double kCenteringKP = 6.0;

    public static final double kMaxCenteringSpeed = 3.0;

    /** Heading controller used to square the robot to the trench walls (0 or 180 degrees). */
    public static final double kHeadingKP = 6.0;

    /** Driver turn rate (rad/s) above which the driver keeps control of heading. */
    public static final double kDriverTurnOverrideRate = 0.2 * MaxAngularRate;
  }
}
