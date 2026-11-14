package edu.wpi.first.math.kinematics;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class ChassisSpeeds {
  public double vxMetersPerSecond;
  public double vyMetersPerSecond;
  public double omegaRadiansPerSecond;

  public ChassisSpeeds() {}

  public ChassisSpeeds(double vx, double vy, double omega) {
    vxMetersPerSecond = vx;
    vyMetersPerSecond = vy;
    omegaRadiansPerSecond = omega;
  }

  public static ChassisSpeeds fromFieldRelativeSpeeds(ChassisSpeeds s, Rotation2d robotAngle) {
    var rotated =
        new Translation2d(s.vxMetersPerSecond, s.vyMetersPerSecond).rotateBy(robotAngle.unaryMinus());
    return new ChassisSpeeds(rotated.getX(), rotated.getY(), s.omegaRadiansPerSecond);
  }

  public static ChassisSpeeds fromRobotRelativeSpeeds(ChassisSpeeds s, Rotation2d robotAngle) {
    var rotated = new Translation2d(s.vxMetersPerSecond, s.vyMetersPerSecond).rotateBy(robotAngle);
    return new ChassisSpeeds(rotated.getX(), rotated.getY(), s.omegaRadiansPerSecond);
  }
}
