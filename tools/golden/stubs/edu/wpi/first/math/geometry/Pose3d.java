package edu.wpi.first.math.geometry;

/** Minimal Pose3d: translation + yaw from a quaternion (only what FieldConstants needs). */
public class Pose3d {
  private final Translation3d m_translation;
  private final double m_yaw;

  public Pose3d(Translation3d translation, double qw, double qx, double qy, double qz) {
    m_translation = translation;
    m_yaw = Math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
  }

  public double getX() { return m_translation.getX(); }
  public double getY() { return m_translation.getY(); }
  public double getZ() { return m_translation.getZ(); }
  public Translation3d getTranslation() { return m_translation; }

  public Pose2d toPose2d() {
    return new Pose2d(m_translation.toTranslation2d(), new Rotation2d(m_yaw));
  }
}
