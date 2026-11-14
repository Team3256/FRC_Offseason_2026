package edu.wpi.first.math.geometry;

public class Pose2d {
  private final Translation2d m_translation;
  private final Rotation2d m_rotation;

  public Pose2d() {
    this(new Translation2d(), new Rotation2d());
  }

  public Pose2d(Translation2d translation, Rotation2d rotation) {
    m_translation = translation;
    m_rotation = rotation;
  }

  public Pose2d(double x, double y, Rotation2d rotation) {
    this(new Translation2d(x, y), rotation);
  }

  public Pose2d plus(Transform2d other) {
    return transformBy(other);
  }

  public Translation2d getTranslation() {
    return m_translation;
  }

  public double getX() {
    return m_translation.getX();
  }

  public double getY() {
    return m_translation.getY();
  }

  public Rotation2d getRotation() {
    return m_rotation;
  }

  public Pose2d transformBy(Transform2d other) {
    return new Pose2d(
        m_translation.plus(other.getTranslation().rotateBy(m_rotation)),
        other.getRotation().plus(m_rotation));
  }

  public Pose2d exp(Twist2d twist) {
    double dx = twist.dx;
    double dy = twist.dy;
    double dtheta = twist.dtheta;
    double sinTheta = Math.sin(dtheta);
    double cosTheta = Math.cos(dtheta);
    double s;
    double c;
    if (Math.abs(dtheta) < 1E-9) {
      s = 1.0 - 1.0 / 6.0 * dtheta * dtheta;
      c = 0.5 * dtheta;
    } else {
      s = sinTheta / dtheta;
      c = (1 - cosTheta) / dtheta;
    }
    var transform =
        new Transform2d(
            new Translation2d(dx * s - dy * c, dx * c + dy * s),
            new Rotation2d(cosTheta, sinTheta));
    return this.plus(transform);
  }
}
