package edu.wpi.first.math.geometry;

public class Transform2d {
  private final Translation2d m_translation;
  private final Rotation2d m_rotation;

  public Transform2d(Translation2d translation, Rotation2d rotation) {
    m_translation = translation;
    m_rotation = rotation;
  }

  public Transform2d(double x, double y, Rotation2d rotation) {
    this(new Translation2d(x, y), rotation);
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
}
