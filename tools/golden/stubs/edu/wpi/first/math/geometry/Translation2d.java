package edu.wpi.first.math.geometry;

public class Translation2d {
  public static final Translation2d kZero = new Translation2d();
  private final double m_x;
  private final double m_y;

  public Translation2d() {
    this(0.0, 0.0);
  }

  public Translation2d(double x, double y) {
    m_x = x;
    m_y = y;
  }

  public double getDistance(Translation2d other) {
    return Math.hypot(other.m_x - m_x, other.m_y - m_y);
  }

  public double getX() {
    return m_x;
  }

  public double getY() {
    return m_y;
  }

  public double getNorm() {
    return Math.hypot(m_x, m_y);
  }

  public Rotation2d getAngle() {
    return new Rotation2d(m_x, m_y);
  }

  public Translation2d rotateBy(Rotation2d other) {
    return new Translation2d(
        m_x * other.getCos() - m_y * other.getSin(), m_x * other.getSin() + m_y * other.getCos());
  }

  public Translation2d plus(Translation2d other) {
    return new Translation2d(m_x + other.m_x, m_y + other.m_y);
  }

  public Translation2d minus(Translation2d other) {
    return new Translation2d(m_x - other.m_x, m_y - other.m_y);
  }

  public Translation2d unaryMinus() {
    return new Translation2d(-m_x, -m_y);
  }

  public Translation2d times(double scalar) {
    return new Translation2d(m_x * scalar, m_y * scalar);
  }

  public Translation2d div(double scalar) {
    return new Translation2d(m_x / scalar, m_y / scalar);
  }

  @Override
  public boolean equals(Object obj) {
    return obj instanceof Translation2d other
        && Math.abs(other.m_x - m_x) < 1E-9
        && Math.abs(other.m_y - m_y) < 1E-9;
  }

  @Override
  public int hashCode() {
    return 0;
  }
}
