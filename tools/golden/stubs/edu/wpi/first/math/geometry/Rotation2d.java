package edu.wpi.first.math.geometry;

public class Rotation2d {
  public static final Rotation2d kZero = new Rotation2d();
  public static final Rotation2d kPi = new Rotation2d(Math.PI);
  public static final Rotation2d k180deg = kPi;

  private final double m_value;
  private final double m_cos;
  private final double m_sin;

  public Rotation2d() {
    m_value = 0.0;
    m_cos = 1.0;
    m_sin = 0.0;
  }

  public Rotation2d(double value) {
    m_value = value;
    m_cos = Math.cos(value);
    m_sin = Math.sin(value);
  }

  public Rotation2d(double x, double y) {
    double magnitude = Math.hypot(x, y);
    if (magnitude > 1e-6) {
      m_cos = x / magnitude;
      m_sin = y / magnitude;
    } else {
      m_cos = 1.0;
      m_sin = 0.0;
    }
    m_value = Math.atan2(m_sin, m_cos);
  }

  public static Rotation2d fromDegrees(double degrees) {
    return new Rotation2d(Math.toRadians(degrees));
  }

  public static Rotation2d fromRadians(double radians) {
    return new Rotation2d(radians);
  }

  public Rotation2d plus(Rotation2d other) {
    return rotateBy(other);
  }

  public Rotation2d minus(Rotation2d other) {
    return rotateBy(other.unaryMinus());
  }

  public Rotation2d unaryMinus() {
    return new Rotation2d(-m_value);
  }

  public Rotation2d rotateBy(Rotation2d other) {
    return new Rotation2d(
        m_cos * other.m_cos - m_sin * other.m_sin, m_cos * other.m_sin + m_sin * other.m_cos);
  }

  public double getRadians() {
    return m_value;
  }

  public double getDegrees() {
    return Math.toDegrees(m_value);
  }

  public double getCos() {
    return m_cos;
  }

  public double getSin() {
    return m_sin;
  }
}
