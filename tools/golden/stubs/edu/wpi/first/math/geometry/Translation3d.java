package edu.wpi.first.math.geometry;

public class Translation3d {
  private final double m_x, m_y, m_z;

  public Translation3d(double x, double y, double z) {
    m_x = x;
    m_y = y;
    m_z = z;
  }

  public double getX() { return m_x; }
  public double getY() { return m_y; }
  public double getZ() { return m_z; }

  public Translation2d toTranslation2d() {
    return new Translation2d(m_x, m_y);
  }
}
