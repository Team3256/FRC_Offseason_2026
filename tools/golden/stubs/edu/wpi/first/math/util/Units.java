package edu.wpi.first.math.util;

public final class Units {
  private Units() {}

  public static double inchesToMeters(double inches) {
    return inches * 0.0254;
  }

  public static double degreesToRadians(double degrees) {
    return Math.toRadians(degrees);
  }
}
