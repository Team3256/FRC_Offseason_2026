package edu.wpi.first.math;

/** Same algorithms as WPILib 2026's MathUtil (only what the reference code uses). */
public final class MathUtil {
  private MathUtil() {}

  public static double clamp(double value, double low, double high) {
    return Math.max(low, Math.min(value, high));
  }

  public static double inputModulus(double input, double minimumInput, double maximumInput) {
    double modulus = maximumInput - minimumInput;
    int numMax = (int) ((input - minimumInput) / modulus);
    input -= numMax * modulus;
    int numMin = (int) ((input - maximumInput) / modulus);
    input -= numMin * modulus;
    return input;
  }

  public static double angleModulus(double angleRadians) {
    return inputModulus(angleRadians, -Math.PI, Math.PI);
  }

  public static double interpolate(double startValue, double endValue, double t) {
    return startValue + (endValue - startValue) * MathUtil.clamp(t, 0, 1);
  }
}
