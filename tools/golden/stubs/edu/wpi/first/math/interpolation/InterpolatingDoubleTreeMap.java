package edu.wpi.first.math.interpolation;

import edu.wpi.first.math.MathUtil;
import java.util.TreeMap;

/** Same lookup as WPILib's InterpolatingTreeMap<Double, Double>. */
public class InterpolatingDoubleTreeMap {
  private final TreeMap<Double, Double> m_map = new TreeMap<>();

  public void put(Double key, Double value) {
    m_map.put(key, value);
  }

  public Double get(Double key) {
    Double val = m_map.get(key);
    if (val == null) {
      Double ceilingKey = m_map.ceilingKey(key);
      Double floorKey = m_map.floorKey(key);
      if (ceilingKey == null && floorKey == null) {
        return null;
      }
      if (ceilingKey == null) {
        return m_map.get(floorKey);
      }
      if (floorKey == null) {
        return m_map.get(ceilingKey);
      }
      Double floor = m_map.get(floorKey);
      Double ceiling = m_map.get(ceilingKey);
      return MathUtil.interpolate(floor, ceiling, inverseInterpolate(floorKey, ceilingKey, key));
    } else {
      return val;
    }
  }

  private static double inverseInterpolate(double startValue, double endValue, double q) {
    double totalRange = endValue - startValue;
    if (totalRange <= 0) {
      return 0.0;
    }
    double queryToStart = q - startValue;
    if (queryToStart <= 0) {
      return 0.0;
    }
    return queryToStart / totalRange;
  }
}
