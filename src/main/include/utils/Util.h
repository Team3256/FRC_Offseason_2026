// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <algorithm>
#include <cmath>
#include <limits>
#include <span>
#include <string>
#include <string_view>

namespace util {

inline constexpr double kEpsilon = 1e-12;

constexpr int Gcd(int a, int b) {
  while (b != 0) {
    int temp = b;
    b = a % b;
    a = temp;
  }
  return a < 0 ? -a : a;
}

constexpr int Lcm(int a, int b) {
  int p = a * b;
  return (p < 0 ? -p : p) / Gcd(a, b);
}

constexpr double Limit(double v, double min, double max) { return std::min(max, std::max(min, v)); }
constexpr double Limit(double v, double maxMagnitude) {
  return Limit(v, -maxMagnitude, maxMagnitude);
}
constexpr int Limit(int v, int min, int max) { return std::min(max, std::max(min, v)); }

constexpr bool InRange(double v, double min, double max) { return v > min && v < max; }
constexpr bool InRange(double v, double maxMagnitude) {
  return InRange(v, -maxMagnitude, maxMagnitude);
}

constexpr double Interpolate(double a, double b, double x) {
  x = Limit(x, 0.0, 1.0);
  return a + (b - a) * x;
}

constexpr bool EpsilonEquals(double a, double b, double epsilon) {
  return (a - epsilon <= b) && (a + epsilon >= b);
}
constexpr bool EpsilonEquals(double a, double b) { return EpsilonEquals(a, b, kEpsilon); }
constexpr bool EpsilonEquals(int a, int b, int epsilon) {
  return (a - epsilon <= b) && (a + epsilon >= b);
}

inline bool AllCloseTo(std::span<const double> list, double value, double epsilon) {
  bool result = true;
  for (double v : list) {
    result &= EpsilonEquals(v, value, epsilon);
  }
  return result;
}

inline double SnapToClosest(std::span<const double> list, double value) {
  double closest = std::numeric_limits<double>::max();
  double minDiff = std::numeric_limits<double>::max();
  for (double v : list) {
    double diff = std::abs(v - value);
    if (diff < minDiff) {
      minDiff = diff;
      closest = v;
    }
  }
  return closest;
}

inline double SnapToZone(std::span<const double> list, double value, double epsilon) {
  double closest = std::numeric_limits<double>::max();
  double minDiff = std::numeric_limits<double>::max();
  for (double v : list) {
    double diff = std::abs(v - value);
    if (diff < minDiff) {
      minDiff = diff;
      closest = v;
    }
  }
  return minDiff < epsilon ? closest : value;
}

constexpr double HandleDeadband(double value, double deadband) {
  deadband = deadband < 0 ? -deadband : deadband;
  if (deadband == 1) {
    return 0;
  }
  double scaledValue = (value + (value < 0 ? deadband : -deadband)) / (1 - deadband);
  return ((value < 0 ? -value : value) > deadband) ? scaledValue : 0;
}

/** Java's Math.signum. */
constexpr double Signum(double v) { return v > 0 ? 1.0 : (v < 0 ? -1.0 : v); }

}  // namespace util
