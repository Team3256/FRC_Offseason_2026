// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <algorithm>
#include <array>
#include <cstddef>
#include <utility>

namespace util {

/**
 * Fixed-size, constexpr version of WPILib's InterpolatingDoubleTreeMap: linear interpolation
 * between points, clamped to the first/last value outside the range. Lookups are a short linear
 * scan over a sorted array (faster than a tree for the handful of points a lookup table has).
 */
template <size_t N>
class InterpolatingMap {
 public:
  constexpr explicit InterpolatingMap(std::array<std::pair<double, double>, N> points)
      : m_points{points} {
    std::sort(m_points.begin(), m_points.end());
  }

  constexpr double Get(double key) const {
    if (key <= m_points.front().first) {
      return m_points.front().second;
    }
    if (key >= m_points.back().first) {
      return m_points.back().second;
    }
    for (size_t i = 1; i < N; ++i) {
      if (key < m_points[i].first) {
        const auto& [x0, y0] = m_points[i - 1];
        const auto& [x1, y1] = m_points[i];
        // same formula as InterpolatingTreeMap: inverse-interpolate then lerp
        double t = (key - x0) / (x1 - x0);
        return y0 + (y1 - y0) * t;
      }
    }
    // key equals the last key exactly (handled above); keep the compiler happy
    return m_points.back().second;
  }

  constexpr const std::array<std::pair<double, double>, N>& Points() const { return m_points; }

 private:
  std::array<std::pair<double, double>, N> m_points;
};

}  // namespace util
