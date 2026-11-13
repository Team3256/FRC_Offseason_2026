// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

/**
 * Rising and falling edges of a condition sampled once per loop (button presses, mode changes).
 * The first sample is never an edge: take one at startup so a button already held then does not
 * count as a press.
 */
class EdgeDetector {
 public:
  /** Samples {@code value}; returns true on a rising edge. */
  bool Update(bool value) {
    m_rose = m_sampled && value && !m_value;
    m_fell = m_sampled && !value && m_value;
    m_value = value;
    m_sampled = true;
    return m_rose;
  }
  bool Get() const { return m_value; }
  bool Rose() const { return m_rose; }
  bool Fell() const { return m_fell; }

 private:
  bool m_value = false;
  bool m_sampled = false;
  bool m_rose = false;
  bool m_fell = false;
};
