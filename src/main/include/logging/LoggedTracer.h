// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

namespace logging {

/**
 * Logs how long each part of the robot loop takes ("RealOutputs/LoggedTracer/<name>MS"). Reset()
 * at the start of the loop, then Record("<name>") after each part.
 *
 * Pass string literals: entries are cached by the pointer, so a call costs one small lookup and a
 * clock read.
 */
class LoggedTracer {
 public:
  LoggedTracer() = delete;

  /** Resets the clock. */
  static void Reset();

  /** Logs the time since the last Reset() or Record() under {@code epochName}. */
  static void Record(const char* epochName);
};

}  // namespace logging
