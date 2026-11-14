// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <source_location>
#include <string>
#include <string_view>

#include "logging/Logger.h"

/**
 * Base of every mechanism. There is no scheduler: the superstructure decides each loop what every
 * mechanism should do and calls one of its request methods (SetVelocity, Off, ...). A mechanism
 * switched off at construction ignores every request.
 *
 * The request driving a mechanism is named after the method that made it (like the Java version's
 * StackWalker lookup, but resolved at compile time through std::source_location) and logged as
 * "<Name>/Goal" whenever it changes.
 */
class DisableSubsystem {
 public:
  DisableSubsystem(bool enabled, std::string_view name);
  virtual ~DisableSubsystem() = default;
  DisableSubsystem(const DisableSubsystem&) = delete;
  DisableSubsystem& operator=(const DisableSubsystem&) = delete;

  const bool disabled;

  /** Reads the mechanism's inputs and logs them. Call once per loop, before any request. */
  virtual void Periodic() = 0;

  const std::string& GetName() const { return m_name; }
  /** Name of the request driving the mechanism, e.g. "setVelocity"; "none" before the first. */
  const std::string& GoalName() const { return m_goal; }

  /** "void Shooter::SetVelocity(double)" -> "setVelocity" (the Java method name). */
  static std::string CallerName(const std::source_location& loc);

 protected:
  /**
   * Call at the top of every request method: records the calling method as the goal and returns
   * false (do nothing) if the mechanism is disabled.
   */
  bool Request(std::source_location loc = std::source_location::current());
  /**
   * True if the last Request() switched to a different request method. One-shot requests (Off)
   * only talk to the motors then: Phoenix keeps applying the last control request on its own.
   */
  bool GoalChanged() const { return m_goalChanged; }

 private:
  std::string m_name;
  std::string m_goal{"none"};
  // function_name() points at a string literal, so a pointer compare finds goal changes without
  // building a string every loop.
  const char* m_lastFunction = nullptr;
  bool m_goalChanged = false;
  logging::Output<std::string> m_goalLog;
};
