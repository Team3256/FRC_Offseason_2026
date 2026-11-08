// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <functional>
#include <source_location>
#include <string>
#include <string_view>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/SubsystemBase.h>
#include <wpi/FunctionExtras.h>

#include "logging/Logger.h"

/**
 * A subsystem that can be switched off at construction: every command factory then returns a
 * no-op command. Commands are named after the factory method that built them (like the Java
 * version's StackWalker lookup, but resolved at compile time through std::source_location), and
 * the running command's name is logged every loop as "<Name>/CurrentCommand".
 */
class DisableSubsystem : public frc2::SubsystemBase {
 public:
  DisableSubsystem(bool enabled, std::string_view name);

  const bool disabled;

  [[nodiscard]] frc2::CommandPtr Run(std::function<void()> action,
                                     std::source_location loc = std::source_location::current());
  [[nodiscard]] frc2::CommandPtr RunOnce(
      std::function<void()> action, std::source_location loc = std::source_location::current());
  [[nodiscard]] frc2::CommandPtr StartEnd(
      std::function<void()> start, std::function<void()> end,
      std::source_location loc = std::source_location::current());
  [[nodiscard]] frc2::CommandPtr RunEnd(std::function<void()> run, std::function<void()> end,
                                        std::source_location loc = std::source_location::current());
  [[nodiscard]] frc2::CommandPtr Defer(wpi::unique_function<frc2::CommandPtr()> supplier,
                                       std::source_location loc = std::source_location::current());

  void Periodic() override;

  /** "frc2::CommandPtr Shooter::SetVelocity(...)" -> "setVelocity" (the Java method name). */
  static std::string CallerName(const std::source_location& loc);

 private:
  logging::Output<std::string> m_currentCommandLog;
  const frc2::Command* m_lastCommand = reinterpret_cast<const frc2::Command*>(1);
};
