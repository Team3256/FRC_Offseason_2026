// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "utils/DisableSubsystem.h"

#include <cctype>

#include <frc2/command/Commands.h>

DisableSubsystem::DisableSubsystem(bool enabled, std::string_view name)
    : frc2::SubsystemBase(name),
      disabled{!enabled},
      m_currentCommandLog{std::string{name} + "/CurrentCommand"} {}

std::string DisableSubsystem::CallerName(const std::source_location& loc) {
  std::string_view fn = loc.function_name();
  auto paren = fn.find('(');
  // Clang spells unnamed namespaces "(anonymous namespace)" in function names.
  // Skip those parentheses before looking for the function's argument list.
  constexpr std::string_view anonymousNamespace = "(anonymous namespace)";
  while (paren != std::string_view::npos && fn.substr(paren).starts_with(anonymousNamespace)) {
    paren = fn.find('(', paren + anonymousNamespace.size());
  }
  if (paren != std::string_view::npos) {
    fn = fn.substr(0, paren);
  }
  auto sep = fn.rfind("::");
  if (sep != std::string_view::npos) {
    fn = fn.substr(sep + 2);
  }
  auto space = fn.rfind(' ');
  if (space != std::string_view::npos) {
    fn = fn.substr(space + 1);
  }
  std::string name{fn};
  if (!name.empty()) {
    name[0] = static_cast<char>(std::tolower(static_cast<unsigned char>(name[0])));
  }
  return name;
}

frc2::CommandPtr DisableSubsystem::Run(std::function<void()> action, std::source_location loc) {
  return disabled ? frc2::cmd::None()
                  : frc2::SubsystemBase::Run(std::move(action)).WithName(CallerName(loc));
}

frc2::CommandPtr DisableSubsystem::RunOnce(std::function<void()> action, std::source_location loc) {
  return disabled ? frc2::cmd::None()
                  : frc2::SubsystemBase::RunOnce(std::move(action))
                        .WithName(CallerName(loc) + ".runOnce");
}

frc2::CommandPtr DisableSubsystem::StartEnd(std::function<void()> start, std::function<void()> end,
                                            std::source_location loc) {
  return disabled ? frc2::cmd::None()
                  : frc2::SubsystemBase::StartEnd(std::move(start), std::move(end))
                        .WithName(CallerName(loc) + ".startEnd");
}

frc2::CommandPtr DisableSubsystem::RunEnd(std::function<void()> run, std::function<void()> end,
                                          std::source_location loc) {
  return disabled ? frc2::cmd::None()
                  : frc2::SubsystemBase::RunEnd(std::move(run), std::move(end))
                        .WithName(CallerName(loc) + ".runEnd");
}

frc2::CommandPtr DisableSubsystem::Defer(wpi::unique_function<frc2::CommandPtr()> supplier,
                                         std::source_location loc) {
  return disabled
             ? frc2::cmd::None()
             : frc2::SubsystemBase::Defer(std::move(supplier)).WithName(CallerName(loc) + ".defer");
}

void DisableSubsystem::Periodic() {
  // Only rebuild the name string when the running command changes.
  const frc2::Command* current = GetCurrentCommand();
  if (current != m_lastCommand) {
    m_lastCommand = current;
    m_currentCommandLog.Set(current != nullptr ? current->GetName() : std::string{"none"});
  }
}
