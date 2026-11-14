// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "utils/DisableSubsystem.h"

#include <cctype>

DisableSubsystem::DisableSubsystem(bool enabled, std::string_view name)
    : disabled{!enabled}, m_name{name}, m_goalLog{std::string{name} + "/Goal"} {
  m_goalLog.Set(m_goal);
}

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

bool DisableSubsystem::Request(std::source_location loc) {
  if (disabled) {
    return false;
  }
  m_goalChanged = loc.function_name() != m_lastFunction;
  if (m_goalChanged) {
    m_lastFunction = loc.function_name();
    m_goal = CallerName(loc);
    m_goalLog.Set(m_goal);
  }
  return true;
}
