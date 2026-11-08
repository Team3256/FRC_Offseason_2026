// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <string>
#include <string_view>

#include <hal/DriverStation.h>

/** Equivalent of Java's DriverStation.reportWarning/reportError(msg, false). */
namespace ds_report {
inline void Warning(std::string_view message) {
  std::string m{message};
  HAL_SendError(0, 1, 0, m.c_str(), "", "", 1);
}
inline void Error(std::string_view message) {
  std::string m{message};
  HAL_SendError(1, 1, 0, m.c_str(), "", "", 1);
}
}  // namespace ds_report
