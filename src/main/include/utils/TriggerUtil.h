// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <functional>
#include <utility>

#include <frc2/command/button/Trigger.h>

/**
 * Trigger composition helpers equivalent to Java's Trigger.and/or/negate. They only use
 * Trigger::Get(), so they behave the same on every WPILib version.
 */
namespace trigger_util {

inline frc2::Trigger And(frc2::Trigger a, frc2::Trigger b) {
  return frc2::Trigger([a, b] { return a.Get() && b.Get(); });
}
inline frc2::Trigger And(frc2::Trigger a, std::function<bool()> b) {
  return frc2::Trigger([a, b = std::move(b)] { return a.Get() && b(); });
}
inline frc2::Trigger Or(frc2::Trigger a, frc2::Trigger b) {
  return frc2::Trigger([a, b] { return a.Get() || b.Get(); });
}
inline frc2::Trigger Not(frc2::Trigger a) {
  return frc2::Trigger([a] { return !a.Get(); });
}

}  // namespace trigger_util
