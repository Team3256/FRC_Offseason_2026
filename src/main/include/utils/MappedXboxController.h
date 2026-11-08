// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <map>
#include <string>
#include <string_view>

#include <frc2/command/button/CommandXboxController.h>

/** CommandXboxController with a name and optional per-button labels (for a controller map). */
class MappedXboxController : public frc2::CommandXboxController {
 public:
  MappedXboxController(int port, std::string_view name)
      : frc2::CommandXboxController(port), name{name} {}

  const std::string name;

  void LabelButton(std::string_view button, std::string_view label) {
    m_buttonMap[std::string{button}] = std::string{label};
  }

  /** Left trigger as negative, right trigger as positive, 0 if both or neither are pressed. */
  double GetTriggerAxes();

  /** JSON {"<controller>": {"<button>": "<label>"}} for both controllers. */
  static std::string GetControllerMap(const MappedXboxController& c1,
                                      const MappedXboxController& c2);
  static void DumpControllerMap(const MappedXboxController& c1, const MappedXboxController& c2);

 private:
  std::map<std::string, std::string> m_buttonMap;
};
