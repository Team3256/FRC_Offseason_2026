// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "utils/MappedXboxController.h"

#include <fstream>
#include <iostream>

#include <wpi/json.h>

double MappedXboxController::GetTriggerAxes() {
  double left = GetLeftTriggerAxis();
  double right = GetRightTriggerAxis();
  if (left != 0 && right == 0) {
    return -left;
  }
  if (right != 0 && left == 0) {
    return right;
  }
  return 0.0;
}

std::string MappedXboxController::GetControllerMap(const MappedXboxController& c1,
                                                   const MappedXboxController& c2) {
  wpi::json j = wpi::json::object();
  j[c1.name] = c1.m_buttonMap;
  j[c2.name] = c2.m_buttonMap;
  return j.dump();
}

void MappedXboxController::DumpControllerMap(const MappedXboxController& c1,
                                             const MappedXboxController& c2) {
  std::ofstream out{"./scripts/map.json"};
  if (!out) {
    std::cerr << "Could not write ./scripts/map.json" << std::endl;
    return;
  }
  out << GetControllerMap(c1, c2);
  std::cout << "File written successfully!" << std::endl;
}
