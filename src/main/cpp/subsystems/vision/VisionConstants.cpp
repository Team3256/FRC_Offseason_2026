// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "subsystems/vision/VisionConstants.h"

#include <frc/apriltag/AprilTagFields.h>

namespace vision {
const frc::AprilTagFieldLayout& AprilTagLayout() {
  static const frc::AprilTagFieldLayout layout =
      frc::AprilTagFieldLayout::LoadField(frc::AprilTagField::kDefaultField);
  return layout;
}
}  // namespace vision
