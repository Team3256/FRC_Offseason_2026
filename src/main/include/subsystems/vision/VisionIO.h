// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <vector>

#include <frc/geometry/Pose3d.h>

enum class PoseObservationType { MEGATAG_1, MEGATAG_2, PHOTONVISION };

/** Represents a robot pose sample used for pose estimation. */
struct PoseObservation {
  double timestamp = 0;
  frc::Pose3d pose;
  double ambiguity = 0;
  int tagCount = 0;
  double averageTagDistance = 0;
  PoseObservationType type = PoseObservationType::PHOTONVISION;
  std::vector<short> tagIds;
};

struct VisionIOInputs {
  bool connected = false;
  std::vector<PoseObservation> poseObservations;
  std::vector<int> tagIds;
};

class VisionIO {
 public:
  virtual ~VisionIO() = default;
  /** Fills {@code inputs} (vectors are cleared and refilled, so their capacity is reused). */
  virtual void UpdateInputs(VisionIOInputs& inputs) {}
};
