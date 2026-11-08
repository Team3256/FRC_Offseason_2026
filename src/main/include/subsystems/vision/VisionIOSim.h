// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <functional>
#include <vector>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Transform3d.h>

#include "subsystems/vision/VisionIO.h"

/**
 * Simulated AprilTag camera. Each loop it finds the tags of the field layout that the camera would
 * see from the robot's pose (in front of the lens, inside the field of view and range, facing the
 * camera) and reports a pose observation the way the PhotonVision IO does: a multi-tag estimate
 * when two or more are visible, a single-tag estimate otherwise.
 *
 * It replaces PhotonLib's VisionSystemSim, which is much heavier (and whose desktop libraries are
 * not available to the desktop build). Optional Gaussian noise; zero by default so simulation runs
 * are deterministic.
 */
class VisionIOSim : public VisionIO {
 public:
  VisionIOSim(frc::Transform3d robotToCamera, std::function<frc::Pose2d()> poseSupplier,
              double linearNoiseStdDev = 0.0);
  void UpdateInputs(VisionIOInputs& inputs) override;

  /** Tags the camera would see from {@code robotPose} (for tests). */
  std::vector<int> VisibleTags(const frc::Pose2d& robotPose) const;

 private:
  struct Tag {
    int id;
    frc::Pose3d pose;
  };
  frc::Transform3d m_robotToCamera;
  std::function<frc::Pose2d()> m_poseSupplier;
  double m_noise;
  std::vector<Tag> m_tags;
  mutable std::vector<int> m_visible;
};
