// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <string_view>

#include <frc/geometry/Transform3d.h>
#include <photon/PhotonCamera.h>

#include "subsystems/vision/VisionIO.h"

/** IO implementation for real PhotonVision hardware. */
class VisionIOPhotonVision : public VisionIO {
 public:
  /**
   * @param name The configured name of the camera.
   * @param robotToCamera The 3D position of the camera relative to the robot.
   */
  VisionIOPhotonVision(std::string_view name, frc::Transform3d robotToCamera);
  void UpdateInputs(VisionIOInputs& inputs) override;

 protected:
  photon::PhotonCamera m_camera;
  frc::Transform3d m_robotToCamera;
};
