// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "subsystems/vision/VisionIOPhotonVision.h"

#include <algorithm>

#include "subsystems/vision/VisionConstants.h"

VisionIOPhotonVision::VisionIOPhotonVision(std::string_view name, frc::Transform3d robotToCamera)
    : m_camera{name}, m_robotToCamera{robotToCamera} {}

void VisionIOPhotonVision::UpdateInputs(VisionIOInputs& inputs) {
  inputs.connected = m_camera.IsConnected();
  inputs.poseObservations.clear();
  inputs.tagIds.clear();
  const frc::Transform3d cameraToRobot = m_robotToCamera.Inverse();

  // Read new camera observations
  for (const auto& result : m_camera.GetAllUnreadResults()) {
    const auto targets = result.GetTargets();
    if (result.MultiTagResult().has_value()) {  // Multitag result
      const auto& multitagResult = *result.MultiTagResult();

      // Calculate robot pose
      const frc::Transform3d fieldToCamera = multitagResult.estimatedPose.best;
      const frc::Transform3d fieldToRobot = fieldToCamera + cameraToRobot;
      const frc::Pose3d robotPose{fieldToRobot.Translation(), fieldToRobot.Rotation()};

      // Calculate average tag distance
      double totalTagDistance = 0.0;
      for (const auto& target : targets) {
        totalTagDistance += target.GetBestCameraToTarget().Translation().Norm().value();
      }

      // Add tag IDs
      for (short id : multitagResult.fiducialIDsUsed) {
        if (std::find(inputs.tagIds.begin(), inputs.tagIds.end(), id) == inputs.tagIds.end()) {
          inputs.tagIds.push_back(id);
        }
      }

      PoseObservation obs;
      obs.timestamp = result.GetTimestamp().value();
      obs.pose = robotPose;
      obs.ambiguity = multitagResult.estimatedPose.ambiguity;
      obs.tagCount = static_cast<int>(multitagResult.fiducialIDsUsed.size());
      obs.averageTagDistance = totalTagDistance / static_cast<double>(targets.size());
      obs.type = PoseObservationType::PHOTONVISION;
      obs.tagIds.assign(multitagResult.fiducialIDsUsed.begin(),
                        multitagResult.fiducialIDsUsed.end());
      inputs.poseObservations.push_back(std::move(obs));
    } else if (!targets.empty()) {  // Single tag result
      const auto& target = targets[0];
      auto tagPose = vision::AprilTagLayout().GetTagPose(target.GetFiducialId());
      if (tagPose.has_value()) {
        const frc::Transform3d fieldToTarget{tagPose->Translation(), tagPose->Rotation()};
        const frc::Transform3d cameraToTarget = target.GetBestCameraToTarget();
        const frc::Transform3d fieldToCamera = fieldToTarget + cameraToTarget.Inverse();
        const frc::Transform3d fieldToRobot = fieldToCamera + cameraToRobot;
        const frc::Pose3d robotPose{fieldToRobot.Translation(), fieldToRobot.Rotation()};

        const short id = static_cast<short>(target.GetFiducialId());
        if (std::find(inputs.tagIds.begin(), inputs.tagIds.end(), id) == inputs.tagIds.end()) {
          inputs.tagIds.push_back(id);
        }

        PoseObservation obs;
        obs.timestamp = result.GetTimestamp().value();
        obs.pose = robotPose;
        obs.ambiguity = target.GetPoseAmbiguity();
        obs.tagCount = 1;
        obs.averageTagDistance = cameraToTarget.Translation().Norm().value();
        obs.type = PoseObservationType::PHOTONVISION;
        obs.tagIds = {id};
        inputs.poseObservations.push_back(std::move(obs));
      }
    }
  }
}
