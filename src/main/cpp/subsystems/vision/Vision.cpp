// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "subsystems/vision/Vision.h"

#include <algorithm>
#include <cmath>
#include <string>

#include "logging/LoggedTracer.h"
#include "subsystems/vision/VisionConstants.h"

Vision::Camera::Camera(size_t index, std::unique_ptr<VisionIO> io_)
    : io{std::move(io_)},
      disconnectedAlert{"Vision camera " + std::to_string(index) + " is disconnected.",
                        frc::Alert::AlertType::kWarning},
      connectedLog{"Vision/Camera" + std::to_string(index) + "/connected"},
      tagIdsLog{"Vision/Camera" + std::to_string(index) + "/tagIds"},
      posesLog{"Vision/Camera" + std::to_string(index) + "/poseObservations/pose"},
      timestampsLog{"Vision/Camera" + std::to_string(index) + "/poseObservations/timestamp"},
      ambiguityLog{"Vision/Camera" + std::to_string(index) + "/poseObservations/ambiguity"},
      acceptedLog{"Vision/Camera" + std::to_string(index) + "/RobotPosesAccepted"},
      rejectedLog{"Vision/Camera" + std::to_string(index) + "/RobotPosesRejected"} {}

Vision::Vision(VisionConsumer consumer, std::vector<std::unique_ptr<VisionIO>> io)
    : frc2::SubsystemBase("Vision"), m_consumer{std::move(consumer)} {
  m_cameras.reserve(io.size());
  for (size_t i = 0; i < io.size(); i++) {
    m_cameras.push_back(std::make_unique<Camera>(i, std::move(io[i])));
  }
}

bool Vision::HasElement(std::span<const short> list1, std::span<const short> list2) {
  for (short i : list1) {
    if (std::find(list2.begin(), list2.end(), i) != list2.end()) {
      return true;
    }
  }
  return false;
}

void Vision::Periodic() {
  using namespace vision;
  const auto& layout = AprilTagLayout();
  const double fieldLength = layout.GetFieldLength().value();
  const double fieldWidth = layout.GetFieldWidth().value();

  for (size_t cameraIndex = 0; cameraIndex < m_cameras.size(); cameraIndex++) {
    Camera& cam = *m_cameras[cameraIndex];
    cam.io->UpdateInputs(cam.inputs);

    // Log inputs
    cam.connectedLog.Set(cam.inputs.connected);
    cam.tagIdsScratch.assign(cam.inputs.tagIds.begin(), cam.inputs.tagIds.end());
    cam.tagIdsLog.Set(std::span<const int64_t>{cam.tagIdsScratch});
    cam.posesScratch.clear();
    cam.timestampsScratch.clear();
    cam.ambiguityScratch.clear();
    for (const auto& o : cam.inputs.poseObservations) {
      cam.posesScratch.push_back(o.pose);
      cam.timestampsScratch.push_back(o.timestamp);
      cam.ambiguityScratch.push_back(o.ambiguity);
    }
    cam.posesLog.Set(std::span<const frc::Pose3d>{cam.posesScratch});
    cam.timestampsLog.Set(std::span<const double>{cam.timestampsScratch});
    cam.ambiguityLog.Set(std::span<const double>{cam.ambiguityScratch});

    // Update disconnected alert
    cam.disconnectedAlert.Set(!cam.inputs.connected);

    cam.acceptedScratch.clear();
    cam.rejectedScratch.clear();
    // Loop over pose observations
    for (const auto& observation : cam.inputs.poseObservations) {
      // Check whether to reject pose
      const bool rejectPose =
          observation.tagCount == 0  // Must have at least one tag
          || (observation.tagCount == 1 &&
              observation.ambiguity > kMaxAmbiguity)  // Cannot be high ambiguity
          ||
          std::abs(observation.pose.Z().value()) > kMaxZError  // Must have realistic Z coordinate
          // Must be within the field boundaries
          || observation.pose.X().value() < 0.0 || observation.pose.X().value() > fieldLength ||
          observation.pose.Y().value() < 0.0 || observation.pose.Y().value() > fieldWidth;

      if (rejectPose) {
        cam.rejectedScratch.push_back(observation.pose);
        m_rejectedTotal++;
        continue;
      }
      cam.acceptedScratch.push_back(observation.pose);
      m_acceptedTotal++;

      // Calculate standard deviations
      double stdDevFactor =
          std::pow(observation.averageTagDistance, 2.0) / static_cast<double>(observation.tagCount);
      if (HasElement(kNonHub, observation.tagIds)) {
        stdDevFactor *= 1.3;
      }
      double linearStdDev = kLinearStdDevBaseline * stdDevFactor;
      double angularStdDev = kAngularStdDevBaseline * stdDevFactor;
      if (cameraIndex < kCameraStdDevFactors.size()) {
        linearStdDev *= kCameraStdDevFactors[cameraIndex];
        angularStdDev *= kCameraStdDevFactors[cameraIndex];
      }

      // Send vision observation
      m_consumer(observation.pose.ToPose2d(), units::second_t{observation.timestamp},
                 {linearStdDev, linearStdDev, angularStdDev});
    }
    cam.acceptedLog.Set(std::span<const frc::Pose3d>{cam.acceptedScratch});
    cam.rejectedLog.Set(std::span<const frc::Pose3d>{cam.rejectedScratch});
  }
  logging::LoggedTracer::Record("Vision");
}
