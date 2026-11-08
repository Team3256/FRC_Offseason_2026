// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "subsystems/vision/VisionIOSim.h"

#include <cmath>
#include <random>

#include <frc/Timer.h>

#include "subsystems/vision/VisionConstants.h"

VisionIOSim::VisionIOSim(frc::Transform3d robotToCamera, std::function<frc::Pose2d()> poseSupplier,
                         double linearNoiseStdDev)
    : m_robotToCamera{robotToCamera},
      m_poseSupplier{std::move(poseSupplier)},
      m_noise{linearNoiseStdDev} {
  for (const auto& tag : vision::AprilTagLayout().GetTags()) {
    m_tags.push_back({tag.ID, tag.pose});
  }
}

std::vector<int> VisionIOSim::VisibleTags(const frc::Pose2d& robotPose) const {
  using namespace vision::sim;
  m_visible.clear();
  const frc::Pose3d cameraPose = frc::Pose3d{robotPose}.TransformBy(m_robotToCamera);
  const double halfH = units::radian_t{kHorizontalFov}.value() / 2.0;
  const double halfV = units::radian_t{kVerticalFov}.value() / 2.0;
  const double maxView = std::cos(units::radian_t{kMaxViewAngle}.value());
  for (const auto& tag : m_tags) {
    // Tag position in the camera frame (x forward, y left, z up)
    const frc::Translation3d rel =
        (tag.pose.Translation() - cameraPose.Translation()).RotateBy(-cameraPose.Rotation());
    const double x = rel.X().value();
    const double y = rel.Y().value();
    const double z = rel.Z().value();
    if (x <= 0.05) {
      continue;
    }
    const double dist = std::sqrt(x * x + y * y + z * z);
    if (dist > kMaxRange.value() || std::abs(std::atan2(y, x)) > halfH ||
        std::abs(std::atan2(z, x)) > halfV) {
      continue;
    }
    // The tag's face points along its +X axis; it must face the camera.
    const frc::Translation3d normal =
        frc::Translation3d{1_m, 0_m, 0_m}.RotateBy(tag.pose.Rotation());
    const frc::Translation3d toCamera = cameraPose.Translation() - tag.pose.Translation();
    const double cosView =
        (normal.X() * toCamera.X() + normal.Y() * toCamera.Y() + normal.Z() * toCamera.Z())
            .value() /
        toCamera.Norm().value();
    if (cosView < maxView) {
      continue;
    }
    m_visible.push_back(tag.id);
  }
  return m_visible;
}

void VisionIOSim::UpdateInputs(VisionIOInputs& inputs) {
  inputs.connected = true;
  inputs.poseObservations.clear();
  inputs.tagIds.clear();
  const frc::Pose2d robotPose = m_poseSupplier();
  VisibleTags(robotPose);
  if (m_visible.empty()) {
    return;
  }
  const frc::Pose3d cameraPose = frc::Pose3d{robotPose}.TransformBy(m_robotToCamera);
  double totalDistance = 0;
  for (int id : m_visible) {
    inputs.tagIds.push_back(id);
    totalDistance += vision::AprilTagLayout()
                         .GetTagPose(id)
                         ->Translation()
                         .Distance(cameraPose.Translation())
                         .value();
  }

  frc::Pose3d measured{robotPose};
  if (m_noise > 0) {
    static std::mt19937 rng{3256};
    std::normal_distribution<double> n{0.0, m_noise};
    measured =
        frc::Pose3d{measured.X() + units::meter_t{n(rng)}, measured.Y() + units::meter_t{n(rng)},
                    measured.Z(), measured.Rotation()};
  }

  PoseObservation obs;
  obs.timestamp = frc::Timer::GetFPGATimestamp().value();
  obs.pose = measured;
  obs.ambiguity = m_visible.size() >= 2 ? 0.0 : vision::sim::kSingleTagAmbiguity;
  obs.tagCount = static_cast<int>(m_visible.size());
  obs.averageTagDistance = totalDistance / static_cast<double>(m_visible.size());
  obs.type = PoseObservationType::PHOTONVISION;
  obs.tagIds.assign(m_visible.begin(), m_visible.end());
  inputs.poseObservations.push_back(std::move(obs));
}
