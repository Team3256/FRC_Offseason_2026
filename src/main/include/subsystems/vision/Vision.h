// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <array>
#include <functional>
#include <memory>
#include <span>
#include <vector>

#include <frc/Alert.h>
#include <frc/geometry/Pose2d.h>
#include <frc2/command/SubsystemBase.h>
#include <units/time.h>

#include "logging/Logger.h"
#include "subsystems/vision/VisionIO.h"

/** Filters camera pose observations and feeds the accepted ones to the pose estimator. */
class Vision : public frc2::SubsystemBase {
 public:
  using VisionConsumer =
      std::function<void(const frc::Pose2d& visionRobotPose, units::second_t timestamp,
                         const std::array<double, 3>& stdDevs)>;

  Vision(VisionConsumer consumer, std::vector<std::unique_ptr<VisionIO>> io);

  void Periodic() override;

  static bool HasElement(std::span<const short> list1, std::span<const short> list2);

  int AcceptedCount() const { return m_acceptedTotal; }
  int RejectedCount() const { return m_rejectedTotal; }
  const VisionIOInputs& Inputs(size_t camera) const { return m_cameras[camera]->inputs; }

 private:
  struct Camera {
    explicit Camera(size_t index, std::unique_ptr<VisionIO> io);
    std::unique_ptr<VisionIO> io;
    VisionIOInputs inputs;
    frc::Alert disconnectedAlert;
    logging::LogValue<bool> connectedLog;
    logging::LogValue<std::vector<int64_t>> tagIdsLog;
    logging::LogValue<std::vector<frc::Pose3d>> posesLog;
    logging::LogValue<std::vector<double>> timestampsLog;
    logging::LogValue<std::vector<double>> ambiguityLog;
    logging::Output<std::vector<frc::Pose3d>> acceptedLog;
    logging::Output<std::vector<frc::Pose3d>> rejectedLog;
    std::vector<int64_t> tagIdsScratch;
    std::vector<frc::Pose3d> posesScratch;
    std::vector<double> timestampsScratch;
    std::vector<double> ambiguityScratch;
    std::vector<frc::Pose3d> acceptedScratch;
    std::vector<frc::Pose3d> rejectedScratch;
  };

  VisionConsumer m_consumer;
  std::vector<std::unique_ptr<Camera>> m_cameras;
  int m_acceptedTotal = 0;
  int m_rejectedTotal = 0;
};
