// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "mayhem/CtreSwerve.h"

#include <cmath>

#include <ctre/phoenix6/swerve/SwerveRequest.hpp>

#include "subsystems/swerve/CommandSwerveDrivetrain.h"

namespace mayhem {

std::unique_ptr<AutoFactory> CtreSwerveAutoFactory(
    subsystems::CommandSwerveDrivetrain& drivetrain) {
  using namespace ctre::phoenix6::swerve;
  auto request = std::make_shared<requests::ApplyFieldSpeeds>(
      requests::ApplyFieldSpeeds{}
          .WithDriveRequestType(DriveRequestType::Velocity)
          .WithForwardPerspective(requests::ForwardPerspectiveValue::BlueAlliance)
          .WithDesaturateWheelSpeeds(true));
  request->WheelForceFeedforwardsX.assign(kMaxModules, 0_N);
  request->WheelForceFeedforwardsY.assign(kMaxModules, 0_N);
  auto* dt = &drivetrain;

  auto factory = std::make_unique<AutoFactory>(
      [dt] { return dt->State().Pose; }, [dt](const frc::Pose2d& pose) { dt->ResetPose(pose); },
      [dt] {
        const auto& st = dt->State();
        return frc::ChassisSpeeds::FromRobotRelativeSpeeds(st.Speeds, st.Pose.Rotation());
      },
      [dt, request](const DriveCommand& dc) {
        // Write into the request's own vectors: no allocation per loop.
        request->Speeds = dc.fieldSpeeds;
        request->WheelForceFeedforwardsX.resize(dc.modules);
        request->WheelForceFeedforwardsY.resize(dc.modules);
        for (int i = 0; i < dc.modules && i < kMaxModules; i++) {
          request->WheelForceFeedforwardsX[i] = units::newton_t{dc.wheelForceX[i]};
          request->WheelForceFeedforwardsY[i] = units::newton_t{dc.wheelForceY[i]};
        }
        dt->SetControl(*request);
      },
      true, dt);
  auto& pigeon = drivetrain.GetPigeon2();
  factory
      ->WithAccelerometer([&pigeon] {
        return std::hypot(pigeon.GetAccelerationX().GetValueAsDouble(),
                          pigeon.GetAccelerationY().GetValueAsDouble());
      })
      .WithTiltSensor([&pigeon] { return pigeon.GetPitch().GetValue().value(); },
                      [&pigeon] { return pigeon.GetRoll().GetValue().value(); });
  return factory;
}

}  // namespace mayhem
