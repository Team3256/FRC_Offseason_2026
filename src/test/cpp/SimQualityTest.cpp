// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

// How the simulated robot drives: smooth on flat carpet, and over a bump it climbs, pitches and
// comes down. Set SIM_QUALITY_CSV=<file> to dump every loop (true 3-D pose, estimate, speeds).

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <numbers>
#include <vector>

#include <gtest/gtest.h>

#include "RobotHarness.h"
#include "sim/PhysicsWorld.h"

namespace {

struct Sample {
  double t, x, y, z, roll, pitch, yaw, estX, estY, estYaw, vx, vy, omega;
};

std::vector<Sample> Drive(RobotHarness& h, double stickY, double stickX, int loops, double t0) {
  std::vector<Sample> out;
  h.Driver().SetLeftY(stickY);
  h.Driver().SetLeftX(stickX);
  auto& world = sim::PhysicsWorld::Get();
  for (int i = 0; i < loops; i++) {
    h.Step();
    const auto p = world.TruePose3d();
    const auto est = h.C().GetDrivetrain().State().Pose;
    const auto v = world.Swerve().FieldVelocity();
    out.push_back({t0 + (i + 1) * 0.02, p.X().value(), p.Y().value(), p.Z().value(),
                   p.Rotation().X().value(), p.Rotation().Y().value(), p.Rotation().Z().value(),
                   est.X().value(), est.Y().value(), est.Rotation().Radians().value(), v.vx, v.vy,
                   v.omega});
  }
  return out;
}

void Dump(const std::vector<Sample>& s) {
  if (const char* path = std::getenv("SIM_QUALITY_CSV")) {
    FILE* f = std::fopen(path, "w");
    std::fprintf(f, "t,x,y,z,roll,pitch,yaw,estX,estY,estYaw,vx,vy,omega\n");
    for (const auto& a : s) {
      std::fprintf(f, "%.3f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.5f,%.4f,%.4f,%.4f\n", a.t, a.x,
                   a.y, a.z, a.roll, a.pitch, a.yaw, a.estX, a.estY, a.estYaw, a.vx, a.vy, a.omega);
    }
    std::fclose(f);
  }
}

}  // namespace

TEST(SimQualityTest, DrivesOverTheBump) {
  RobotHarness h{false, /*field=*/true};
  // Blue alliance zone, lined up with the right bump (x 4.06-5.18, y 1.58-3.44), facing red.
  h.C().GetDrivetrain().ResetPose(frc::Pose2d{2.3_m, 2.5_m, frc::Rotation2d{}});
  h.Teleop();
  h.Step(10);
  auto s = Drive(h, -0.6, 0, 150, 0);  // forward at ~60% stick for 3 s
  auto rest = Drive(h, 0, 0, 50, 3);
  s.insert(s.end(), rest.begin(), rest.end());
  Dump(s);

  double maxPitch = 0, maxZ = 0, flatPitchNoise = 0, yawDrift = 0;
  for (const auto& a : s) {
    maxPitch = std::max(maxPitch, std::abs(a.pitch));
    maxZ = std::max(maxZ, a.z);
    if (a.x < 3.3) {  // on flat carpet before the bump
      flatPitchNoise = std::max(flatPitchNoise, std::abs(a.pitch));
    }
    yawDrift = std::max(yawDrift, std::abs(a.yaw));
  }
  std::printf("max pitch %.1f deg, max z %.3f m, flat pitch %.2f deg, yaw drift %.2f deg, end x %.2f\n",
              maxPitch * 180 / std::numbers::pi, maxZ, flatPitchNoise * 180 / std::numbers::pi,
              yawDrift * 180 / std::numbers::pi, s.back().x);
  EXPECT_GT(s.back().x, 5.4) << "made it over the bump";
  EXPECT_GT(maxPitch * 180 / std::numbers::pi, 8) << "pitches on the 15 deg ramps";
  EXPECT_GT(maxZ, 0.08) << "the chassis rises on the bump";
  EXPECT_LT(flatPitchNoise * 180 / std::numbers::pi, 1.5) << "flat carpet is flat";
}
