// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

// Behavioural equivalence with the original Java implementation. The CSV files in
// src/test/resources/golden were produced by tools/golden/generate.sh, which runs the ORIGINAL
// Java code (the MayhemLib jar from the old vendordep and the robot's Java TrenchAssist /
// ShotCalculator / FieldConstants). Each test feeds the C++ port the same inputs.

#include <cmath>
#include <string>
#include <vector>

#include <frc/geometry/Pose2d.h>
#include <gtest/gtest.h>

#include "FieldConstants.h"
#include "TestUtil.h"
#include "mayhem/Runner.h"
#include "mayhem/Trajectory.h"
#include "subsystems/sotm/ShotCalculator.h"
#include "subsystems/swerve/TrenchAssist.h"

using test_util::Csv;

namespace {
constexpr double kTol = 1e-9;
// The closed-loop runs integrate thousands of steps; allow for last-ulp libm differences.
constexpr double kLoopTol = 1e-6;

frc::Pose2d Pose(double x, double y, double h) {
  return frc::Pose2d{units::meter_t{x}, units::meter_t{y}, frc::Rotation2d{units::radian_t{h}}};
}
frc::ChassisSpeeds Speeds(double vx, double vy, double w) {
  return frc::ChassisSpeeds{units::meters_per_second_t{vx}, units::meters_per_second_t{vy},
                            units::radians_per_second_t{w}};
}
}  // namespace

TEST(GoldenTest, FieldConstantsMatchJava) {
  Csv csv{"src/test/resources/golden/field_constants.csv"};
  namespace fc = field_constants;
  const std::map<std::string, double> cpp = {
      {"fieldLength", fc::fieldLength},
      {"fieldWidth", fc::fieldWidth},
      {"aprilTagCount", fc::kAprilTagCount},
      {"hubCenter", fc::lines_vertical::hubCenter},
      {"oppHubCenter", fc::lines_vertical::oppHubCenter},
      {"oppAllianceZone", fc::lines_vertical::oppAllianceZone},
      {"starting", fc::lines_vertical::starting},
      {"rightTrenchOpenStart", fc::lines_horizontal::rightTrenchOpenStart},
      {"leftTrenchOpenEnd", fc::lines_horizontal::leftTrenchOpenEnd},
      {"rightBumpStart", fc::lines_horizontal::rightBumpStart},
      {"leftBumpStart", fc::lines_horizontal::leftBumpStart},
      {"hubTopX", fc::hub::topCenterPoint.X().value()},
      {"hubTopY", fc::hub::topCenterPoint.Y().value()},
      {"oppHubTopX", fc::hub::oppTopCenterPoint.X().value()},
      {"towerCenterY", fc::tower::centerPoint.Y().value()},
      {"oppTowerCenterX", fc::tower::oppCenterPoint.X().value()},
      {"oppTowerCenterY", fc::tower::oppCenterPoint.Y().value()},
      {"outpostCenterY", fc::outpost::centerPoint.Y().value()},
  };
  ASSERT_EQ(csv.Size(), cpp.size());
  for (size_t i = 0; i < csv.Size(); i++) {
    const auto& name = csv.Str(i, "name");
    ASSERT_TRUE(cpp.contains(name)) << name;
    EXPECT_NEAR(cpp.at(name), csv.D(i, "value"), 1e-12) << name;
  }
}

TEST(GoldenTest, TrenchAssistMatchesJava) {
  Csv csv{"src/test/resources/golden/trench_assist.csv"};
  ASSERT_GT(csv.Size(), 1000u);
  int engaged = 0;
  for (size_t i = 0; i < csv.Size(); i++) {
    auto r = trench_assist::Apply(Pose(csv.D(i, "px"), csv.D(i, "py"), csv.D(i, "ph")),
                                  Speeds(csv.D(i, "vx"), csv.D(i, "vy"), csv.D(i, "w")));
    ASSERT_NEAR(r.speeds.vx.value(), csv.D(i, "ovx"), kTol) << "row " << i;
    ASSERT_NEAR(r.speeds.vy.value(), csv.D(i, "ovy"), kTol) << "row " << i;
    ASSERT_NEAR(r.speeds.omega.value(), csv.D(i, "ow"), kTol) << "row " << i;
    ASSERT_NEAR(r.weight, csv.D(i, "weight"), kTol) << "row " << i;
    engaged += r.weight > 0;
  }
  // the inputs must actually exercise the assist
  EXPECT_GT(engaged, 200);
}

TEST(GoldenTest, ShotCalculatorMatchesJava) {
  Csv csv{"src/test/resources/golden/shot_calculator.csv"};
  ASSERT_GT(csv.Size(), 1000u);
  for (size_t i = 0; i < csv.Size(); i++) {
    const frc::Pose2d pose = Pose(csv.D(i, "px"), csv.D(i, "py"), csv.D(i, "ph"));
    const frc::ChassisSpeeds speeds = Speeds(csv.D(i, "vx"), csv.D(i, "vy"), csv.D(i, "w"));
    const frc::Transform2d robotToShooter =
        csv.I(i, "tf") == 0 ? frc::Transform2d{0_m, 0_m, frc::Rotation2d{}}
                            : frc::Transform2d{0.2_m, -0.1_m, frc::Rotation2d{0.3_rad}};
    ShotCalculator sc{[pose] { return pose; }, [speeds] { return speeds; }, robotToShooter};
    sc.SetTarget(
        frc::Translation2d{units::meter_t{csv.D(i, "tx")}, units::meter_t{csv.D(i, "ty")}});
    sc.Periodic();
    ASSERT_NEAR(sc.GetLookaheadPose().X().value(), csv.D(i, "lx"), kTol) << "row " << i;
    ASSERT_NEAR(sc.GetLookaheadPose().Y().value(), csv.D(i, "ly"), kTol) << "row " << i;
    ASSERT_NEAR(sc.GetLookaheadPose().Rotation().Radians().value(), csv.D(i, "lh"), kTol)
        << "row " << i;
    ASSERT_NEAR(sc.GetDistance(), csv.D(i, "distance"), kTol) << "row " << i;
    ASSERT_NEAR(sc.GetDriveAngle().Radians().value(), csv.D(i, "driveAngle"), kTol) << "row " << i;
  }
}

TEST(GoldenTest, MayhemSamplingAndFlippingMatchJava) {
  test_util::UseRepoDeployDir();
  Csv csv{"src/test/resources/golden/mayhem_samples.csv"};
  ASSERT_GT(csv.Size(), 100u);
  for (size_t i = 0; i < csv.Size(); i++) {
    auto blue = mayhem::trajectory_loader::Load(csv.Str(i, "traj"));
    auto traj = csv.I(i, "flipped") ? blue->Flipped() : blue;
    const auto s = traj->SampleAt(csv.D(i, "t"));
    for (auto [name, value] : std::vector<std::pair<const char*, double>>{{"x", s.x},
                                                                          {"y", s.y},
                                                                          {"heading", s.heading},
                                                                          {"vx", s.vx},
                                                                          {"vy", s.vy},
                                                                          {"omega", s.omega},
                                                                          {"ax", s.ax},
                                                                          {"ay", s.ay},
                                                                          {"alpha", s.alpha}}) {
      ASSERT_NEAR(value, csv.D(i, name), kTol) << csv.Str(i, "traj") << " row " << i << " " << name;
    }
    ASSERT_EQ(s.nForces, csv.I(i, "nf"));
    for (int k = 0; k < s.nForces; k++) {
      ASSERT_NEAR(s.fx[k], csv.D(i, "fx" + std::to_string(k)), kTol);
      ASSERT_NEAR(s.fy[k], csv.D(i, "fy" + std::to_string(k)), kTol);
    }
  }
}

TEST(GoldenTest, MayhemBridgePlannerMatchesJava) {
  test_util::UseRepoDeployDir();
  Csv csv{"src/test/resources/golden/mayhem_planner.csv"};
  mayhem::RecoveryConfig rc;
  rc.persistenceLoops = 10;
  mayhem::BridgePlanner planner{rc};
  for (size_t i = 0; i < csv.Size(); i++) {
    auto traj = mayhem::trajectory_loader::Load(csv.Str(i, "traj"));
    auto res =
        planner.Plan(*traj, csv.D(i, "tNow"), Pose(csv.D(i, "px"), csv.D(i, "py"), csv.D(i, "ph")),
                     Speeds(csv.D(i, "vx"), csv.D(i, "vy"), csv.D(i, "w")));
    ASSERT_EQ(res.bridge.has_value(), csv.I(i, "found") == 1) << "row " << i;
    if (!res.bridge) {
      continue;
    }
    ASSERT_NEAR(res.bridge->joinTime, csv.D(i, "joinTime"), kTol) << "row " << i;
    ASSERT_NEAR(res.bridge->Duration(), csv.D(i, "duration"), kTol) << "row " << i;
    const auto m = res.bridge->SampleAt(res.bridge->Duration() / 2);
    ASSERT_NEAR(m.x, csv.D(i, "midx"), kTol);
    ASSERT_NEAR(m.y, csv.D(i, "midy"), kTol);
    ASSERT_NEAR(m.heading, csv.D(i, "midh"), kTol);
    ASSERT_NEAR(m.vx, csv.D(i, "midvx"), kTol);
    ASSERT_NEAR(m.vy, csv.D(i, "midvy"), kTol);
    ASSERT_NEAR(m.omega, csv.D(i, "midw"), kTol);
  }
}

/**
 * Closed-loop runs of every trajectory (blue and red; nominal, shoved, and beached on fuel) with
 * the same simple plant as tools/golden/src/Golden.java. Every step's drive command, runner state,
 * clock, errors and fired events must match the Java MayhemLib.
 */
TEST(GoldenTest, MayhemRunnerMatchesJava) {
  test_util::UseRepoDeployDir();
  Csv csv{"src/test/resources/golden/mayhem_runner.csv"};
  ASSERT_GT(csv.Size(), 5000u);

  size_t row = 0;
  int scenarios = 0;
  while (row < csv.Size()) {
    const std::string name = csv.Str(row, "traj");
    const bool flip = csv.I(row, "flipped") == 1;
    const std::string scenario = csv.Str(row, "scenario");
    SCOPED_TRACE(name + (flip ? " red " : " blue ") + scenario);
    auto blue = mayhem::trajectory_loader::Load(name);
    auto traj = flip ? blue->Flipped() : blue;

    std::vector<std::string> fired;
    mayhem::TrajectoryRunner::EventListener listener;
    listener.onEvent = [&fired](const mayhem::TrajectoryEvent& e) { fired.push_back(e.name); };
    listener.onZoneEnd = [&fired](const mayhem::TrajectoryEvent& e) {
      fired.push_back("end:" + e.name);
    };
    mayhem::RecoveryConfig rc;
    rc.persistenceLoops = 10;
    mayhem::TrajectoryRunner r{traj, mayhem::FollowerConfig{}, rc, listener};
    if (scenario == "unbeach") {
      r.WithUnbeach(mayhem::UnbeachConfig{});
    }
    constexpr double dt = 0.02;
    constexpr double gain = 0.92;
    const frc::Pose2d start = traj->InitialPose();
    double x = start.X().value() + 0.05;
    double y = start.Y().value() - 0.03;
    double h = start.Rotation().Radians().value() + 0.02;
    frc::ChassisSpeeds measured{};
    r.Start(10.0);
    int k = 0;
    for (; k < 1500 && !r.IsFinished(); k++, row++) {
      ASSERT_LT(row, csv.Size());
      ASSERT_EQ(csv.Str(row, "traj"), name);
      ASSERT_EQ(csv.Str(row, "scenario"), scenario);
      ASSERT_EQ(csv.I(row, "k"), k);
      const double now = 10.0 + (k + 1) * dt;
      double accel = mayhem::kNaN;
      double pitch = mayhem::kNaN;
      double roll = mayhem::kNaN;
      if (scenario == "shove" && k == 60) {
        x += 0.55;
        y -= 0.25;
        accel = 2.5;
      }
      if (scenario == "unbeach") {
        pitch = (k >= 50 && k < 90) ? 11.0 : 0.5;
        roll = 0.0;
      }
      ASSERT_NEAR(x, csv.D(row, "px"), kLoopTol) << "k=" << k;
      ASSERT_NEAR(y, csv.D(row, "py"), kLoopTol) << "k=" << k;
      ASSERT_NEAR(h, csv.D(row, "ph"), kLoopTol) << "k=" << k;
      fired.clear();
      const auto dc = r.Update(now, Pose(x, y, h), measured, accel, pitch, roll);
      ASSERT_NEAR(dc.fieldSpeeds.vx.value(), csv.D(row, "cvx"), kLoopTol) << "k=" << k;
      ASSERT_NEAR(dc.fieldSpeeds.vy.value(), csv.D(row, "cvy"), kLoopTol) << "k=" << k;
      ASSERT_NEAR(dc.fieldSpeeds.omega.value(), csv.D(row, "cw"), kLoopTol) << "k=" << k;
      for (int m = 0; m < 4; m++) {
        ASSERT_NEAR(dc.wheelForceX[m], csv.D(row, "fx" + std::to_string(m)), kLoopTol) << "k=" << k;
        ASSERT_NEAR(dc.wheelForceY[m], csv.D(row, "fy" + std::to_string(m)), kLoopTol) << "k=" << k;
      }
      ASSERT_EQ(mayhem::TrajectoryRunner::StateName(r.GetState()), csv.Str(row, "state"))
          << "k=" << k;
      ASSERT_NEAR(r.TrajectoryTime(), csv.D(row, "trajTime"), kLoopTol) << "k=" << k;
      ASSERT_NEAR(r.ClockRate(), csv.D(row, "rate"), kLoopTol) << "k=" << k;
      ASSERT_NEAR(r.PositionError(), csv.D(row, "posErr"), kLoopTol) << "k=" << k;
      ASSERT_NEAR(r.HeadingError(), csv.D(row, "headErr"), kLoopTol) << "k=" << k;
      ASSERT_EQ(r.BridgesPlanned(), csv.I(row, "bridges")) << "k=" << k;
      std::string events;
      for (size_t e = 0; e < fired.size(); e++) {
        events += (e ? "|" : "") + fired[e];
      }
      ASSERT_EQ(events, csv.Str(row, "events")) << "k=" << k;

      x += dc.fieldSpeeds.vx.value() * gain * dt;
      y += dc.fieldSpeeds.vy.value() * gain * dt;
      h += dc.fieldSpeeds.omega.value() * gain * dt;
      measured = Speeds(dc.fieldSpeeds.vx.value() * gain, dc.fieldSpeeds.vy.value() * gain,
                        dc.fieldSpeeds.omega.value() * gain);
    }
    // the Java run must have ended at the same step
    if (row < csv.Size()) {
      ASSERT_FALSE(csv.Str(row, "traj") == name && csv.Str(row, "scenario") == scenario &&
                   (csv.I(row, "flipped") == 1) == flip)
          << "Java kept running after the C++ runner finished at k=" << k;
    }
    scenarios++;
  }
  EXPECT_EQ(scenarios, 10 * 2 * 2 + 10);
}
