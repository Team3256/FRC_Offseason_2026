// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

// Behavioural equivalence with the original Java implementation. The CSV files in
// src/test/resources/golden were produced by tools/golden/generate.sh, which runs the ORIGINAL
// Java code (the robot's Java TrenchAssist / ShotCalculator / FieldConstants). Each test feeds the
// C++ port the same inputs.

#include <cmath>
#include <string>
#include <vector>

#include <frc/geometry/Pose2d.h>
#include <gtest/gtest.h>

#include "FieldConstants.h"
#include "TestUtil.h"
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
