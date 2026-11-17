// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

// Shots are only taken once the robot is lined up with the hub, and the autos shoot soon after
// they arrive. Full field, physics simulation: every FUEL that leaves the shooter is checked
// against the robot's true heading at that moment.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <numbers>
#include <string>
#include <vector>

#include <frc/geometry/Pose2d.h>
#include <gtest/gtest.h>
#include <networktables/NetworkTableInstance.h>

#include "RobotHarness.h"
#include "commands/MayhemProject.h"
#include "sim/PhysicsWorld.h"
#include "subsystems/sotm/ShotCalculator.h"

namespace {

constexpr double kDeg = std::numbers::pi / 180.0;

/** |true heading - shot drive angle| in degrees. */
double AimErrorDeg(RobotHarness& h) {
  const auto truth = sim::PhysicsWorld::Get().TruePose().Rotation();
  const auto want = h.C().GetShotCalculator().GetDriveAngle();
  return std::abs((truth - want).Radians().value()) / kDeg;
}

struct ShotLog {
  std::vector<double> errorsDeg;  // aim error of each FUEL as it left the shooter
  int scored = 0;
  double firstShotAt = -1;  // s after A was pressed
  double Worst() const {
    return errorsDeg.empty() ? 0 : *std::max_element(errorsDeg.begin(), errorsDeg.end());
  }
};

/** Holds A for {@code seconds} from {@code start} (teleop, full field), logging every shot. */
ShotLog HoldA(const frc::Pose2d& start, double seconds) {
  RobotHarness h{false, /*field=*/true};
  h.C().GetDrivetrain().ResetPose(start);
  h.Step(5);
  h.Teleop();
  h.Step(100);  // in the alliance zone: the flywheel is already revved, as in a match
  auto& world = sim::PhysicsWorld::Get();
  const int scoredBefore = world.GetMetrics().scoredOwnHub;
  ShotLog log;
  h.Driver().SetAButton(true);
  int shots = world.GetMetrics().shotsFired;
  const int loops = static_cast<int>(seconds / 0.02);
  for (int i = 0; i < loops; i++) {
    h.Step();
    const int now = world.GetMetrics().shotsFired;
    for (; shots < now; shots++) {
      log.errorsDeg.push_back(AimErrorDeg(h));
      if (log.firstShotAt < 0) {
        log.firstShotAt = (i + 1) * 0.02;
      }
    }
  }
  h.Driver().SetAButton(false);
  h.Step(75);  // the last shots land
  log.scored = world.GetMetrics().scoredOwnHub - scoredBefore;
  return log;
}

class HoldAShotTest : public ::testing::TestWithParam<double> {};

// Facing well away from the hub when A is pressed: no FUEL leaves until the robot has turned to
// the shot.
TEST_P(HoldAShotTest, WaitsUntilLinedUpWithTheHub) {
  const double startDeg = GetParam();
  const auto log = HoldA(frc::Pose2d{2.5_m, 6.0_m, frc::Rotation2d{units::degree_t{startDeg}}}, 3);
  std::printf(
      "[shot-aim] start %.0f deg: %zu shots, first at %.2f s, worst aim %.1f deg, %d scored\n",
      startDeg, log.errorsDeg.size(), log.firstShotAt, log.Worst(), log.scored);
  ASSERT_FALSE(log.errorsDeg.empty()) << "never shot";
  EXPECT_LE(log.Worst(), 3.5) << "shot before lining up";
  EXPECT_GT(log.scored, 0);
  EXPECT_LT(log.firstShotAt, 1.2) << "too slow to start shooting";
}

INSTANTIATE_TEST_SUITE_P(StartHeadings, HoldAShotTest, ::testing::Values(0.0, 90.0, -90.0, 180.0),
                         [](const auto& info) {
                           const int d = static_cast<int>(info.param);
                           return std::string{d < 0 ? "Minus" : ""} + std::to_string(std::abs(d));
                         });

struct AutoShots {
  // s from the routine arriving at its shooting spot (aim-and-shoot step) to the first FUEL out
  std::vector<double> arriveToFirstShot;
  std::vector<double> errorsDeg;
};

AutoShots RunAutoShots(const std::string& name, bool red) {
  RobotHarness h{red, /*field=*/true};
  h.Step(5);
  h.C().GetAutoChooser().Select(name);
  h.Step(3);
  h.Autonomous();
  auto graph = nt::NetworkTableInstance::GetDefault().GetTable("Mayhem/States/Autonomous");
  auto& world = sim::PhysicsWorld::Get();
  AutoShots out;
  int shots = world.GetMetrics().shotsFired;
  double arrivedAt = -1;
  bool shotThisStep = false;
  for (int i = 0; i < 1000; i++) {  // 20 s
    h.Step();
    const double t = (i + 1) * 0.02;
    const bool aligning = graph->GetString("state", "").rfind("align-shoot", 0) == 0;
    if (aligning && arrivedAt < 0) {
      arrivedAt = t;
      shotThisStep = false;
    } else if (!aligning) {
      arrivedAt = -1;
    }
    const int now = world.GetMetrics().shotsFired;
    for (; shots < now; shots++) {
      out.errorsDeg.push_back(AimErrorDeg(h));
      if (arrivedAt >= 0 && !shotThisStep) {
        out.arriveToFirstShot.push_back(t - arrivedAt);
        shotThisStep = true;
      }
    }
  }
  h.Disable();
  h.Step(2);
  return out;
}

class AutoShotTest : public ::testing::TestWithParam<std::tuple<std::string, bool>> {};

TEST_P(AutoShotTest, ShootsSoonAfterArrivingAndLinedUp) {
  const auto& [name, red] = GetParam();
  const auto r = RunAutoShots(name, red);
  std::string waits;
  for (double w : r.arriveToFirstShot) {
    waits += std::to_string(w).substr(0, 4) + " ";
  }
  const double worst =
      r.errorsDeg.empty() ? 0 : *std::max_element(r.errorsDeg.begin(), r.errorsDeg.end());
  std::printf("[auto-shot] %s %s: arrive->first shot %s s, %zu shots, worst aim %.1f deg\n",
              name.c_str(), red ? "red" : "blue", waits.c_str(), r.errorsDeg.size(), worst);
  ASSERT_FALSE(r.arriveToFirstShot.empty()) << "never shot";
  for (double w : r.arriveToFirstShot) {
    EXPECT_LT(w, 0.5) << "pause between arriving and the first shot";
  }
  EXPECT_LE(worst, 3.5) << "shot before lining up";
}

std::vector<std::tuple<std::string, bool>> AllAutos() {
  std::vector<std::tuple<std::string, bool>> out;
  for (const auto& r : mayhem_generated::routines) {
    out.emplace_back(std::string{r.name}, false);
    out.emplace_back(std::string{r.name}, true);
  }
  return out;
}

INSTANTIATE_TEST_SUITE_P(EveryAuto, AutoShotTest, ::testing::ValuesIn(AllAutos()),
                         [](const auto& info) {
                           std::string out;
                           for (char c : std::get<0>(info.param)) {
                             if (std::isalnum(static_cast<unsigned char>(c))) {
                               out += c;
                             }
                           }
                           return out + (std::get<1>(info.param) ? "Red" : "Blue");
                         });

}  // namespace
