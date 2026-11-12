// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

// Every autonomous routine on the dashboard chooser, run end to end in the full-robot simulation.

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <numbers>
#include <sstream>
#include <string>
#include <vector>

#include <frc/MathUtil.h>
#include <gtest/gtest.h>

#include "FieldConstants.h"
#include "RobotHarness.h"
#include "commands/MayhemAutos.h"

using SS = Superstructure::StructureState;

namespace {

struct AutoCase {
  const char* chooserName;
  int shots;  // alignAndShoot steps
};

// Driving time allowed for each pass between shots (the old solved passes took 2.4-5.7 s).
constexpr double kPassBudgetSeconds = 7.5;
// Median angle between the intake (robot front) and the direction of travel while intaking near
// the centerline pickup. The old solved paths had 17-48 degrees.
constexpr double kIntakeLeadingDegrees = 35;

const robot_mayhem::AutoDefinition& Definition(const char* name) {
  const auto* def = suite_autos::Find(name);
  EXPECT_NE(def, nullptr) << name;
  return *def;
}

frc::Pose2d Expected(const robot_mayhem::AutoDefinition& def, bool red, bool end) {
  if (!end) {
    return suite_autos::ToWpi(suite_autos::Resolve(def.start, red));
  }
  mayhem::Pose last = def.start;
  for (std::size_t i = 0; i < def.routine.nodeCount; ++i) {
    if (def.routine.nodes[i].kind == mayhem::NodeKind::Pose) {
      last = def.routine.nodes[i].goal.pose;
    }
  }
  return suite_autos::ToWpi(suite_autos::Resolve(last, red));
}

double Distance(const frc::Pose2d& a, const frc::Pose2d& b) {
  return a.Translation().Distance(b.Translation()).value();
}

void RunAuto(const AutoCase& c, bool red) {
  RobotHarness h{red};
  h.Step(5);
  // Selected while disabled, like on the dashboard: the routine (and its paths) are built now.
  h.C().GetAutoChooser().Select(c.chooserName);
  h.Step(3);
  EXPECT_EQ(h.C().GetAutoChooser().Selected(), c.chooserName);

  h.Autonomous();
  std::vector<SS> visited;
  auto& ss = h.C().GetSuperstructure();
  auto& dt = h.C().GetDrivetrain();
  const auto& def = Definition(c.chooserName);
  const frc::Pose2d start = Expected(def, red, false);
  std::vector<double> passes, intakeErrors;
  double passStart = -1;
  frc::Pose2d lastPose = dt.State().Pose;
  bool resetSeen = false;
  auto status =
      nt::NetworkTableInstance::GetDefault().GetTable("AdvantageKit/RealOutputs/Mayhem/Auto");
  // Pose/condition completion replaces the legacy trajectory clock. The watchdog
  // remains bounded; a fault is a test failure, never treated as completion.
  std::string result, graphTrace, previousNode;
  std::ostringstream trace;
  trace << "time,x,y,heading,mode,intake,shot,node,status\n";
  double minY = 100, maxY = -100, minX = 100;
  auto graph = nt::NetworkTableInstance::GetDefault().GetTable("Mayhem/States/Autonomous");
  for (int i = 0; i < 3000; i++) {
    h.Step();
    if (visited.empty() || visited.back() != ss.GetState())
      visited.push_back(ss.GetState());
    if (!resetSeen && i < 10 && Distance(dt.State().Pose, start) < 0.05)
      resetSeen = true;
    result = status->GetString("Status", "");
    const auto node = graph->GetString("state", "");
    const auto pose = dt.State().Pose;
    const bool driving = node.find("-pass") != std::string::npos;
    if (driving && passStart < 0)
      passStart = i * .02;
    if (!driving && passStart >= 0) {
      passes.push_back(i * .02 - passStart);
      passStart = -1;
    }
    const auto step = pose.Translation() - lastPose.Translation();
    const double speed = step.Norm().value() / .02;
    if (driving && ss.GetState() == SS::INTAKE && speed > 0.8 &&
        std::abs(pose.X().value() - field_constants::fieldLength / 2) < 1.0) {
      const double travel = std::atan2(step.Y().value(), step.X().value());
      intakeErrors.push_back(std::abs(
          frc::AngleModulus(units::radian_t{pose.Rotation().Radians().value() - travel}).value()) *
                             180 / std::numbers::pi);
    }
    lastPose = pose;
    minY = std::min(minY, pose.Y().value());
    maxY = std::max(maxY, pose.Y().value());
    minX = std::min(minX, pose.X().value());
    trace << i * .02 << ',' << pose.X().value() << ',' << pose.Y().value() << ','
          << pose.Rotation().Radians().value() << ',' << Superstructure::ToString(ss.GetState())
          << ',' << ss.IntakePhase() << ',' << ss.ShotPhase() << ',' << node << ',' << result
          << '\n';
    if (node != previousNode) {
      graphTrace += node + " @ " + std::to_string(dt.State().Pose.X().value()) + "," +
                    std::to_string(dt.State().Pose.Y().value()) + "\n";
      previousNode = node;
    }
    if (result == "Completed" || result == "Failed")
      break;
  }
  if (const char* folder = std::getenv("MAYHEM_AUTO_TRACE_DIR")) {
    std::filesystem::create_directories(folder);
    std::ofstream out{std::string{folder} + "/" + c.chooserName + (red ? "-red" : "-blue") +
                      ".csv"};
    out << trace.str();
  }
  EXPECT_GT(minY, .32) << "bumper approaches the lower wall";
  EXPECT_LT(maxY, 8.069 - .32) << "bumper approaches the upper wall";
  EXPECT_GT(minX, .32) << "bumper approaches the alliance wall";
  EXPECT_EQ(result, "Completed") << c.chooserName << (red ? " red" : " blue") << "\n" << graphTrace;
  EXPECT_TRUE(resetSeen) << "odometry was not reset to the start of " << c.chooserName;

  ASSERT_EQ(passes.size(), static_cast<size_t>(c.shots));
  for (double pass : passes) {
    EXPECT_LT(pass, kPassBudgetSeconds) << c.chooserName << " drove too long before shooting";
  }
  ASSERT_GT(intakeErrors.size(), 10u) << "never intaked while driving through the pickup";
  std::sort(intakeErrors.begin(), intakeErrors.end());
  EXPECT_LT(intakeErrors[intakeErrors.size() / 2], kIntakeLeadingDegrees)
      << c.chooserName << ": the intake must lead through the pickup";

  // the robot ends where the last pass ends (it may have turned in place to aim afterwards)
  const frc::Pose2d end = Expected(def, red, true);
  EXPECT_LT(Distance(dt.State().Pose, end), 0.3)
      << c.chooserName << (red ? " red" : " blue") << ": ended at (" << dt.State().Pose.X().value()
      << ", " << dt.State().Pose.Y().value() << "), path ends at (" << end.X().value() << ", "
      << end.Y().value() << ")";

  std::string seq;
  for (auto s : visited) {
    seq += std::string{Superstructure::ToString(s)} + " ";
  }
  // the intake marker fired
  EXPECT_NE(std::find(visited.begin(), visited.end(), SS::INTAKE), visited.end()) << seq;
  // and the routine ended by shooting
  if (c.shots > 0) {
    EXPECT_TRUE(ss.GetState() == SS::SHOOT || ss.GetState() == SS::SHOOT_AND_INTAKE ||
                ss.GetState() == SS::JITTER_AND_SHOOT)
        << seq;
  }

  // Leaving autonomous stops the routine
  h.Disable();
  h.Step(3);
  EXPECT_EQ(dt.GetCurrentCommand(), nullptr);
}

const std::vector<AutoCase> kAutos = {
    {"Depot Trench Sweep Depot", 2}, {"Outpost Trench Sweep", 1}, {"Outpost Trench Sweep x2", 2},
    {"Depot Trench Sweep", 1},       {"Depot Trench Sweep x2", 2},
};

}  // namespace

TEST(AutoSimTest, ChooserListsEveryRoutine) {
  RobotHarness h;
  auto options = h.C().GetAutoChooser().Options();
  ASSERT_EQ(options.size(), kAutos.size() + 1);
  EXPECT_EQ(options[0], "Nothing");
  for (size_t i = 0; i < kAutos.size(); i++) {
    EXPECT_EQ(options[i + 1], kAutos[i].chooserName);
  }
}

TEST(AutoSimTest, NothingDoesNothing) {
  RobotHarness h;
  h.C().GetDrivetrain().ResetPose(frc::Pose2d{8_m, 4_m, frc::Rotation2d{}});
  h.Step(3);
  h.Autonomous();
  h.Step(50);
  EXPECT_LT(h.C()
                .GetDrivetrain()
                .State()
                .Pose.Translation()
                .Distance(frc::Translation2d{8_m, 4_m})
                .value(),
            0.02);
  EXPECT_EQ(h.C().GetSuperstructure().GetState(), SS::IDLE);
}

TEST(AutoSimTest, DepotTrenchSweepDepotBlue) { RunAuto(kAutos[0], false); }
TEST(AutoSimTest, OutpostTrenchSweepBlue) { RunAuto(kAutos[1], false); }
TEST(AutoSimTest, OutpostTrenchSweepX2Blue) { RunAuto(kAutos[2], false); }
TEST(AutoSimTest, DepotTrenchSweepBlue) { RunAuto(kAutos[3], false); }
TEST(AutoSimTest, DepotTrenchSweepX2Blue) { RunAuto(kAutos[4], false); }
TEST(AutoSimTest, DepotTrenchSweepRed) { RunAuto(kAutos[3], true); }
TEST(AutoSimTest, DepotTrenchSweepDepotRed) { RunAuto(kAutos[0], true); }
TEST(AutoSimTest, OutpostTrenchSweepRed) { RunAuto(kAutos[1], true); }
TEST(AutoSimTest, DepotTrenchSweepX2Red) { RunAuto(kAutos[4], true); }
TEST(AutoSimTest, OutpostTrenchSweepX2Red) { RunAuto(kAutos[2], true); }

TEST(AutoSimTest, DisablingMidAutoStopsEverything) {
  RobotHarness h;
  h.Step(3);
  h.C().GetAutoChooser().Select("Depot Trench Sweep");
  h.Step(2);
  h.Autonomous();
  h.Step(60);
  auto& dt = h.C().GetDrivetrain();
  // the path follower (inside the routine's reset-then-follow sequence) owns the drivetrain
  ASSERT_NE(dt.GetCurrentCommand(), nullptr);
  EXPECT_NE(dt.GetCurrentCommand(), dt.GetDefaultCommand());
  const double startX = Expected(Definition("Depot Trench Sweep"), false, false).X().value();
  h.Disable();
  h.Step(2);
  EXPECT_EQ(dt.GetCurrentCommand(), nullptr);
  h.Step(25);  // the simulated drivetrain coasts to a stop
  const auto p = dt.State().Pose;
  h.Step(25);
  EXPECT_LT(dt.State().Pose.Translation().Distance(p.Translation()).value(), 0.05);
  // re-enabling autonomous starts the routine again from the beginning (odometry reset to the
  // start)
  h.Autonomous();
  h.Step(3);
  ASSERT_NE(dt.GetCurrentCommand(), nullptr);
  EXPECT_NE(dt.GetCurrentCommand(), dt.GetDefaultCommand());
  EXPECT_NEAR(dt.State().Pose.X().value(), startX, 0.1);
  h.Disable();
  h.Step(2);
}
