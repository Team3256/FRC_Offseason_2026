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
#include <sstream>
#include <string>
#include <vector>

#include <gtest/gtest.h>

#include "RobotHarness.h"
#include "mayhem/Trajectory.h"

using SS = Superstructure::StructureState;

namespace {

struct AutoCase {
  const char* chooserName;
  std::vector<const char*> trajectories;  // in the order they run
  int shots;                              // AlignAndShoot calls
};

frc::Pose2d Expected(const char* traj, bool red, bool end) {
  auto t = mayhem::trajectory_loader::Load(traj);
  if (red) {
    t = t->Flipped();
  }
  return end ? t->FinalPose() : t->InitialPose();
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
  const frc::Pose2d start = Expected(c.trajectories.front(), red, false);
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
  EXPECT_TRUE(resetSeen) << "odometry was not reset to the start of " << c.trajectories.front();

  // the robot ends where the last path ends (it may have turned in place to aim afterwards). A path
  // that ends moving hands over without braking, so the robot coasts on until AlignAndShoot.
  const frc::Pose2d end = Expected(c.trajectories.back(), red, true);
  const double endSpeed =
      mayhem::trajectory_loader::Load(c.trajectories.back())->FinalSample().Speed();
  EXPECT_LT(Distance(dt.State().Pose, end), 0.35 + 0.2 * endSpeed)
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
    {"Depot Trench Sweep Depot", {"DepotTrenchSweepDepotPt", "DepotTrenchSweepDepotPt2"}, 2},
    {"Outpost Trench Sweep", {"OutpostTrenchSweepBumpPt1"}, 1},
    {"Outpost Trench Sweep x2", {"OutpostTrenchSweepBumpPt1", "OutpostTrenchSweepBumpPt2"}, 2},
    {"Depot Trench Sweep", {"DepotTrenchSweepBumpPt1"}, 1},
    {"Depot Trench Sweep x2", {"DepotTrenchSweepBumpPt1", "DepotTrenchSweepBump2xPt2"}, 2},
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
  const double startX = Expected("DepotTrenchSweepBumpPt1", false, false).X().value();
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
