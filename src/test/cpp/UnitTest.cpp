// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <memory>
#include <set>
#include <source_location>
#include <string>
#include <thread>
#include <vector>

#include <frc/DataLogManager.h>
#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/simulation/DriverStationSim.h>
#include <frc/simulation/SimHooks.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/Commands.h>
#include <frc2/command/SubsystemBase.h>
#include <gtest/gtest.h>
#include <wpi/DataLogReader.h>
#include <wpi/MemoryBuffer.h>
#include <wpi/json.h>
#include <ctre/phoenix6/TalonFX.hpp>

#include "FieldConstants.h"
#include "TestUtil.h"
#include "logging/Logger.h"
#include "mayhem/Auto.h"
#include "subsystems/linearslide/LinearSlideConstants.h"
#include "subsystems/shooter/ShooterConstants.h"
#include "subsystems/shooterpivot/ShooterPivotConstants.h"
#include "subsystems/vision/Vision.h"
#include "subsystems/vision/VisionConstants.h"
#include "subsystems/vision/VisionIOSim.h"
#include "utils/DisableSubsystem.h"
#include "utils/InterpolatingMap.h"
#include "utils/PhoenixUtil.h"
#include "utils/Util.h"

// ----------------------------------------------------------------------------------------- utils
TEST(InterpolatingMapTest, MatchesInterpolatingDoubleTreeMap) {
  constexpr util::InterpolatingMap<3> m{
      {{{2.0, 20.0}, {1.0, 10.0}, {4.0, 0.0}}}};  // unsorted input
  EXPECT_DOUBLE_EQ(m.Get(0.0), 10.0);             // below range: first value
  EXPECT_DOUBLE_EQ(m.Get(1.0), 10.0);             // exact keys
  EXPECT_DOUBLE_EQ(m.Get(2.0), 20.0);
  EXPECT_DOUBLE_EQ(m.Get(1.5), 15.0);  // linear
  EXPECT_DOUBLE_EQ(m.Get(3.0), 10.0);
  EXPECT_DOUBLE_EQ(m.Get(4.0), 0.0);
  EXPECT_DOUBLE_EQ(m.Get(100.0), 0.0);  // above range: last value
  // compile-time evaluation works too (no runtime cost for the tables)
  static_assert(util::InterpolatingMap<2>{{{{0.0, 0.0}, {1.0, 2.0}}}}.Get(0.25) == 0.5);
}

TEST(InterpolatingMapTest, RobotTablesAreSortedAndClamped) {
  EXPECT_DOUBLE_EQ(shooter::kHubLUT.Get(0.0), 65.8);
  EXPECT_DOUBLE_EQ(shooter::kHubLUT.Get(10.0), 100.0);
  EXPECT_NEAR(shooter::kHubLUT.Get((1.62 + 2.71) / 2), (65.8 + 74.8) / 2, 1e-12);
  EXPECT_DOUBLE_EQ(shooter::kFeedLUT.Get(20.0), 95.0);
  EXPECT_DOUBLE_EQ(shooter_pivot::kHubLUT.Get(3.95), 0.069);
  EXPECT_DOUBLE_EQ(shooter_pivot::kFeedLUT.Get(7.0), 0.01);
}

TEST(UtilTest, MatchesJavaUtil) {
  EXPECT_EQ(util::Gcd(12, 18), 6);
  EXPECT_EQ(util::Lcm(4, 6), 12);
  EXPECT_TRUE(util::EpsilonEquals(1.0, 1.04, 0.05));
  EXPECT_FALSE(util::EpsilonEquals(1.0, 1.06, 0.05));
  EXPECT_DOUBLE_EQ(util::Limit(5.0, 2.0), 2.0);
  EXPECT_DOUBLE_EQ(util::Interpolate(0, 10, 1.5), 10);
  EXPECT_DOUBLE_EQ(util::HandleDeadband(0.05, 0.1), 0.0);
  EXPECT_NEAR(util::HandleDeadband(0.55, 0.1), 0.5, 1e-12);
  EXPECT_NEAR(util::HandleDeadband(-0.55, 0.1), -0.5, 1e-12);
  EXPECT_DOUBLE_EQ(util::HandleDeadband(0.7, 1.0), 0.0);
  const std::vector<double> zones{0.0, 1.0, 2.0};
  EXPECT_DOUBLE_EQ(util::SnapToClosest(zones, 1.4), 1.0);
  EXPECT_DOUBLE_EQ(util::SnapToZone(zones, 1.4, 0.2), 1.4);
  EXPECT_DOUBLE_EQ(util::SnapToZone(zones, 1.1, 0.2), 1.0);
  EXPECT_DOUBLE_EQ(util::Signum(-3.0), -1.0);
  EXPECT_DOUBLE_EQ(util::Signum(0.0), 0.0);
}

namespace {
struct Probe {
  frc2::CommandPtr SetVelocity(std::source_location loc = std::source_location::current()) {
    name = DisableSubsystem::CallerName(loc);
    return frc2::cmd::None();
  }
  std::string name;
};

class TestSubsystem : public DisableSubsystem {
 public:
  explicit TestSubsystem(bool enabled) : DisableSubsystem(enabled, "TestSubsystem") {}
  frc2::CommandPtr SetVelocity(double v) {
    return Run([this, v] { last = v; });
  }
  frc2::CommandPtr Off() {
    return RunOnce([this] { last = 0; });
  }
  double last = -1;
};
}  // namespace

TEST(DisableSubsystemTest, CommandsAreNamedAfterTheirFactoryMethod) {
  TestSubsystem s{true};
  auto vel = s.SetVelocity(1);
  auto off = s.Off();
  EXPECT_EQ(vel.get()->GetName(), "setVelocity");
  EXPECT_EQ(off.get()->GetName(), "off.runOnce");
  EXPECT_TRUE(vel.get()->HasRequirement(&s));
  frc2::CommandScheduler::GetInstance().UnregisterSubsystem(&s);
}

TEST(DisableSubsystemTest, DisabledSubsystemsReturnNoOps) {
  TestSubsystem s{false};
  EXPECT_TRUE(s.disabled);
  auto c = s.SetVelocity(3);
  EXPECT_FALSE(c.get()->HasRequirement(&s));
  frc2::CommandScheduler::GetInstance().UnregisterSubsystem(&s);
}

// ------------------------------------------------------------------------------- field / vision
TEST(FieldTest, ConstantsMatchTheDeployedLayout) {
  std::ifstream in{test_util::RepoPath("src/main/deploy/apriltags/welded/2026-official.json")};
  wpi::json j = wpi::json::parse(in);
  EXPECT_DOUBLE_EQ(j["field"]["length"].get<double>(), field_constants::kFieldLength);
  EXPECT_DOUBLE_EQ(j["field"]["width"].get<double>(), field_constants::kFieldWidth);
  EXPECT_EQ(j["tags"].size(), static_cast<size_t>(field_constants::kAprilTagCount));
  auto tag = [&](int id) {
    for (auto& t : j["tags"]) {
      if (t["ID"].get<int>() == id)
        return t["pose"]["translation"];
    }
    throw std::runtime_error("no tag");
  };
  EXPECT_DOUBLE_EQ(tag(26)["x"].get<double>(), field_constants::kTag26X);
  EXPECT_DOUBLE_EQ(tag(4)["x"].get<double>(), field_constants::kTag4X);
  EXPECT_DOUBLE_EQ(tag(10)["x"].get<double>(), field_constants::kTag10X);
  EXPECT_DOUBLE_EQ(tag(31)["y"].get<double>(), field_constants::kTag31Y);
  EXPECT_DOUBLE_EQ(tag(15)["y"].get<double>(), field_constants::kTag15Y);
  EXPECT_DOUBLE_EQ(tag(29)["y"].get<double>(), field_constants::kTag29Y);
}

TEST(FieldTest, WpilibDefaultFieldMatchesTheDeployedLayout) {
  // Vision uses WPILib's built-in 2026 field; FieldConstants uses the deployed file.
  const auto& layout = vision::AprilTagLayout();
  EXPECT_NEAR(layout.GetFieldLength().value(), field_constants::kFieldLength, 1e-3);
  EXPECT_NEAR(layout.GetTagPose(26)->X().value(), field_constants::kTag26X, 1e-3);
  EXPECT_NEAR(layout.GetTagPose(4)->X().value(), field_constants::kTag4X, 1e-3);
}

namespace {
class FakeVisionIO : public VisionIO {
 public:
  void UpdateInputs(VisionIOInputs& in) override {
    in.connected = connected;
    in.poseObservations = observations;
    in.tagIds.clear();
  }
  bool connected = true;
  std::vector<PoseObservation> observations;
};

PoseObservation Obs(double x, double y, double z, double ambiguity, int tags, double dist,
                    std::vector<short> ids) {
  PoseObservation o;
  o.timestamp = 1.0;
  o.pose = frc::Pose3d{units::meter_t{x}, units::meter_t{y}, units::meter_t{z}, frc::Rotation3d{}};
  o.ambiguity = ambiguity;
  o.tagCount = tags;
  o.averageTagDistance = dist;
  o.tagIds = std::move(ids);
  return o;
}
}  // namespace

TEST(VisionTest, RejectsBadObservationsAndScalesStdDevs) {
  struct Got {
    frc::Pose2d pose;
    double t;
    std::array<double, 3> sd;
  };
  std::vector<Got> got;
  auto io = std::make_unique<FakeVisionIO>();
  auto* fake = io.get();
  std::vector<std::unique_ptr<VisionIO>> ios;
  ios.push_back(std::move(io));
  Vision v{[&](const frc::Pose2d& p, units::second_t t, const std::array<double, 3>& sd) {
             got.push_back({p, t.value(), sd});
           },
           std::move(ios)};
  fake->observations = {
      Obs(2, 3, 0, 0.1, 2, 2.0, {18, 21}),    // good, hub tags: factor 4/2 = 2
      Obs(2, 3, 0, 0.5, 1, 1.0, {18}),        // single tag, ambiguous -> rejected
      Obs(2, 3, 1.0, 0.0, 2, 1.0, {18, 21}),  // z error -> rejected
      Obs(-1, 3, 0, 0.0, 2, 1.0, {18, 21}),   // off the field -> rejected
      Obs(2, 30, 0, 0.0, 2, 1.0, {18, 21}),   // off the field -> rejected
      Obs(2, 3, 0, 0.0, 0, 1.0, {}),          // no tags -> rejected
      Obs(5, 4, 0, 0.2, 1, 3.0, {7}),         // non-hub tag: x1.3
  };
  v.Periodic();
  ASSERT_EQ(got.size(), 2u);
  EXPECT_EQ(v.AcceptedCount(), 2);
  EXPECT_EQ(v.RejectedCount(), 5);
  EXPECT_NEAR(got[0].sd[0], vision::kLinearStdDevBaseline * 2.0, 1e-12);
  EXPECT_NEAR(got[0].sd[2], vision::kAngularStdDevBaseline * 2.0, 1e-12);
  EXPECT_NEAR(got[1].sd[0], vision::kLinearStdDevBaseline * 9.0 * 1.3, 1e-12);
  EXPECT_NEAR(got[1].pose.X().value(), 5.0, 1e-12);
  frc2::CommandScheduler::GetInstance().UnregisterSubsystem(&v);
}

TEST(VisionTest, SimulatedCameraSeesTagsInItsFieldOfView) {
  frc::Pose2d pose{2.0_m, 4.0_m, frc::Rotation2d{}};
  VisionIOSim sim{vision::kRobotToBackCam, [&] { return pose; }};
  // back camera at x = 2 m looking at the blue wall: the blue tower tags are visible
  auto tags = sim.VisibleTags(pose);
  EXPECT_FALSE(tags.empty());
  // turned around, the back camera faces the far field: no blue wall tags
  auto facingAway = sim.VisibleTags(frc::Pose2d{2.0_m, 4.0_m, frc::Rotation2d{180_deg}});
  for (int id : facingAway) {
    EXPECT_FALSE(std::find(tags.begin(), tags.end(), id) != tags.end()) << id;
  }
  VisionIOInputs in;
  sim.UpdateInputs(in);
  ASSERT_EQ(in.poseObservations.size(), 1u);
  EXPECT_NEAR(in.poseObservations[0].pose.X().value(), 2.0, 1e-9);
  EXPECT_EQ(static_cast<size_t>(in.poseObservations[0].tagCount), tags.size());
}

// ------------------------------------------------------------------------------------- phoenix
TEST(PhoenixTest, AppliedConfigsReadBackEqual) {
  ctre::phoenix6::hardware::TalonFX motor{61};
  for (const auto& config : {shooter::MotorConfigs(), shooter_pivot::MotorConfigs(),
                             linear_slide::RightMotorConfigs(), linear_slide::LeftMotorConfigs()}) {
    ASSERT_TRUE(phoenix_util::ApplyMotorConfigs(motor, config, 5));
    EXPECT_TRUE(phoenix_util::ReadAndVerifyConfiguration(motor, config));
  }
  // a different config does not verify
  auto other = shooter::MotorConfigs();
  other.Slot0.kP = 5.0;
  EXPECT_FALSE(phoenix_util::ReadAndVerifyConfiguration(motor, other));
  EXPECT_EQ(linear_slide::LeftMotorConfigs().MotorOutput.Inverted.value,
            ctre::phoenix6::signals::InvertedValue::CounterClockwise_Positive);
  EXPECT_EQ(linear_slide::LeftMotorConfigs().Slot0.kP.value(),
            linear_slide::RightMotorConfigs().Slot0.kP.value());
}

TEST(PhoenixTest, SignalsAreRefreshedTogether) {
  phoenix_util::ClearRegisteredSignals();
  ctre::phoenix6::hardware::TalonFX motor{62};
  phoenix_util::RegisterSignals(false, {&motor.GetVelocity(false), &motor.GetPosition(false)});
  EXPECT_EQ(phoenix_util::RegisteredSignalCount(), 2u);
  phoenix_util::RefreshAll();  // must not throw or block
  phoenix_util::ClearRegisteredSignals();
}

// -------------------------------------------------------------------------------------- mayhem
TEST(MayhemTest, EveryDeployedTrajectoryLoads) {
  // Same check as the Java MayhemTrajectoriesTest.
  test_util::UseRepoDeployDir();
  namespace fs = std::filesystem;
  int n = 0;
  for (const auto& f : fs::directory_iterator{test_util::RepoPath("src/main/deploy/mayhem")}) {
    if (f.path().extension() != ".mtraj") {
      continue;
    }
    const std::string name = f.path().stem().string();
    auto traj = mayhem::trajectory_loader::LoadFile(f.path().string());
    EXPECT_EQ(name, traj->Name()) << "file name should match the trajectory name";
    EXPECT_FALSE(traj->Samples().empty()) << name;
    EXPECT_GT(traj->TotalTime(), 0) << name;
    EXPECT_NE(traj->Recovery(), nullptr) << name;
    EXPECT_TRUE(std::isfinite(traj->InitialPose().X().value()) &&
                std::isfinite(traj->FinalPose().X().value()));
    n++;
  }
  EXPECT_EQ(n, 10);
}

TEST(MayhemTest, LoaderRejectsBadFiles) {
  EXPECT_THROW(mayhem::trajectory_loader::Parse("{not json"), std::invalid_argument);
  EXPECT_THROW(mayhem::trajectory_loader::Parse(R"({"name":"x","formatVersion":2,"output":{}})"),
               std::invalid_argument);
  EXPECT_THROW(mayhem::trajectory_loader::Parse(R"({"name":"x","formatVersion":1})"),
               std::invalid_argument);
  EXPECT_THROW(mayhem::trajectory_loader::LoadFile("/nonexistent.mtraj"), std::invalid_argument);
}

namespace {
/** A drivetrain that does exactly what it is told. */
class FakeDrive : public frc2::SubsystemBase {
 public:
  FakeDrive() : SubsystemBase("FakeDrive") {}
  frc::Pose2d pose;
  mayhem::DriveCommand last;
};
}  // namespace

/** Routine/trigger semantics (Choreo-style), with stepped simulated time and an ideal drivetrain.
 */
TEST(MayhemTest, RoutineTriggersFireOnceInOrder) {
  test_util::UseRepoDeployDir();
  frc::sim::PauseTiming();
  frc::sim::DriverStationSim::ResetData();
  frc::sim::DriverStationSim::SetAllianceStationId(HAL_AllianceStationID_kBlue1);
  frc::sim::DriverStationSim::SetEnabled(true);
  frc::sim::DriverStationSim::SetAutonomous(true);
  frc::sim::DriverStationSim::NotifyNewData();
  frc::DriverStation::RefreshData();

  FakeDrive drive;
  mayhem::AutoFactory factory{[&] { return drive.pose; },
                              [&](const frc::Pose2d& p) { drive.pose = p; },
                              [&] { return drive.last.fieldSpeeds; },
                              [&](const mayhem::DriveCommand& dc) { drive.last = dc; },
                              true,
                              &drive};
  std::vector<std::string> log;
  frc2::CommandPtr routineCmd = [&] {
    auto routine = factory.NewRoutine("test");
    auto& traj = routine->Trajectory("trenchDepotOnly");
    routine->Active().OnTrue(traj.ResetOdometry().AndThen(traj.Cmd()));
    routine->Active().OnTrue(frc2::cmd::RunOnce([&] { log.push_back("active"); }));
    traj.AtTime("intake").OnTrue(frc2::cmd::RunOnce([&] { log.push_back("intake"); }));
    traj.AtTime(2.0).OnTrue(frc2::cmd::RunOnce([&] { log.push_back("t2"); }));
    traj.AtTime("shoot").OnTrue(frc2::cmd::RunOnce([&] { log.push_back("shoot"); }));
    traj.Done().OnTrue(frc2::cmd::RunOnce([&] { log.push_back("done"); }));
    return routine->Cmd();
  }();
  auto& scheduler = frc2::CommandScheduler::GetInstance();
  scheduler.Schedule(routineCmd.get());
  const auto t = mayhem::trajectory_loader::Load("trenchDepotOnly");
  for (int i = 0; i < 300; i++) {
    frc::sim::StepTiming(20_ms);
    // ideal plant: the robot moves exactly as commanded
    const auto& s = drive.last.fieldSpeeds;
    drive.pose = frc::Pose2d{drive.pose.X() + s.vx * 20_ms, drive.pose.Y() + s.vy * 20_ms,
                             drive.pose.Rotation() + frc::Rotation2d{s.omega * 20_ms}};
    scheduler.Run();
    if (i == 1) {
      EXPECT_LT(drive.pose.Translation().Distance(t->InitialPose().Translation()).value(), 0.05);
    }
  }
  EXPECT_EQ(log, (std::vector<std::string>{"active", "intake", "t2", "shoot", "done"}));
  EXPECT_LT(drive.pose.Translation().Distance(t->FinalPose().Translation()).value(), 0.05);
  EXPECT_TRUE(routineCmd.get()->IsScheduled());  // a routine runs until autonomous ends

  // cancelling resets it; running it again replays the whole thing
  scheduler.Cancel(routineCmd.get());
  log.clear();
  scheduler.Schedule(routineCmd.get());
  for (int i = 0; i < 300; i++) {
    frc::sim::StepTiming(20_ms);
    const auto& s = drive.last.fieldSpeeds;
    drive.pose = frc::Pose2d{drive.pose.X() + s.vx * 20_ms, drive.pose.Y() + s.vy * 20_ms,
                             drive.pose.Rotation() + frc::Rotation2d{s.omega * 20_ms}};
    scheduler.Run();
  }
  EXPECT_EQ(log, (std::vector<std::string>{"active", "intake", "t2", "shoot", "done"}));
  scheduler.Cancel(routineCmd.get());
  scheduler.GetDefaultButtonLoop()->Clear();
  scheduler.UnregisterSubsystem(&drive);
  frc::sim::ResumeTiming();
}

TEST(MayhemTest, RedAllianceRunsTheFlippedPath) {
  test_util::UseRepoDeployDir();
  frc::sim::DriverStationSim::SetAllianceStationId(HAL_AllianceStationID_kRed1);
  frc::sim::DriverStationSim::NotifyNewData();
  frc::DriverStation::RefreshData();
  FakeDrive drive;
  mayhem::AutoFactory factory{[&] { return drive.pose; },
                              [&](const frc::Pose2d& p) { drive.pose = p; },
                              [] { return frc::ChassisSpeeds{}; },
                              [](const mayhem::DriveCommand&) {},
                              true,
                              &drive};
  auto routine = factory.NewRoutine("red");
  auto& traj = routine->Trajectory("trenchDepotOnly");
  auto start = traj.GetInitialPose();
  ASSERT_TRUE(start.has_value());
  const auto blue = mayhem::trajectory_loader::Load("trenchDepotOnly")->InitialPose();
  EXPECT_NEAR(start->X().value(), field_constants::fieldLength - blue.X().value(), 1e-3);
  EXPECT_NEAR(start->Y().value(), 8.0692 - blue.Y().value(), 1e-6);
  // a missing trajectory gives an inert placeholder instead of crashing
  auto& missing = routine->Trajectory("doesNotExist");
  EXPECT_FALSE(missing.IsLoaded());
  EXPECT_FALSE(missing.GetInitialPose().has_value());
  frc2::CommandScheduler::GetInstance().UnregisterSubsystem(&drive);
  frc::sim::DriverStationSim::SetAllianceStationId(HAL_AllianceStationID_kBlue1);
  frc::sim::DriverStationSim::NotifyNewData();
}

// ------------------------------------------------------------------------------------- logging
TEST(LoggingTest, ValuesReachTheLogFileWithAdvantageKitKeys) {
  namespace fs = std::filesystem;
  const fs::path dir = fs::temp_directory_path() / ("robot-log-test-" + std::to_string(::getpid()));
  fs::remove_all(dir);
  logging::Logger::Start(dir.string());
  ASSERT_NE(logging::Logger::GetLog(), nullptr);
  {
    logging::Output<double> out{"Test/Value"};
    logging::Output<frc::Pose2d> pose{"Test/Pose"};
    double input = 0;
    logging::InputsLogger inputs{"TestInputs"};
    inputs.Add("field", &input);
    for (int i = 0; i < 5; i++) {
      out.Set(i * 1.5);
      pose.Set(frc::Pose2d{units::meter_t{1.0 * i}, 2_m, frc::Rotation2d{}});
      input = i;
      inputs.Publish();
    }
    out.Set(6.0);
    out.Set(6.0);  // unchanged values are not written again
  }
  logging::Logger::GetLog()->Flush();
  std::this_thread::sleep_for(std::chrono::milliseconds(600));

  std::set<std::string> names;
  std::map<int, std::string> ids;
  int valueRecords = 0;
  for (const auto& f : fs::directory_iterator{dir}) {
    auto buf = wpi::MemoryBuffer::GetFile(f.path().string());
    ASSERT_TRUE(buf) << f.path();
    wpi::log::DataLogReader reader{std::move(*buf)};
    ASSERT_TRUE(reader.IsValid());
    for (const auto& rec : reader) {
      wpi::log::StartRecordData start;
      if (rec.GetStartData(&start)) {
        names.insert(std::string{start.name});
        ids[start.entry] = std::string{start.name};
      } else if (!rec.IsControl() && ids[rec.GetEntry()] == "/RealOutputs/Test/Value") {
        valueRecords++;
      }
    }
  }
  EXPECT_TRUE(names.contains("/RealOutputs/Test/Value"));
  EXPECT_TRUE(names.contains("/RealOutputs/Test/Pose"));
  EXPECT_TRUE(names.contains("/TestInputs/field"));
  EXPECT_EQ(valueRecords, 5);  // 0, 1.5, 3, 4.5, 6.0; the two repeated 6.0s are not written
}
