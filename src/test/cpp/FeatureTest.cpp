// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

// The dashboard-only features: robot and device alerts, the auto preview / start-pose helper and
// shot readiness telemetry. None of them may change what the robot does.

#include <algorithm>
#include <string>
#include <vector>

#include <gtest/gtest.h>
#include <networktables/NetworkTableInstance.h>

#include "RobotHarness.h"
#include "commands/MayhemAutos.h"
#include "utils/RobotAlerts.h"

using SS = Superstructure::StructureState;

namespace {
std::vector<std::string> ActiveAlerts(const char* kind) {
  return nt::NetworkTableInstance::GetDefault()
      .GetTable("SmartDashboard/Alerts")
      ->GetStringArray(kind, {});
}

bool Contains(const std::vector<std::string>& v, const std::string& s) {
  return std::find(v.begin(), v.end(), s) != v.end();
}

double Distance(const frc::Pose2d& a, const frc::Pose2d& b) {
  return a.Translation().Distance(b.Translation()).value();
}

void Feed(RobotAlerts& a, const RobotAlerts::Inputs& in, int loops) {
  for (int i = 0; i < loops; i++) {
    a.Update(in);
  }
}
}  // namespace

// ------------------------------------------------------------------------------- RobotAlerts logic

TEST(RobotAlertsTest, LowBatteryOnlyJudgedWhileDisabledAndDebounced) {
  RobotAlerts a;
  RobotAlerts::Inputs in;
  in.batteryVolts = 11.6;
  Feed(a, in, RobotAlerts::kLowBatteryLoops - 1);
  EXPECT_FALSE(a.LowBattery());
  Feed(a, in, 1);
  EXPECT_TRUE(a.LowBattery());
  // Sagging under load while enabled neither raises nor clears it.
  in.disabled = false;
  in.batteryVolts = 12.6;
  Feed(a, in, 500);
  EXPECT_TRUE(a.LowBattery());
  in.disabled = true;
  Feed(a, in, RobotAlerts::kLowBatteryLoops);
  EXPECT_FALSE(a.LowBattery());

  RobotAlerts b;
  RobotAlerts::Inputs enabled;
  enabled.disabled = false;
  enabled.batteryVolts = 9.0;
  Feed(b, enabled, 500);
  EXPECT_FALSE(b.LowBattery());
}

TEST(RobotAlertsTest, ControllerBrownoutCanOverrunAndAutoSelection) {
  RobotAlerts a;
  RobotAlerts::Inputs in;
  in.driverControllerConnected = false;
  Feed(a, in, RobotAlerts::kControllerLoops - 1);
  EXPECT_FALSE(a.DriverControllerDisconnected());
  Feed(a, in, 1);
  EXPECT_TRUE(a.DriverControllerDisconnected());
  in.driverControllerConnected = true;
  Feed(a, in, 1);
  EXPECT_FALSE(a.DriverControllerDisconnected());

  in.brownedOut = true;
  Feed(a, in, 1);
  in.brownedOut = false;
  Feed(a, in, 100);
  EXPECT_TRUE(a.BrownedOut()) << "brownouts stay latched";

  in.canUtilization = 0.95;
  Feed(a, in, RobotAlerts::kHighCanSamples);
  EXPECT_TRUE(a.HighCanUtilization());
  in.canUtilization.reset();  // not sampled this loop: no change
  Feed(a, in, 100);
  EXPECT_TRUE(a.HighCanUtilization());
  in.canUtilization = 0.4;
  Feed(a, in, RobotAlerts::kHighCanSamples);
  EXPECT_FALSE(a.HighCanUtilization());

  in.loopTimeMs = 25;
  Feed(a, in, RobotAlerts::kOverrunLoops - 1);
  in.loopTimeMs = 3;  // one good loop resets the count
  Feed(a, in, 1);
  in.loopTimeMs = 25;
  Feed(a, in, RobotAlerts::kOverrunLoops - 1);
  EXPECT_FALSE(a.LoopOverrun());
  Feed(a, in, 1);
  EXPECT_TRUE(a.LoopOverrun());
  in.loopTimeMs = 3;
  Feed(a, in, RobotAlerts::kOverrunClearLoops);
  EXPECT_FALSE(a.LoopOverrun());

  in.autoSelected = false;
  Feed(a, in, 1);
  EXPECT_TRUE(a.NoAutoSelected());
  in.disabled = false;
  Feed(a, in, 1);
  EXPECT_FALSE(a.NoAutoSelected()) << "only a pre-match warning";
}

// ------------------------------------------------------------------------------- full robot (sim)

TEST(FeatureSimTest, EveryDeviceIsWatchedAndHealthyInSimulation) {
  RobotHarness h;
  h.Step(30);
  // 13 mechanism TalonFXs, 4 swerve modules x (drive, steer, CANcoder), and the Pigeon 2.
  EXPECT_EQ(phoenix_util::WatchedDeviceCount(), 26u);
  EXPECT_EQ(phoenix_util::DisconnectedDeviceCount(), 0);
  EXPECT_EQ(phoenix_util::HotDeviceCount(), 0);
  EXPECT_EQ(phoenix_util::ConfigFailureCount(), 0u);
  const auto& a = h.R().Alerts();
  EXPECT_FALSE(a.LowBattery());
  EXPECT_FALSE(a.DriverControllerDisconnected());
  EXPECT_FALSE(a.BrownedOut());
  EXPECT_FALSE(a.HighCanUtilization());
  EXPECT_FALSE(a.LoopOverrun());
  EXPECT_TRUE(ActiveAlerts("errors").empty());
  EXPECT_TRUE(Contains(ActiveAlerts("warnings"), "No autonomous routine is selected."));

  h.C().GetAutoChooser().Select("Depot Trench Sweep");
  h.Step(2);
  EXPECT_FALSE(Contains(ActiveAlerts("warnings"), "No autonomous routine is selected."));
}

TEST(FeatureSimTest, AutoPreviewShowsTheSelectedRoutineForEachAlliance) {
  for (bool red : {false, true}) {
    RobotHarness h{red};
    h.Step(5);
    EXPECT_FALSE(h.C().GetAutoStartPose().has_value()) << "nothing selected";

    h.C().GetAutoChooser().Select("Depot Trench Sweep x2");
    h.Step(2);
    const auto* definition = suite_autos::Find("Depot Trench Sweep x2");
    ASSERT_NE(definition, nullptr);
    const auto first = suite_autos::ToWpi(suite_autos::Resolve(definition->start, red));
    ASSERT_TRUE(h.C().GetAutoStartPose().has_value());
    EXPECT_LT(Distance(*h.C().GetAutoStartPose(), first), 1e-9);
    const auto path = h.C().GetField().GetObject("AutoPath")->GetPoses();
    EXPECT_GT(path.size(), 20u) << "both paths are drawn";
    EXPECT_LT(Distance(path.front(), first), 1e-9);
    const auto expected = suite_autos::Preview(*definition, red);
    EXPECT_EQ(path.size(), expected.size());
    EXPECT_LT(Distance(path.back(), expected.back()), 1e-9);

    // Off the start pose: not lined up. On it: lined up.
    auto nt = nt::NetworkTableInstance::GetDefault().GetTable("AdvantageKit/RealOutputs/Auto");
    h.C().GetDrivetrain().ResetPose(frc::Pose2d{8_m, 4_m, frc::Rotation2d{}});
    h.Step(2);
    EXPECT_FALSE(nt->GetBoolean("AtStartPose", true));
    EXPECT_GT(nt->GetNumber("StartPoseErrorMeters", 0), 1.0);
    h.C().GetDrivetrain().ResetPose(first);
    h.Step(2);
    EXPECT_TRUE(nt->GetBoolean("AtStartPose", false));
    EXPECT_LT(nt->GetNumber("StartPoseErrorMeters", 1), RobotContainer::kAutoStartToleranceMeters);

    // Hidden once the match starts.
    h.Teleop();
    h.Step(2);
    EXPECT_FALSE(h.C().GetAutoStartPose().has_value());
    EXPECT_TRUE(h.C().GetField().GetObject("AutoPath")->GetPoses().empty());
  }
}

TEST(FeatureSimTest, ReadyToShootLightsUpWhileShooting) {
  RobotHarness h;
  // Midfield (outside the hub zone, so nothing revs until SHOOT), feeding a corner.
  h.C().GetDrivetrain().ResetPose(frc::Pose2d{8.0_m, 2.0_m, frc::Rotation2d{}});
  h.Step(5);
  h.Teleop();
  h.Step(5);
  auto& ss = h.C().GetSuperstructure();
  EXPECT_EQ(ss.GetState(), SS::IDLE);
  EXPECT_FALSE(ss.IsReadyToShoot()) << "flywheel is off";
  h.Driver().SetAButton(true);
  h.Step();
  h.Driver().SetAButton(false);
  EXPECT_TRUE(h.StepUntil([&] { return ss.IsReadyToShoot(); }, 150));
  EXPECT_EQ(ss.GetState(), SS::SHOOT);
}
