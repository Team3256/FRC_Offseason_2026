#include <gtest/gtest.h>
#include <map>
#include "commands/MayhemAutos.h"
#include "mayhem/state_machine.hpp"

namespace {
class Hooks : public mayhem::RobotHooks {
 public:
  mayhem::SignalValue Signal(std::string_view key) const override {
    auto it = inputs.find(std::string{key});
    return it == inputs.end() ? mayhem::SignalValue{} : it->second;
  }
  mayhem::ActionStatus TickAction(std::string_view) override {
    return mayhem::ActionStatus::Succeeded;
  }
  void StateEntered(std::string_view s) override { state = s; }
  void Transitioned(std::string_view, std::string_view, std::string_view) override {
    ++transitions;
  }
  std::map<std::string, mayhem::SignalValue> inputs;
  std::string_view state;
  int transitions = 0;
};
}  // namespace
TEST(MayhemSuiteTest, ShooterModeShootsWhileHeldAndRevsInTheZone) {
  using S = mayhem_generated::ShooterMode::State;
  using In = mayhem_generated::ShooterMode::Inputs;
  mayhem_generated::ShooterMode::Machine machine;
  ASSERT_TRUE(machine.Valid());
  machine.In().enabled = true;
  // One robot loop: the machine settles, so A released in the zone is SHOOTING -> OFF -> REV.
  auto set = [&](bool In::* input, bool v) {
    machine.In().*input = v;
    machine.Settle(.02);
    EXPECT_TRUE(machine.Settled());
    return machine.Current();
  };
  EXPECT_EQ(set(&In::shootHeld, true), S::SHOOTING);
  EXPECT_EQ(set(&In::inAllianceZone, true), S::SHOOTING);  // the zone never interrupts a shot
  EXPECT_EQ(set(&In::shootHeld, false), S::REV);
  EXPECT_EQ(set(&In::autoShooting, true), S::SHOOTING);
  EXPECT_EQ(set(&In::outtakeHeld, true), S::REV);  // outtaking stops the shot
  EXPECT_EQ(set(&In::outtakeHeld, false), S::SHOOTING);
  EXPECT_EQ(set(&In::autoShooting, false), S::REV);
  EXPECT_EQ(set(&In::inAllianceZone, false), S::OFF);
  EXPECT_EQ(set(&In::inAllianceZone, true), S::REV);
  EXPECT_EQ(set(&In::stopAll, true), S::REV) << "stop all, then back to REV in the zone";
  EXPECT_FALSE(machine.In().stopAll) << "an event lasts one loop";
  EXPECT_EQ(set(&In::enabled, false), S::OFF);  // no rev while disabled
}

TEST(MayhemSuiteTest, IntakeModeTogglesAndHolds) {
  using S = mayhem_generated::IntakeMode::State;
  using In = mayhem_generated::IntakeMode::Inputs;
  mayhem_generated::IntakeMode::Machine machine;
  ASSERT_TRUE(machine.Valid());
  machine.In().enabled = true;
  // A press is an event: used by the first step of one loop, so X toggles once, not on and off.
  auto press = [&](bool In::* input) {
    machine.In().*input = true;
    machine.Settle(.02);
    EXPECT_FALSE(machine.In().*input);
    return machine.Current();
  };
  auto hold = [&](bool In::* input, bool v) {
    machine.In().*input = v;
    machine.Settle(.02);
    return machine.Current();
  };
  EXPECT_EQ(press(&In::intakePressed), S::INTAKING);
  EXPECT_EQ(press(&In::intakePressed), S::STOWED);
  EXPECT_EQ(press(&In::autoIntake), S::INTAKING);
  EXPECT_EQ(hold(&In::jitterHeld, true), S::JITTERING);
  EXPECT_EQ(hold(&In::jitterHeld, false), S::INTAKING);
  EXPECT_EQ(press(&In::deployPressed), S::DEPLOYED);
  EXPECT_EQ(press(&In::deployPressed), S::STOWED);
  EXPECT_EQ(hold(&In::outtakeHeld, true), S::OUTTAKING);
  EXPECT_EQ(press(&In::intakePressed), S::OUTTAKING);  // outtaking wins while held
  EXPECT_EQ(hold(&In::outtakeHeld, false), S::STOWED);
  EXPECT_EQ(press(&In::intakePressed), S::INTAKING);
  EXPECT_EQ(press(&In::autoStow), S::STOWED);
  EXPECT_EQ(press(&In::intakePressed), S::INTAKING);
  EXPECT_EQ(hold(&In::enabled, false), S::STOWED);
}

TEST(MayhemSuiteTest, IntakeGraphClearsTheHoodBeforeTheSlideMoves) {
  using S = mayhem_generated::IntakeSequence::State;
  mayhem_generated::IntakeSequence::Machine machine;
  auto& in = machine.In();
  in.enabled = true;
  in.deploy = true;
  auto tick = [&] {
    machine.Tick(.02);
    return machine.Current();
  };
  EXPECT_EQ(tick(), S::CLEARING_HOOD);
  EXPECT_EQ(tick(), S::CLEARING_HOOD) << "the slide waits for the hood";
  in.hoodClear = true;
  EXPECT_EQ(tick(), S::DEPLOYING);
  in.slideDeployed = true;
  EXPECT_EQ(tick(), S::READY);
  in.jitterDemand = true;
  EXPECT_EQ(tick(), S::JITTERING);
  in.deploy = false;
  EXPECT_EQ(tick(), S::PARKED);
  // Jitter from parked starts as soon as the hood is clear; it does not deploy first.
  in.deploy = true;
  in.hoodClear = false;
  EXPECT_EQ(tick(), S::CLEARING_HOOD);
  in.hoodClear = true;
  EXPECT_EQ(tick(), S::JITTERING);
}
TEST(MayhemSuiteTest, ShotGraphFeedsOnceAtSpeedAndPausesUnderTheTrench) {
  using S = mayhem_generated::ShotCycle::State;
  mayhem_generated::ShotCycle::Machine machine;
  auto& in = machine.In();
  in.enabled = true;
  in.shotDemand = true;
  auto tick = [&] {
    machine.Tick(.02);
    return machine.Current();
  };
  EXPECT_EQ(tick(), S::SPINNING_UP);
  in.flywheelReady = true;
  EXPECT_EQ(tick(), S::FEEDING);
  // the speed dip as a piece goes through does not stop the feed
  in.flywheelReady = false;
  EXPECT_EQ(tick(), S::FEEDING);
  in.trench = true;
  EXPECT_EQ(tick(), S::INHIBITED);
  in.trench = false;
  EXPECT_EQ(tick(), S::SPINNING_UP);
  in.shotDemand = false;
  EXPECT_EQ(tick(), S::OFF);
}
TEST(MayhemSuiteTest, AllFiveGraphsReachTheirFinalNodeWithPoseFeedback) {
  for (const auto& def : mayhem_generated::routines) {
    SCOPED_TRACE(std::string{def.name});
    Hooks hooks;
    mayhem::AutoRunner runner(def.routine, hooks);
    mayhem::Pose pose = def.startPose;
    mayhem::Velocity velocity{};
    ASSERT_TRUE(runner.Start());
    for (int i = 0; i < 6000 && runner.Status() == mayhem::RunStatus::Running; ++i) {
      velocity = runner.Tick(pose, velocity, .02);
      pose.x += velocity.vx * .02;
      pose.y += velocity.vy * .02;
      pose.heading = mayhem::Wrap(pose.heading + velocity.omega * .02);
    }
    EXPECT_EQ(runner.Status(), mayhem::RunStatus::Completed) << runner.ActiveState();
    EXPECT_EQ(runner.ActiveState(), "done");
  }
}
TEST(MayhemSuiteTest, FrozenDrivetrainFailsAndCancelStopsOutput) {
  Hooks hooks;
  const auto& def = mayhem_generated::routines[0];
  mayhem::AutoRunner runner(def.routine, hooks);
  runner.Start();
  for (int i = 0; i < 2000 && runner.Status() == mayhem::RunStatus::Running; ++i)
    runner.Tick(def.startPose, {}, .02);
  EXPECT_EQ(runner.Status(), mayhem::RunStatus::Failed);
  EXPECT_EQ(runner.Tick(def.startPose, {}, .02).vx, 0);
  runner.Start();
  runner.Tick(def.startPose, {}, .02);
  runner.Cancel();
  EXPECT_EQ(runner.Status(), mayhem::RunStatus::Cancelled);
  EXPECT_EQ(runner.Tick(def.startPose, {}, .02).vx, 0);
}
TEST(MayhemSuiteTest, PreviewAndAllianceTransformsUseExecutableGraph) {
  for (const auto& def : mayhem_generated::routines)
    for (bool red : {false, true}) {
      const auto poses = suite_autos::Preview(def, red);
      ASSERT_GT(poses.size(), 2u);
      // Mayhem stores the preview to the millimetre.
      constexpr double kTolerance = 1e-3;
      EXPECT_NEAR(poses.front().X().value(), suite_autos::Resolve(def.startPose, red).x,
                  kTolerance);
      const mayhem::AutoNode* last = nullptr;
      for (std::size_t i = 0; i < def.routine.nodeCount; ++i)
        if (def.routine.nodes[i].kind == mayhem::NodeKind::Pose)
          last = &def.routine.nodes[i];
      ASSERT_NE(last, nullptr);
      EXPECT_NEAR(poses.back().X().value(), suite_autos::Resolve(last->goal.pose, red).x,
                  kTolerance);
      EXPECT_NEAR(poses.back().Y().value(), suite_autos::Resolve(last->goal.pose, red).y,
                  kTolerance);
    }
}
