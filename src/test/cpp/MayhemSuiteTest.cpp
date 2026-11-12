#include <gtest/gtest.h>
#include <map>
#include "commands/MayhemAutos.h"

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
TEST(MayhemSuiteTest, SupervisorPrioritizesIntentAndPreservesShootingAcrossZones) {
  Hooks h;
  for (const auto& key :
       {"enabled", "poseValid", "cancel", "home", "outtake", "shoot", "intake", "jitter", "rev"})
    h.inputs[key] = false;
  h.inputs["enabled"] = true;
  h.inputs["poseValid"] = true;
  mayhem::AutoRunner machine(robot_mayhem::Superstructure::routine, h);
  ASSERT_TRUE(machine.Start());
  auto tick = [&] { machine.Tick({}, {}, .02); };
  h.inputs["intake"] = true;
  tick();
  EXPECT_EQ(h.state, "INTAKE");
  h.inputs["shoot"] = true;
  tick();
  EXPECT_EQ(h.state, "SHOOT_AND_INTAKE");
  h.inputs["rev"] = true;
  tick();
  EXPECT_EQ(h.state, "SHOOT_AND_INTAKE");
  h.inputs["outtake"] = true;
  tick();
  EXPECT_EQ(h.state, "OUTTAKE");
  h.inputs["cancel"] = true;
  tick();
  EXPECT_EQ(h.state, "CANCEL_ALL");
  h.inputs["enabled"] = false;
  tick();
  EXPECT_EQ(h.state, "IDLE");
  h.inputs["enabled"] = true;
  h.inputs["cancel"] = false;
  h.inputs["poseValid"] = false;
  tick();
  EXPECT_EQ(h.state, "CANCEL_ALL");
}
TEST(MayhemSuiteTest, IntakeGraphClearsTheHoodBeforeTheSlideMoves) {
  Hooks h;
  h.inputs = {{"enabled", true},
              {"deploy", true},
              {"jitterDemand", false},
              {"hoodClear", false},
              {"slideDeployed", false}};
  mayhem::AutoRunner machine(robot_mayhem::IntakeSequence::routine, h);
  machine.Start();
  auto tick = [&] { machine.Tick({}, {}, .02); };
  tick();
  EXPECT_EQ(h.state, "CLEARING_HOOD");
  tick();
  EXPECT_EQ(h.state, "CLEARING_HOOD") << "the slide waits for the hood";
  h.inputs["hoodClear"] = true;
  tick();
  EXPECT_EQ(h.state, "DEPLOYING");
  h.inputs["slideDeployed"] = true;
  tick();
  EXPECT_EQ(h.state, "READY");
  h.inputs["jitterDemand"] = true;
  tick();
  EXPECT_EQ(h.state, "JITTERING");
  h.inputs["deploy"] = false;
  tick();
  EXPECT_EQ(h.state, "PARKED");
  // Jitter from parked starts as soon as the hood is clear; it does not deploy first.
  h.inputs["deploy"] = true;
  h.inputs["hoodClear"] = false;
  tick();
  EXPECT_EQ(h.state, "CLEARING_HOOD");
  h.inputs["hoodClear"] = true;
  tick();
  EXPECT_EQ(h.state, "JITTERING");
}
TEST(MayhemSuiteTest, ShotGraphFeedsOnceAtSpeedAndPausesUnderTheTrench) {
  Hooks h;
  h.inputs = {{"enabled", true}, {"shotDemand", true}, {"flywheelReady", false}, {"trench", false}};
  mayhem::AutoRunner machine(robot_mayhem::ShotCycle::routine, h);
  machine.Start();
  auto tick = [&] { machine.Tick({}, {}, .02); };
  tick();
  EXPECT_EQ(h.state, "SPINNING_UP");
  h.inputs["flywheelReady"] = true;
  tick();
  EXPECT_EQ(h.state, "FEEDING");
  // the speed dip as a piece goes through does not stop the feed
  h.inputs["flywheelReady"] = false;
  tick();
  EXPECT_EQ(h.state, "FEEDING");
  h.inputs["trench"] = true;
  tick();
  EXPECT_EQ(h.state, "INHIBITED");
  h.inputs["trench"] = false;
  tick();
  EXPECT_EQ(h.state, "SPINNING_UP");
  h.inputs["shotDemand"] = false;
  tick();
  EXPECT_EQ(h.state, "OFF");
}
TEST(MayhemSuiteTest, AllFiveGraphsReachTheirFinalNodeWithPoseFeedback) {
  for (const auto& def : robot_mayhem::autos) {
    SCOPED_TRACE(std::string{def.name});
    Hooks hooks;
    mayhem::AutoRunner runner(def.routine, hooks);
    mayhem::Pose pose = def.start;
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
  const auto& def = robot_mayhem::autos[0];
  mayhem::AutoRunner runner(def.routine, hooks);
  runner.Start();
  for (int i = 0; i < 2000 && runner.Status() == mayhem::RunStatus::Running; ++i)
    runner.Tick(def.start, {}, .02);
  EXPECT_EQ(runner.Status(), mayhem::RunStatus::Failed);
  EXPECT_EQ(runner.Tick(def.start, {}, .02).vx, 0);
  runner.Start();
  runner.Tick(def.start, {}, .02);
  runner.Cancel();
  EXPECT_EQ(runner.Status(), mayhem::RunStatus::Cancelled);
  EXPECT_EQ(runner.Tick(def.start, {}, .02).vx, 0);
}
TEST(MayhemSuiteTest, PreviewAndAllianceTransformsUseExecutableGraph) {
  for (const auto& def : robot_mayhem::autos)
    for (bool red : {false, true}) {
      const auto poses = suite_autos::Preview(def, red);
      ASSERT_GT(poses.size(), 2u);
      EXPECT_NEAR(poses.front().X().value(), suite_autos::Resolve(def.start, red).x, 1e-9);
      const mayhem::AutoNode* last = nullptr;
      for (std::size_t i = 0; i < def.routine.nodeCount; ++i)
        if (def.routine.nodes[i].kind == mayhem::NodeKind::Pose)
          last = &def.routine.nodes[i];
      ASSERT_NE(last, nullptr);
      EXPECT_NEAR(poses.back().X().value(), suite_autos::Resolve(last->goal.pose, red).x, 1e-9);
      EXPECT_NEAR(poses.back().Y().value(), suite_autos::Resolve(last->goal.pose, red).y, 1e-9);
    }
}
