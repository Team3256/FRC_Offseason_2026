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
  Hooks h;
  for (const auto& key : {"stopAll", "shootHeld", "autoShooting", "outtakeHeld", "inAllianceZone"})
    h.inputs[key] = false;
  h.inputs["enabled"] = true;
  mayhem::StateMachine machine(mayhem_generated::ShooterMode::machine,
                               [&](std::string_view key) { return h.Signal(key); });
  ASSERT_TRUE(machine.Valid());
  auto set = [&](const char* key, bool v) {
    h.inputs[key] = v;
    machine.Tick(.02);
    machine.Tick(.02);  // settle
    return machine.State();
  };
  EXPECT_EQ(set("shootHeld", true), "SHOOTING");
  EXPECT_EQ(set("inAllianceZone", true), "SHOOTING");  // the zone never interrupts a shot
  EXPECT_EQ(set("shootHeld", false), "REV");
  EXPECT_EQ(set("autoShooting", true), "SHOOTING");
  EXPECT_EQ(set("outtakeHeld", true), "REV");  // outtaking stops the shot
  EXPECT_EQ(set("outtakeHeld", false), "SHOOTING");
  EXPECT_EQ(set("autoShooting", false), "REV");
  EXPECT_EQ(set("inAllianceZone", false), "OFF");
  EXPECT_EQ(set("inAllianceZone", true), "REV");
  EXPECT_EQ(set("enabled", false), "OFF");  // no rev while disabled
}

TEST(MayhemSuiteTest, IntakeModeTogglesAndHolds) {
  Hooks h;
  for (const auto& key : {"stopAll", "intakePressed", "autoIntake", "autoStow", "jitterHeld",
                          "deployPressed", "outtakeHeld"})
    h.inputs[key] = false;
  h.inputs["enabled"] = true;
  mayhem::StateMachine machine(mayhem_generated::IntakeMode::machine,
                               [&](std::string_view key) { return h.Signal(key); });
  ASSERT_TRUE(machine.Valid());
  // A press is an input for one tick; a hold stays until released.
  auto press = [&](const char* key) {
    h.inputs[key] = true;
    machine.Tick(.02);
    h.inputs[key] = false;
    machine.Tick(.02);
    return machine.State();
  };
  auto hold = [&](const char* key, bool v) {
    h.inputs[key] = v;
    machine.Tick(.02);
    return machine.State();
  };
  EXPECT_EQ(press("intakePressed"), "INTAKING");
  EXPECT_EQ(press("intakePressed"), "STOWED");
  EXPECT_EQ(press("autoIntake"), "INTAKING");
  EXPECT_EQ(hold("jitterHeld", true), "JITTERING");
  EXPECT_EQ(hold("jitterHeld", false), "INTAKING");
  EXPECT_EQ(press("deployPressed"), "DEPLOYED");
  EXPECT_EQ(press("deployPressed"), "STOWED");
  EXPECT_EQ(hold("outtakeHeld", true), "OUTTAKING");
  EXPECT_EQ(press("intakePressed"), "OUTTAKING");  // outtaking wins while held
  EXPECT_EQ(hold("outtakeHeld", false), "STOWED");
  EXPECT_EQ(press("intakePressed"), "INTAKING");
  EXPECT_EQ(press("autoStow"), "STOWED");
  EXPECT_EQ(press("intakePressed"), "INTAKING");
  EXPECT_EQ(hold("enabled", false), "STOWED");
}

TEST(MayhemSuiteTest, IntakeGraphClearsTheHoodBeforeTheSlideMoves) {
  Hooks h;
  h.inputs = {{"enabled", true},
              {"deploy", true},
              {"jitterDemand", false},
              {"hoodClear", false},
              {"slideDeployed", false}};
  mayhem::StateMachine machine(mayhem_generated::IntakeSequence::machine,
                               [&](std::string_view key) { return h.Signal(key); });
  auto tick = [&] {
    machine.Tick(.02);
    h.state = machine.State();
  };
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
  mayhem::StateMachine machine(mayhem_generated::ShotCycle::machine,
                               [&](std::string_view key) { return h.Signal(key); });
  auto tick = [&] {
    machine.Tick(.02);
    h.state = machine.State();
  };
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
      EXPECT_NEAR(poses.front().X().value(), suite_autos::Resolve(def.startPose, red).x, 1e-9);
      const mayhem::AutoNode* last = nullptr;
      for (std::size_t i = 0; i < def.routine.nodeCount; ++i)
        if (def.routine.nodes[i].kind == mayhem::NodeKind::Pose)
          last = &def.routine.nodes[i];
      ASSERT_NE(last, nullptr);
      EXPECT_NEAR(poses.back().X().value(), suite_autos::Resolve(last->goal.pose, red).x, 1e-9);
      EXPECT_NEAR(poses.back().Y().value(), suite_autos::Resolve(last->goal.pose, red).y, 1e-9);
    }
}
