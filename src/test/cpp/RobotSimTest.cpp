// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

// Full-robot simulation tests: the real robot program with simulated hardware, driven through the
// simulated Driver Station and Xbox controller, in real time.

#include <cmath>
#include <cstdio>
#include <numbers>

#include <frc/geometry/Pose2d.h>
#include <gtest/gtest.h>

#include "RobotHarness.h"
#include "subsystems/linearslide/LinearSlideConstants.h"
#include "subsystems/shooter/ShooterConstants.h"
#include "subsystems/swerve/SwerveConstants.h"
#include "subsystems/vision/VisionConstants.h"

using ShooterMode = Superstructure::ShooterMode;
using IntakeMode = Superstructure::IntakeMode;
using XSim = frc::sim::XboxControllerSim;

namespace {
frc::Pose2d Pose(double x, double y, double deg = 0) {
  return frc::Pose2d{units::meter_t{x}, units::meter_t{y}, frc::Rotation2d{units::degree_t{deg}}};
}
auto A = [](XSim& c, bool v) { c.SetAButton(v); };
auto B = [](XSim& c, bool v) { c.SetBButton(v); };
auto X = [](XSim& c, bool v) { c.SetXButton(v); };
auto Y = [](XSim& c, bool v) { c.SetYButton(v); };
auto LeftTrigger = [](XSim& c, bool v) { c.SetLeftTriggerAxis(v ? 1.0 : 0.0); };
auto PovUp = [](XSim& c, bool v) { c.SetPOV(v ? 0 : -1); };
auto PovDown = [](XSim& c, bool v) { c.SetPOV(v ? 180 : -1); };
}  // namespace

TEST(RobotSimTest, BootsDisabledAndIdles) {
  RobotHarness h;
  h.Step(25);
  auto& ss = h.C().GetSuperstructure();
  EXPECT_EQ(ss.GetShooterMode(), ShooterMode::OFF);
  EXPECT_EQ(ss.GetIntakeMode(), IntakeMode::STOWED);
  // every mechanism's status signals are batched for one refresh per loop
  EXPECT_GE(phoenix_util::RegisteredSignalCount(), 5u * 10u);
  // nothing moves while disabled
  EXPECT_NEAR(h.C().GetShooter().Inputs().shooterMotorVoltages[0], 0.0, 1e-6);
  EXPECT_NEAR(h.C().GetIntakeRollers().Inputs().leftVoltage, 0.0, 1e-6);
}

TEST(RobotSimTest, DisabledButtonPressesDoNotChangeState) {
  RobotHarness h;
  h.Step(5);
  h.Tap(X);  // presses while disabled do nothing
  h.Step(2);
  EXPECT_EQ(h.C().GetSuperstructure().GetIntakeMode(), IntakeMode::STOWED);
}

TEST(RobotSimTest, IntakeExtendsSlideAfterHoodClearsIt) {
  RobotHarness h;
  h.C().GetDrivetrain().ResetPose(Pose(8, 2));  // mid-field: no hub targeting
  h.Teleop();
  h.Step(5);
  h.Tap(X);
  auto& ss = h.C().GetSuperstructure();
  EXPECT_EQ(ss.GetIntakeMode(), IntakeMode::INTAKING);
  // rollers commanded to 8 V (the simulated roller is heavy, so the 80 A stator limit holds the
  // output near 2 V while it spins up -- same physics as the Java simulation)
  EXPECT_EQ(h.C().GetIntakeRollers().GoalName(), "setVoltage");
  ASSERT_TRUE(h.StepUntil([&] { return h.C().GetIntakeRollers().Inputs().leftVoltage > 1.5; }, 25));
  // hood goes to 0.07 to clear the slide, then the slide extends to 0.86
  auto& slide = h.C().GetLinearSlide();
  auto& pivot = h.C().GetShooterPivot();
  ASSERT_TRUE(h.StepUntil(
      [&] { return std::abs(pivot.Inputs().shooterPivotMotorPosition - 0.07) < 0.01; }, 150))
      << "hood at " << pivot.Inputs().shooterPivotMotorPosition;
  ASSERT_TRUE(h.StepUntil([&] { return slide.Inputs().rightMotorPosition > 0.8; }, 200))
      << "slide at " << slide.Inputs().rightMotorPosition;
  EXPECT_NEAR(slide.RequestedPosition(), linear_slide::kIntakeFinalPosition, 1e-9);
  // once the slide is out of the hood's way the hood drops to 0.02
  ASSERT_TRUE(h.StepUntil([&] { return std::abs(pivot.RequestedPosition() - 0.02) < 1e-9; }, 50));
}

TEST(RobotSimTest, ShootSpinsUpThenFeeds) {
  RobotHarness h;
  // Close to the blue hub (x < 4): the hub is the target. Bring the robot there while disabled so
  // the hub trigger's rising edge (which only retargets while disabled) has already happened.
  h.C().GetDrivetrain().ResetPose(Pose(2.6, 4.0));
  h.Step(5);
  h.Teleop();
  h.Step(5);
  A(h.Driver(), true);  // shoots while A is held
  h.Step();
  auto& ss = h.C().GetSuperstructure();
  EXPECT_EQ(ss.GetShooterMode(), ShooterMode::SHOOTING);
  auto& shooter = h.C().GetShooter();
  const double expected = shooter::kHubLUT.Get(h.C().GetShotCalculator().GetDistance());
  ASSERT_TRUE(h.StepUntil([&] { return shooter.RequestedVelocity() > 1.0; }, 10));
  EXPECT_NEAR(shooter.RequestedVelocity(), expected, 2.0);
  ASSERT_TRUE(h.StepUntil([&] { return shooter.ReachedVelocity(); }, 250))
      << "flywheel " << shooter.Inputs().shooterMotorVelocitys[0] << " / "
      << shooter.RequestedVelocity();
  // indexer and feeder start (60 rot/s) once the flywheels are at speed
  ASSERT_TRUE(h.StepUntil([&] { return h.C().GetIndexer().GoalName() == "setVelocity"; }, 10));
  EXPECT_EQ(h.C().GetFeeder().GoalName(), "setVelocity");
  ASSERT_TRUE(h.StepUntil([&] { return h.C().GetIndexer().Inputs().leftVelocity > 8; }, 100));
  ASSERT_TRUE(h.StepUntil([&] { return h.C().GetFeeder().Inputs().leftVelocity > 8; }, 100));
  // the hood follows the hub lookup table (clamped by the sim's travel)
  EXPECT_GT(h.C().GetShooterPivot().RequestedPosition(), 0.03);
}

TEST(RobotSimTest, ShooterAndIntakeModesAreIndependent) {
  RobotHarness h;
  h.C().GetDrivetrain().ResetPose(Pose(8, 2));
  h.Teleop();
  h.Step(3);
  auto& ss = h.C().GetSuperstructure();
  A(h.Driver(), true);
  h.Step();
  EXPECT_EQ(ss.GetShooterMode(), ShooterMode::SHOOTING);
  h.Tap(X);  // intake on while still shooting
  EXPECT_EQ(ss.GetIntakeMode(), IntakeMode::INTAKING);
  EXPECT_EQ(ss.GetShooterMode(), ShooterMode::SHOOTING);
  A(h.Driver(), false);
  h.Step();
  EXPECT_EQ(ss.GetShooterMode(), ShooterMode::OFF);
  EXPECT_EQ(ss.GetIntakeMode(), IntakeMode::INTAKING);
  h.Tap(B);  // slide stays out, rollers off
  EXPECT_EQ(ss.GetIntakeMode(), IntakeMode::DEPLOYED);
  // (the hood first lifts clear while the slide moves through its travel)
  ASSERT_TRUE(h.StepUntil(
      [&] { return std::abs(h.C().GetShooterPivot().RequestedPosition() - 0.05) < 1e-9; }, 100));
  Y(h.Driver(), true);  // jitter while Y is held
  h.Step();
  EXPECT_EQ(ss.GetIntakeMode(), IntakeMode::JITTERING);
  // the slide jitters between 0.7 and 0.3745 every 0.2 s
  double minReq = 10;
  double maxReq = -10;
  for (int i = 0; i < 60; i++) {
    h.Step();
    minReq = std::min(minReq, h.C().GetLinearSlide().RequestedPosition());
    maxReq = std::max(maxReq, h.C().GetLinearSlide().RequestedPosition());
  }
  EXPECT_NEAR(minReq, linear_slide::kJitterIntermediate, 1e-9);
  EXPECT_NEAR(maxReq, linear_slide::kJitterPosition, 1e-9);
  Y(h.Driver(), false);
  h.Step();
  EXPECT_EQ(ss.GetIntakeMode(), IntakeMode::INTAKING);
  LeftTrigger(h.Driver(), true);  // outtake while the trigger is held
  h.Step();
  EXPECT_EQ(ss.GetIntakeMode(), IntakeMode::OUTTAKING);
  ASSERT_TRUE(
      h.StepUntil([&] { return h.C().GetIntakeRollers().Inputs().leftVoltage < -1.5; }, 25));
  ASSERT_TRUE(h.StepUntil([&] { return h.C().GetFeeder().Inputs().leftVelocity < -8; }, 100));
  LeftTrigger(h.Driver(), false);
  h.Step();
  EXPECT_EQ(ss.GetIntakeMode(), IntakeMode::STOWED);
}

TEST(RobotSimTest, EnteringHubZoneRevsAndLeavingIdles) {
  RobotHarness h;
  h.C().GetDrivetrain().ResetPose(Pose(8, 2));
  h.Teleop();
  h.Step(5);
  auto& ss = h.C().GetSuperstructure();
  EXPECT_EQ(ss.GetShooterMode(), ShooterMode::OFF);
  h.C().GetDrivetrain().ResetPose(Pose(3, 4));  // into the blue alliance zone
  h.Step(3);
  EXPECT_EQ(ss.GetShooterMode(), ShooterMode::REV);
  ASSERT_TRUE(h.StepUntil([&] { return h.C().GetShooter().RequestedVelocity() > 50; }, 20));
  h.C().GetDrivetrain().ResetPose(Pose(8, 4));  // back out
  h.Step(3);
  EXPECT_EQ(ss.GetShooterMode(), ShooterMode::OFF);
  // the corner feed target is chosen by which half of the field the robot is in
  EXPECT_NEAR(ss.GetTarget().X().value(), Superstructure::kBottomCorner.X().value(), 1e-9);
  EXPECT_NEAR(ss.GetTarget().Y().value(), Superstructure::kBottomCorner.Y().value(), 1e-9);
}

TEST(RobotSimTest, RedAllianceTargetsRedHubAndFlippedCorners) {
  RobotHarness h{true};
  h.C().GetDrivetrain().ResetPose(Pose(8, 6));
  h.Teleop();
  h.Step(5);
  auto& ss = h.C().GetSuperstructure();
  // top half, red: feed target is the flipped bottom corner
  EXPECT_NEAR(ss.GetTarget().X().value(), field_constants::fieldLength - 1.5, 1e-9);
  EXPECT_NEAR(ss.GetTarget().Y().value(), field_constants::fieldWidth - 1.5, 1e-9);
  h.C().GetDrivetrain().ResetPose(Pose(13, 4));  // red alliance zone (x > 12.5)
  h.Step(3);
  EXPECT_EQ(ss.GetShooterMode(), ShooterMode::REV);
  EXPECT_NEAR(ss.GetTarget().X().value(), field_constants::hub::kOppCenterX, 1e-9);
}

TEST(RobotSimTest, TrenchZoneForcesHoodDown) {
  RobotHarness h;
  h.C().GetDrivetrain().ResetPose(Pose(8, 2));
  h.Teleop();
  h.Step(3);
  h.Tap(B);  // slide out, rollers off: hood to 0.05 once the slide is out of its way
  ASSERT_TRUE(h.StepUntil([&] { return h.C().GetSuperstructure().IntakePhase() == "READY"; }, 150));
  h.Step(3);
  EXPECT_FALSE(h.C().GetShooterPivot().TrenchOverride());
  EXPECT_NEAR(h.C().GetShooterPivot().RequestedPosition(), 0.05, 1e-9);
  // under the alliance-side right trench
  h.C().GetDrivetrain().ResetPose(Pose(field_constants::lines_vertical::hubCenter, 0.6));
  h.Step(3);
  EXPECT_TRUE(h.C().GetShooterPivot().TrenchOverride());
  EXPECT_NEAR(h.C().GetShooterPivot().RequestedPosition(), 0.0, 1e-9);
}

TEST(RobotSimTest, ShootMultiplierAdjustsWithDpad) {
  RobotHarness h;
  h.Teleop();
  h.Step(2);
  h.Tap(PovUp);
  h.Tap(PovUp);
  h.Tap(PovDown);
  EXPECT_NEAR(h.C().GetSuperstructure().GetVelMultiplier(), 1.02, 1e-9);
}

TEST(RobotSimTest, DriverDrivesFieldCentric) {
  RobotHarness h;
  h.C().GetDrivetrain().ResetPose(Pose(8, 4));
  h.Teleop();
  h.Step(3);
  h.Driver().SetLeftY(-1.0);  // stick forward
  h.Step(75);
  const auto p1 = h.C().GetDrivetrain().State().Pose;
  EXPECT_GT(p1.X().value(), 8.0 + 3.0) << "full speed for 1.5 s";
  EXPECT_NEAR(p1.Y().value(), 4.0, 0.3);
  // slow mode
  h.Driver().SetLeftY(0.0);
  h.Step(40);
  const double x0 = h.C().GetDrivetrain().State().Pose.X().value();
  h.Driver().SetLeftBumperButton(true);
  h.Driver().SetLeftY(-1.0);
  h.Step(50);
  const double slowDistance = h.C().GetDrivetrain().State().Pose.X().value() - x0;
  EXPECT_GT(slowDistance, 0.5);
  EXPECT_LT(slowDistance, swerve_constants::SlowMaxSpeed * 1.0 * 1.15);
  h.Driver().SetLeftBumperButton(false);
  h.Driver().SetLeftY(0.0);
  h.Step(5);
}

TEST(RobotSimTest, HoldingAAimsAtTheHub) {
  RobotHarness h;
  h.C().GetDrivetrain().ResetPose(Pose(2.5, 6.0, 90));
  h.Step(3);
  h.Teleop();
  h.Step(3);
  h.Driver().SetAButton(true);
  auto& dt = h.C().GetDrivetrain();
  auto& sc = h.C().GetShotCalculator();
  ASSERT_TRUE(h.StepUntil(
      [&] { return ShotCalculator::IsAimed(dt.State().Pose.Rotation(), sc.GetDriveAngle(), 0.05); },
      150))
      << "heading " << dt.State().Pose.Rotation().Degrees().value() << " target "
      << sc.GetDriveAngle().Degrees().value();
  h.Driver().SetAButton(false);
  h.Step(2);
}

// Same as the command-based bindings: the most recent drive button wins, and releasing the button
// that set the mode goes back to normal driving even if another one is still held.
TEST(RobotSimTest, LastDriveButtonPressedWins) {
  using Mode = RobotContainer::DriveMode;
  RobotHarness h;
  h.C().GetDrivetrain().ResetPose(Pose(8, 2));
  h.Teleop();
  h.Step(2);
  EXPECT_EQ(h.C().GetDriveMode(), Mode::kNormal);
  h.Driver().SetAButton(true);
  h.Step();
  EXPECT_EQ(h.C().GetDriveMode(), Mode::kAim);
  h.Driver().SetLeftBumperButton(true);
  h.Step();
  EXPECT_EQ(h.C().GetDriveMode(), Mode::kSlow);
  h.Driver().SetAButton(false);  // not the button that set slow: no change
  h.Step();
  EXPECT_EQ(h.C().GetDriveMode(), Mode::kSlow);
  h.Driver().SetRightBumperButton(true);
  h.Step();
  EXPECT_EQ(h.C().GetDriveMode(), Mode::kSuperSlow);
  h.Driver().SetRightBumperButton(false);  // left bumper still held, but normal
  h.Step();
  EXPECT_EQ(h.C().GetDriveMode(), Mode::kNormal);
  h.Driver().SetLeftBumperButton(false);
  // a press while disabled does nothing, and holding it through enable is not a press
  h.Disable();
  h.Step();
  h.Driver().SetLeftBumperButton(true);
  h.Step();
  h.Teleop();
  h.Step(2);
  EXPECT_EQ(h.C().GetDriveMode(), Mode::kNormal);
  h.Driver().SetLeftBumperButton(false);
  h.Step();
}

TEST(RobotSimTest, VisionSimulationFeedsThePoseEstimator) {
  RobotHarness h;
  // back camera facing the blue alliance wall tags
  h.C().GetDrivetrain().ResetPose(Pose(2.0, 4.0, 0));
  h.Step(20);
  EXPECT_GT(h.C().GetVision().AcceptedCount(), 0);
  EXPECT_FALSE(h.C().GetVision().Inputs(0).tagIds.empty());
  // poses stay put (the simulated camera agrees with odometry)
  EXPECT_NEAR(h.C().GetDrivetrain().State().Pose.X().value(), 2.0, 0.05);
}

TEST(RobotSimTest, LoopTimeIsWellUnderBudget) {
  RobotHarness h;
  h.C().GetDrivetrain().ResetPose(Pose(2.6, 4.0));
  h.Step(5);
  h.Teleop();
  h.Step(10);
  h.ClearLoopStats();
  h.Tap(A);
  h.Driver().SetLeftY(-0.5);
  h.Driver().SetLeftX(0.3);
  h.Step(150);
  h.Driver().SetLeftY(0);
  h.Driver().SetLeftX(0);
  const double mean = h.MeanLoopMicros();
  const double p99 = h.PercentileLoopMicros(0.99);
  std::printf("[ LOOP TIME ] mean %.0f us, p99 %.0f us (desktop sim, includes simulated physics)\n",
              mean, p99);
  EXPECT_LT(mean, 5000.0);
  EXPECT_LT(p99, 15000.0);
}
