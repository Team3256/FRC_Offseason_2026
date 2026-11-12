// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

// The superstructure state machine with ideal (fake) mechanisms and stepped simulated time: every
// trigger and transition of Superstructure::ConfigStateTransitions, checked against the behaviour
// of the Java version.

#include <memory>
#include <string>

#include <frc/geometry/Pose2d.h>
#include <frc/simulation/DriverStationSim.h>
#include <frc/simulation/SimHooks.h>
#include <frc2/command/CommandScheduler.h>
#include <gtest/gtest.h>

#include "FieldConstants.h"
#include "subsystems/Superstructure.h"
#include "subsystems/feeder/Feeder.h"
#include "subsystems/indexer/Indexer.h"
#include "subsystems/intakerollers/IntakeRollers.h"
#include "subsystems/linearslide/LinearSlide.h"
#include "subsystems/shooter/Shooter.h"
#include "subsystems/shooter/ShooterConstants.h"
#include "subsystems/shooterpivot/ShooterPivot.h"
#include "subsystems/shooterpivot/ShooterPivotConstants.h"
#include "subsystems/sotm/ShotCalculator.h"

using SS = Superstructure::StructureState;

namespace {

/** Flywheel that reaches whatever speed it is asked for. */
class FakeShooterIO : public ShooterIO {
 public:
  void UpdateInputs(ShooterIOInputs& in) override {
    in.shooterMotorVelocitys.fill(velocity);
    in.shooterMotorVoltages.fill(voltage);
  }
  void SetShooterVoltage(double v) override {
    voltage = v;
    mode = "voltage";
  }
  void SetShooterVelocity(double v) override {
    velocity = v;
    mode = "velocity";
  }
  void Off() override {
    velocity = 0;
    voltage = 0;
    mode = "off";
  }
  double velocity = 0, voltage = 0;
  std::string mode = "none";
};

/** Hood that reaches its setpoint immediately. */
class FakePivotIO : public ShooterPivotIO {
 public:
  void UpdateInputs(ShooterPivotIOInputs& in) override { in.shooterPivotMotorPosition = position; }
  void SetPosition(double p, double) override { position = p; }
  void SetPosition(double p) override { position = p; }
  double position = 0;
};

/** Slide that moves at a fixed rate toward its setpoint. */
class FakeSlideIO : public LinearSlideIO {
 public:
  void UpdateInputs(LinearSlideIOInputs& in) override {
    if (target >= 0) {
      double step = std::clamp(target - position, -0.05, 0.05);
      position += step;
    }
    in.rightMotorPosition = position;
    in.leftMotorPosition = position;
  }
  void SetPosition(double p) override {
    target = p;
    extendedSlot = false;
  }
  void SetExtendedPosition(double p) override {
    target = p;
    extendedSlot = true;
  }
  void Off() override { target = -1; }
  double position = 0.053;
  double target = -1;
  bool extendedSlot = false;
};

/** Rollers that remember what they were last told. */
class FakeRollerIO : public RollerPairIO {
 public:
  void UpdateInputs(RollerPairInputs& in) override {
    in.leftVoltage = voltage;
    in.leftVelocity = velocity;
  }
  void SetVoltage(double v) override {
    voltage = v;
    velocity = 0;
  }
  void SetVelocity(double v) override {
    velocity = v;
    voltage = 0;
  }
  void Off() override {
    voltage = 0;
    velocity = 0;
  }
  double voltage = 0, velocity = 0;
};

class SuperstructureLogicTest : public ::testing::Test {
 protected:
  void SetUp() override {
    frc::sim::PauseTiming();
    frc::sim::DriverStationSim::ResetData();
    frc::sim::DriverStationSim::SetDsAttached(true);
    frc::sim::DriverStationSim::SetAllianceStationId(HAL_AllianceStationID_kBlue1);
    SetEnabled(true);

    auto shooterIO = std::make_unique<FakeShooterIO>();
    auto pivotIO = std::make_unique<FakePivotIO>();
    auto slideIO = std::make_unique<FakeSlideIO>();
    auto rollers = std::make_unique<FakeRollerIO>();
    auto feeder = std::make_unique<FakeRollerIO>();
    auto indexer = std::make_unique<FakeRollerIO>();
    m_shooterIO = shooterIO.get();
    m_pivotIO = pivotIO.get();
    m_slideIO = slideIO.get();
    m_rollerIO = rollers.get();
    m_feederIO = feeder.get();
    m_indexerIO = indexer.get();
    m_intakeRollers = std::make_unique<IntakeRollers>(true, std::move(rollers));
    m_shooter = std::make_unique<Shooter>(true, std::move(shooterIO));
    m_pivot = std::make_unique<ShooterPivot>(true, std::move(pivotIO));
    m_slide = std::make_unique<LinearSlide>(true, std::move(slideIO));
    m_feeder = std::make_unique<Feeder>(true, std::move(feeder));
    m_indexer = std::make_unique<Indexer>(true, std::move(indexer));
    m_shot = std::make_unique<ShotCalculator>(
        [this] { return m_pose; }, [] { return frc::ChassisSpeeds{}; }, frc::Transform2d{});
    m_ss =
        std::make_unique<Superstructure>(*m_indexer, *m_pivot, *m_shooter, *m_intakeRollers,
                                         *m_slide, *m_feeder, *m_shot, [this] { return m_pose; });
  }

  void TearDown() override {
    auto& s = frc2::CommandScheduler::GetInstance();
    s.CancelAll();
    s.GetDefaultButtonLoop()->Clear();
    s.UnregisterAllSubsystems();
    m_ss.reset();
    m_shot.reset();
    m_indexer.reset();
    m_feeder.reset();
    m_slide.reset();
    m_pivot.reset();
    m_shooter.reset();
    m_intakeRollers.reset();
    frc::sim::ResumeTiming();
  }

  void SetEnabled(bool enabled) {
    frc::sim::DriverStationSim::SetEnabled(enabled);
    frc::sim::DriverStationSim::NotifyNewData();
    frc::DriverStation::RefreshData();
  }

  /** One robot loop: scheduler, then the superstructure and shot calculator (as RobotContainer). */
  /** Runs until the intake phase graph reaches {@code phase} (the hood clears the slide first). */
  void LoopUntilIntakePhase(std::string_view phase, int max = 150) {
    for (int i = 0; i < max && m_ss->IntakePhase() != phase; ++i) {
      Loop();
    }
    ASSERT_EQ(m_ss->IntakePhase(), phase);
  }

  void Loop(int n = 1) {
    for (int i = 0; i < n; i++) {
      frc::sim::StepTiming(20_ms);
      frc::DriverStation::RefreshData();
      frc2::CommandScheduler::GetInstance().Run();
      m_ss->Periodic();
      m_shot->Periodic();
    }
  }

  void Set(SS state) {
    frc2::CommandScheduler::GetInstance().Schedule(
        m_holder.emplace_back(m_ss->SetState(state)).get());
    Loop();
  }

  std::string CmdName(frc2::Subsystem& s) {
    auto* c = s.GetCurrentCommand();
    return c ? c->GetName() : "none";
  }

  frc::Pose2d m_pose{8_m, 2_m, frc::Rotation2d{}};
  std::vector<frc2::CommandPtr> m_holder;
  FakeShooterIO* m_shooterIO;
  FakePivotIO* m_pivotIO;
  FakeSlideIO* m_slideIO;
  FakeRollerIO* m_rollerIO;
  FakeRollerIO* m_feederIO;
  FakeRollerIO* m_indexerIO;
  std::unique_ptr<IntakeRollers> m_intakeRollers;
  std::unique_ptr<Shooter> m_shooter;
  std::unique_ptr<ShooterPivot> m_pivot;
  std::unique_ptr<LinearSlide> m_slide;
  std::unique_ptr<Feeder> m_feeder;
  std::unique_ptr<Indexer> m_indexer;
  std::unique_ptr<ShotCalculator> m_shot;
  std::unique_ptr<Superstructure> m_ss;
};

}  // namespace

TEST_F(SuperstructureLogicTest, StartsIdleAndSelectsTheCornerImmediately) {
  Loop(3);
  EXPECT_EQ(m_ss->GetState(), SS::IDLE);
  // The target is chosen from the pose every loop, so a robot that starts outside the alliance
  // zone already aims at its feed corner (the trigger version kept the hub until it moved).
  EXPECT_NEAR(m_ss->GetTarget().X().value(), Superstructure::kBottomCorner.X().value(), 1e-12);
}

TEST_F(SuperstructureLogicTest, SetStateTracksPreviousState) {
  Set(SS::HOME);
  EXPECT_EQ(m_ss->GetState(), SS::HOME);
  EXPECT_EQ(m_ss->GetPrevState(), SS::IDLE);
  Set(SS::HOME);  // same state again: previous is kept
  EXPECT_EQ(m_ss->GetPrevState(), SS::IDLE);
  Set(SS::OUTTAKE);
  EXPECT_EQ(m_ss->GetPrevState(), SS::HOME);
}

TEST_F(SuperstructureLogicTest, SetStateDoesNotRunWhileDisabled) {
  SetEnabled(false);
  Set(SS::INTAKE);
  EXPECT_EQ(m_ss->GetState(), SS::IDLE);
}

TEST_F(SuperstructureLogicTest, IntakeSequence) {
  Set(SS::INTAKE);
  Loop();
  EXPECT_EQ(CmdName(*m_intakeRollers), "setVoltage");
  EXPECT_DOUBLE_EQ(m_rollerIO->voltage, 8.0);
  // slide inside the hood's zone: hood up to clear it
  EXPECT_DOUBLE_EQ(m_pivot->RequestedPosition(), Superstructure::kIntakeHoodPosition);
  // the (instant) hood has reached it, so the slide goes out to the intake position
  Loop(40);
  EXPECT_NEAR(m_slideIO->position, 0.86, 1e-9);
  // and once the slide is out of the zone the hood drops to 0.02
  EXPECT_DOUBLE_EQ(m_pivot->RequestedPosition(), 0.02);
  // at 0.86 the slide counts as extended: further position requests use slot 1
  EXPECT_TRUE(m_slideIO->extendedSlot);
}

TEST_F(SuperstructureLogicTest, IntakeThenShootBecomesShootAndIntake) {
  Set(SS::INTAKE);
  Set(SS::SHOOT);
  Loop();
  EXPECT_EQ(m_ss->GetState(), SS::SHOOT_AND_INTAKE);
  // the intent graph goes straight there instead of passing through SHOOT for one loop
  EXPECT_EQ(m_ss->GetPrevState(), SS::INTAKE);
}

TEST_F(SuperstructureLogicTest, ShootThenIntakeBecomesShootAndIntake) {
  Set(SS::SHOOT);
  Set(SS::INTAKE);
  Loop();
  EXPECT_EQ(m_ss->GetState(), SS::SHOOT_AND_INTAKE);
}

TEST_F(SuperstructureLogicTest, ShootingFeedsOnceTheFlywheelIsAtSpeed) {
  m_pose = frc::Pose2d{3_m, 4_m, frc::Rotation2d{}};  // in the blue zone -> hub targeting (REV)
  Loop(2);
  EXPECT_EQ(m_ss->GetState(), SS::REV);
  EXPECT_EQ(CmdName(*m_shooter), "setVelocity");
  EXPECT_EQ(CmdName(*m_feeder), "none");  // feeder.off() ran once
  Set(SS::SHOOT);
  Loop();
  // hub shot: velocity from the hub table times the multiplier, hood from the hub table
  const double dist = m_shot->GetDistance();
  EXPECT_NEAR(m_shooterIO->velocity, shooter::kHubLUT.Get(dist), 1e-9);
  EXPECT_NEAR(m_pivot->RequestedPosition(), shooter_pivot::kHubLUT.Get(dist), 1e-9);
  // flywheel at speed (debounced 20 ms) -> indexer and feeder at 60 rot/s
  Loop(3);
  EXPECT_DOUBLE_EQ(m_indexerIO->velocity, 60);
  EXPECT_DOUBLE_EQ(m_feederIO->velocity, 60);
}

TEST_F(SuperstructureLogicTest, ShootMultiplierScalesHubShots) {
  m_pose = frc::Pose2d{3_m, 4_m, frc::Rotation2d{}};
  Loop(2);
  frc2::CommandScheduler::GetInstance().Schedule(
      m_holder.emplace_back(m_ss->AddShootMultiplier(0.02)).get());
  Loop();
  Set(SS::SHOOT);
  Loop();
  EXPECT_NEAR(m_shooterIO->velocity, shooter::kHubLUT.Get(m_shot->GetDistance()) * 1.02, 1e-9);
}

TEST_F(SuperstructureLogicTest, FeedShotsUseTheFeedTables) {
  // mid-field, bottom half; enter the top half to retarget to the top corner
  m_pose = frc::Pose2d{8_m, 6_m, frc::Rotation2d{}};
  Loop(2);
  EXPECT_NEAR(m_ss->GetTarget().Y().value(), Superstructure::kTopCorner.Y().value(), 1e-12);
  Set(SS::SHOOT);
  Loop();
  const double dist = m_shot->GetDistance();
  EXPECT_NEAR(m_shooterIO->velocity, shooter::kFeedLUT.Get(dist), 1e-9);
  EXPECT_NEAR(m_pivot->RequestedPosition(), shooter_pivot::kFeedLUT.Get(dist), 1e-9);
}

TEST_F(SuperstructureLogicTest, HubZoneRevAndIntakeCombine) {
  m_pose = frc::Pose2d{3_m, 4_m, frc::Rotation2d{}};
  Loop(2);
  EXPECT_EQ(m_ss->GetState(), SS::REV);
  Set(SS::INTAKE);
  Loop();
  EXPECT_EQ(m_ss->GetState(), SS::REV_AND_INTAKE);
  EXPECT_EQ(CmdName(*m_shooter), "setVelocity");      // still revving
  m_pose = frc::Pose2d{8_m, 4_m, frc::Rotation2d{}};  // leave the zone
  Loop(2);
  EXPECT_EQ(m_ss->GetState(), SS::INTAKE);
  EXPECT_EQ(CmdName(*m_shooter), "none");  // whileTrue rev ended
}

TEST_F(SuperstructureLogicTest, LeavingTheHubZoneWhileRevvingIdles) {
  m_pose = frc::Pose2d{3_m, 4_m, frc::Rotation2d{}};
  Loop(2);
  m_pose = frc::Pose2d{8_m, 4_m, frc::Rotation2d{}};
  Loop(2);
  EXPECT_EQ(m_ss->GetState(), SS::IDLE);
  EXPECT_DOUBLE_EQ(m_rollerIO->voltage, 0);
  EXPECT_EQ(m_shooterIO->mode, "off");
}

TEST_F(SuperstructureLogicTest, JitterAndShootCombinations) {
  Set(SS::SHOOT);
  Set(SS::JITTER);
  Loop();
  EXPECT_EQ(m_ss->GetState(), SS::JITTER_AND_SHOOT);
  EXPECT_EQ(CmdName(*m_intakeRollers), "setVoltage");
  EXPECT_DOUBLE_EQ(m_rollerIO->voltage, 8.0);
  LoopUntilIntakePhase("JITTERING");
  Loop(2);
  // jitter alternates the slide between 0.7 and 0.3745, 0.2 s each
  double lo = 10, hi = -10;
  for (int i = 0; i < 40; i++) {
    Loop();
    lo = std::min(lo, m_slide->RequestedPosition());
    hi = std::max(hi, m_slide->RequestedPosition());
  }
  EXPECT_DOUBLE_EQ(lo, 0.3745);
  EXPECT_DOUBLE_EQ(hi, 0.7);
  EXPECT_FALSE(m_slideIO->extendedSlot);  // jitter always uses slot 0
}

TEST_F(SuperstructureLogicTest, JitterThenShootBecomesJitterAndShoot) {
  Set(SS::JITTER);
  Set(SS::SHOOT);
  Loop();
  EXPECT_EQ(m_ss->GetState(), SS::JITTER_AND_SHOOT);
}

TEST_F(SuperstructureLogicTest, HomeOuttakeIdleAndCancelAll) {
  Set(SS::INTAKE);
  Loop(5);
  Set(SS::HOME);
  LoopUntilIntakePhase("READY");
  Loop();
  EXPECT_DOUBLE_EQ(m_pivot->RequestedPosition(), 0.05);
  EXPECT_DOUBLE_EQ(m_rollerIO->voltage, 0);
  EXPECT_NEAR(m_slide->RequestedPosition(), 0.86, 1e-12);  // ground intake position
  Set(SS::OUTTAKE);
  Loop();
  EXPECT_DOUBLE_EQ(m_rollerIO->voltage, -8.0);
  EXPECT_DOUBLE_EQ(m_feederIO->velocity, -80);
  Set(SS::CANCEL_ALL);
  Loop();
  EXPECT_DOUBLE_EQ(m_rollerIO->voltage, 0);
  EXPECT_DOUBLE_EQ(m_feederIO->velocity, 0);
  EXPECT_DOUBLE_EQ(m_pivot->RequestedPosition(), 0.05);
  Set(SS::IDLE);
  Loop();
  EXPECT_EQ(m_slideIO->target, -1);  // slide off
}

TEST_F(SuperstructureLogicTest, TrenchApproachForcesHoodDown) {
  Set(SS::HOME);
  LoopUntilIntakePhase("READY");
  Loop();
  EXPECT_DOUBLE_EQ(m_pivot->RequestedPosition(), 0.05);
  m_pose = frc::Pose2d{units::meter_t{field_constants::lines_vertical::oppHubCenter}, 7.5_m,
                       frc::Rotation2d{}};
  Loop(2);
  EXPECT_TRUE(m_pivot->TrenchOverride());
  EXPECT_DOUBLE_EQ(m_pivot->RequestedPosition(), 0.0);
  m_pose = frc::Pose2d{8_m, 7.5_m, frc::Rotation2d{}};
  Loop(2);
  EXPECT_FALSE(m_pivot->TrenchOverride());
  EXPECT_DOUBLE_EQ(m_pivot->RequestedPosition(), 0.05);
}

TEST_F(SuperstructureLogicTest, TrenchZoneGeometry) {
  using namespace field_constants;
  auto zone = [&](double x, double y) {
    m_pose = frc::Pose2d{units::meter_t{x}, units::meter_t{y}, frc::Rotation2d{}};
    return m_ss->IsInTrenchApproachZone();
  };
  const double margin = Inches(18.0) + left_trench::width / 2.0;
  EXPECT_TRUE(zone(lines_vertical::hubCenter, 0.5));
  EXPECT_TRUE(zone(lines_vertical::hubCenter + margin - 0.01, 0.5));
  EXPECT_FALSE(zone(lines_vertical::hubCenter + margin + 0.01, 0.5));
  EXPECT_TRUE(zone(lines_vertical::oppHubCenter, 7.6));
  EXPECT_FALSE(zone(lines_vertical::hubCenter, 4.0));  // the bump/hub, not a trench
  EXPECT_TRUE(zone(lines_vertical::hubCenter, lines_horizontal::rightTrenchOpenStart));
  EXPECT_TRUE(zone(lines_vertical::hubCenter, lines_horizontal::leftTrenchOpenEnd));
}

TEST_F(SuperstructureLogicTest, RedAllianceZonesAndTargets) {
  frc::sim::DriverStationSim::SetAllianceStationId(HAL_AllianceStationID_kRed2);
  frc::sim::DriverStationSim::NotifyNewData();
  Loop();
  EXPECT_FALSE(m_ss->TargetBlueHub());
  m_pose = frc::Pose2d{3_m, 4_m, frc::Rotation2d{}};
  Loop(2);
  EXPECT_EQ(m_ss->GetState(), SS::IDLE);  // the blue zone means nothing on red
  m_pose = frc::Pose2d{13_m, 4_m, frc::Rotation2d{}};
  Loop(2);
  EXPECT_EQ(m_ss->GetState(), SS::REV);
  EXPECT_NEAR(m_ss->GetTarget().X().value(), field_constants::hub::kOppCenterX, 1e-12);
  m_pose = frc::Pose2d{8_m, 2_m, frc::Rotation2d{}};  // bottom half on red: flipped top corner
  Loop(2);
  EXPECT_NEAR(m_ss->GetTarget().X().value(), field_constants::fieldLength - 1.5, 1e-12);
  EXPECT_NEAR(m_ss->GetTarget().Y().value(), field_constants::fieldWidth - 6.8, 1e-12);
}

TEST_F(SuperstructureLogicTest, TargetChangesEvenWhileDisabled) {
  SetEnabled(false);
  m_pose = frc::Pose2d{8_m, 6_m, frc::Rotation2d{}};  // top half: feed the top corner
  Loop(2);
  EXPECT_NEAR(m_ss->GetTarget().Y().value(), Superstructure::kTopCorner.Y().value(), 1e-12);
  m_pose = frc::Pose2d{3_m, 4_m, frc::Rotation2d{}};  // blue zone: the hub
  Loop(2);
  // ChangeTarget ignores disable, SetState(REV) does not
  EXPECT_NEAR(m_ss->GetTarget().X().value(), field_constants::hub::kCenterX, 1e-12);
  EXPECT_NEAR(m_shot->GetTarget().X().value(), field_constants::hub::kCenterX, 1e-12);
  EXPECT_EQ(m_ss->GetState(), SS::IDLE);
}
