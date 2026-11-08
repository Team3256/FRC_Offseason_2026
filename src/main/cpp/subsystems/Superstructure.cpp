// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "subsystems/Superstructure.h"

#include <cmath>
#include <cstdio>
#include <string>

#include <frc/DriverStation.h>
#include <frc2/command/Commands.h>

#include "FieldConstants.h"
#include "logging/LoggedTracer.h"
#include "subsystems/feeder/Feeder.h"
#include "subsystems/indexer/Indexer.h"
#include "subsystems/intakerollers/IntakeRollers.h"
#include "subsystems/linearslide/LinearSlide.h"
#include "subsystems/shooter/Shooter.h"
#include "subsystems/shooterpivot/ShooterPivot.h"
#include "subsystems/sotm/ShotCalculator.h"
#include "utils/TriggerUtil.h"

using trigger_util::And;
using trigger_util::Not;
using trigger_util::Or;
using SS = Superstructure::StructureState;

std::string_view Superstructure::ToString(StructureState state) {
  switch (state) {
    case SS::INTAKE:
      return "INTAKE";
    case SS::SHOOT:
      return "SHOOT";
    case SS::IDLE:
      return "IDLE";
    case SS::CANCEL_ALL:
      return "CANCEL_ALL";
    case SS::HOME:
      return "HOME";
    case SS::REV:
      return "REV";
    case SS::SHOOT_AND_INTAKE:
      return "SHOOT_AND_INTAKE";
    case SS::REV_AND_INTAKE:
      return "REV_AND_INTAKE";
    case SS::JITTER:
      return "JITTER";
    case SS::JITTER_AND_SHOOT:
      return "JITTER_AND_SHOOT";
    case SS::OUTTAKE:
      return "OUTTAKE";
  }
  return "?";
}

namespace {
template <typename F, size_t... I>
std::array<frc2::Trigger, sizeof...(I)> MakeTriggers(F&& make, std::index_sequence<I...>) {
  return {make(static_cast<SS>(I))...};
}
}  // namespace

Superstructure::Superstructure(Indexer& indexer, ShooterPivot& shooterPivot, Shooter& shooter,
                               IntakeRollers& intakeRollers, LinearSlide& linearSlide,
                               Feeder& feeder, ShotCalculator& shotCalculator,
                               std::function<frc::Pose2d()> robotPoseSupplier)
    : m_indexer{indexer},
      m_shooterPivot{shooterPivot},
      m_shooter{shooter},
      m_intakeRollers{intakeRollers},
      m_linearSlide{linearSlide},
      m_feeder{feeder},
      m_shotCalculator{shotCalculator},
      m_robotPoseSupplier{std::move(robotPoseSupplier)},
      m_stateTriggers{
          MakeTriggers([this](SS s) { return frc2::Trigger([this, s] { return m_state == s; }); },
                       std::make_index_sequence<kNumStates>{})},
      m_prevStateTriggers{MakeTriggers(
          [this](SS s) { return frc2::Trigger([this, s] { return m_prevState == s; }); },
          std::make_index_sequence<kNumStates>{})},
      m_targetBlueHub{[this] { return TargetBlueHub(); }},
      m_targetRedHub{[this] { return TargetRedHub(); }},
      m_feedTopCorner{
          And(Not(Or(m_targetBlueHub, m_targetRedHub)), [this] { return IsRobotTopHalf(); })},
      m_feedBottomCorner{Not(Or(Or(m_targetBlueHub, m_targetRedHub), m_feedTopCorner))},
      m_target{field_constants::hub::kTarget2d, frc::Rotation2d{}} {
  m_stateTimer.Start();
  ConfigStateTransitions();
}

void Superstructure::ConfigStateTransitions() {
  auto distance = [this] { return m_shotCalculator.GetDistance(); };
  auto velMultiplier = [this] { return m_velMultiplier; };
  auto isEnabled = [] { return frc::DriverStation::IsEnabled(); };
  const auto anyHub = [this] { return Or(m_targetBlueHub, m_targetRedHub); };
  const auto shooting = [this] {
    return Or(Or(S(SS::SHOOT), S(SS::SHOOT_AND_INTAKE)), S(SS::JITTER_AND_SHOOT));
  };

  m_targetBlueHub.OnTrue(ChangeTarget(field_constants::hub::kTarget2d));

  m_targetRedHub.OnTrue(ChangeTarget(field_constants::hub::kOppTarget2d));

  anyHub().OnTrue(SetState(SS::REV));

  And(Not(anyHub()), S(SS::REV)).OnTrue(SetState(SS::IDLE));
  And(Not(anyHub()), S(SS::REV_AND_INTAKE)).OnTrue(SetState(SS::INTAKE));

  m_feedTopCorner.OnTrue(ChangeTarget([this] {
    return GetAllianceBlue() ? kTopCorner : field_constants::FlipRotational(kBottomCorner);
  }));
  m_feedBottomCorner.OnTrue(ChangeTarget([this] {
    return GetAllianceBlue() ? kBottomCorner : field_constants::FlipRotational(kTopCorner);
  }));

  And(And(Or(m_targetRedHub, m_targetBlueHub), isEnabled), shooting())
      .OnTrue(m_shooter.ShootHub(distance, velMultiplier))
      .WhileTrue(m_shooterPivot.ShootHub(distance));

  And(And(Or(m_feedTopCorner, m_feedBottomCorner), isEnabled), shooting())
      .OnTrue(m_shooter.FeedCorner(distance))
      .WhileTrue(m_shooterPivot.FeedCorner(distance));

  And(shooting(), m_shooter.reachedVelocity)
      .Debounce(.02_s)
      .OnTrue(m_indexer.SetIndexVel())
      .OnTrue(m_feeder.SetFeedVel());

  Or(Or(And(S(SS::SHOOT), P(SS::INTAKE)), And(S(SS::INTAKE), P(SS::SHOOT))),
     And(S(SS::SHOOT), P(SS::REV_AND_INTAKE)))
      .OnTrue(SetState(SS::SHOOT_AND_INTAKE));

  // Intake rollers
  Or(Or(S(SS::INTAKE), S(SS::SHOOT_AND_INTAKE)), S(SS::REV_AND_INTAKE))
      .OnTrue(m_intakeRollers.SetVoltage(8));

  frc2::Trigger intaking = S(SS::INTAKE);
  And(intaking, m_linearSlide.isBetweenPos)
      .WhileTrue(m_shooterPivot.SetPosition(kIntakeHoodPosition));
  And(intaking, Not(m_linearSlide.isBetweenPos))
      //        .and(linearSlide.reachedPosition)
      .OnTrue(m_shooterPivot.SetPosition(0.02));

  intaking.OnTrue(frc2::cmd::Sequence(frc2::cmd::WaitUntil([this] {
                                        return !m_linearSlide.isBetweenPos.Get() ||
                                               m_shooterPivot.ReachedPosition();
                                      }),
                                      m_linearSlide.GoToIntakeSequence()));

  Or(S(SS::SHOOT_AND_INTAKE), S(SS::REV_AND_INTAKE)).OnTrue(m_linearSlide.GoToGroundIntake());

  S(SS::IDLE)
      .OnTrue(m_intakeRollers.Off())
      .OnTrue(m_shooter.Off())
      .OnTrue(m_indexer.Off())
      .OnTrue(m_feeder.Off())
      .OnTrue(m_linearSlide.Off());

  // Kills all subsystems
  S(SS::CANCEL_ALL)
      .OnTrue(m_intakeRollers.Off())
      .OnTrue(m_linearSlide.Off())
      .OnTrue(m_shooter.Off())
      .OnTrue(m_indexer.Off())
      .OnTrue(m_feeder.Off());

  S(SS::HOME)
      .OnTrue(m_shooterPivot.SetPosition(0.05))
      .OnTrue(m_shooter.Off())
      .OnTrue(m_intakeRollers.Off())
      .OnTrue(m_indexer.Off())
      .OnTrue(m_linearSlide.GoToGroundIntake())
      .OnTrue(m_feeder.Off());

  S(SS::OUTTAKE).OnTrue(m_intakeRollers.Outtake()).OnTrue(m_feeder.Outtake());

  Or(S(SS::JITTER), S(SS::JITTER_AND_SHOOT))
      .OnTrue(m_intakeRollers.Intake())
      .OnTrue(m_linearSlide.JitterIntake());

  // (kept as written in Java: the second half uses the *previous* SHOOT_AND_INTAKE state)
  Or(And(S(SS::JITTER), Or(P(SS::SHOOT), P(SS::SHOOT_AND_INTAKE))),
     And(Or(S(SS::SHOOT), P(SS::SHOOT_AND_INTAKE)), P(SS::JITTER)))
      .OnTrue(SetState(SS::JITTER_AND_SHOOT));

  Or(And(S(SS::REV), P(SS::INTAKE)), And(S(SS::INTAKE), P(SS::REV)))
      .OnTrue(SetState(SS::REV_AND_INTAKE));

  Or(S(SS::REV), S(SS::REV_AND_INTAKE))
      .OnTrue(m_feeder.Off())
      .WhileTrue(m_shooter.ShootHub(distance, velMultiplier));

  // Hood to 0 in these states. INTAKE is intentionally not here: intakeSequence() owns the hood
  // while intaking.
  Or(Or(Or(Or(Or(S(SS::IDLE), S(SS::CANCEL_ALL)), S(SS::HOME)), S(SS::REV)), S(SS::REV_AND_INTAKE)),
     S(SS::JITTER))
      .OnTrue(m_shooterPivot.SetPosition(0.05));

  frc2::Trigger trenchZone{[this] { return IsInTrenchApproachZone(); }};
  trenchZone.OnTrue(frc2::cmd::RunOnce([this] { m_shooterPivot.SetTrenchOverride(true); }));
  trenchZone.OnFalse(frc2::cmd::RunOnce([this] { m_shooterPivot.SetTrenchOverride(false); }));
}

void Superstructure::Periodic() {
  m_stateLog.Set(ToString(m_state));
  m_prevStateLog.Set(ToString(m_prevState));
  m_stateTimeLog.Set(m_stateTimer.Get().value());
  m_velMultiplierLog.Set(m_velMultiplier);
  m_targetLog.Set(m_target);
  // Dashboard readiness lights (Superstructure/ReadyToShoot/...); only logged, nothing waits on it.
  m_ready = ReadyToShoot();
  logging::LoggedTracer::Record("Superstructure");
  m_shooterPivot.SetTrenchOverride(IsInTrenchApproachZone());
}

frc2::CommandPtr Superstructure::ChangeTarget(frc::Translation2d target) {
  return ChangeTarget([target] { return target; });
}

frc2::CommandPtr Superstructure::ChangeTarget(std::function<frc::Translation2d()> target) {
  return frc2::cmd::RunOnce([this, target = std::move(target)] {
           const frc::Translation2d t = target();
           m_target = frc::Pose2d{t, frc::Rotation2d{}};
           m_shotCalculator.SetTarget(t);
         })
      .IgnoringDisable(true);
}

bool Superstructure::ReadyToShoot() {
  // (a stopped flywheel "reaches" a request of 0, which is not ready to shoot)
  const bool flywheelReady = m_shooter.RequestedVelocity() > 0 && m_shooter.ReachedVelocity();
  const bool pivotReady = m_shooterPivot.ReachedPosition();
  // Aiming is reported but not required to shoot (the old code forced it to true).
  const bool aimed = ShotCalculator::IsAimed(m_robotPoseSupplier().Rotation(),
                                             m_shotCalculator.GetDriveAngle(), kAimToleranceRad);
  const bool ready = flywheelReady && pivotReady;
  m_readyFlywheelLog.Set(flywheelReady);
  m_readyPivotLog.Set(pivotReady);
  m_readyAimedLog.Set(aimed);
  m_readyLog.Set(ready);
  return ready;
}

bool Superstructure::GetAllianceBlue() const {
  return frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kBlue) ==
         frc::DriverStation::Alliance::kBlue;
}

bool Superstructure::TargetBlueHub() const {
  return m_robotPoseSupplier().X().value() < 4 && GetAllianceBlue();
}

bool Superstructure::TargetRedHub() const {
  return m_robotPoseSupplier().X().value() > 12.5 && !GetAllianceBlue();
}

bool Superstructure::IsRobotTopHalf() const { return m_robotPoseSupplier().Y().value() > 4; }

frc2::CommandPtr Superstructure::SetState(StructureState state) {
  return frc2::cmd::RunOnce([this, state] {
    m_prevState = m_state == state ? m_prevState : m_state;
    m_state = state;
    m_stateTimer.Restart();
    std::printf("%s->%s\n", std::string{ToString(m_prevState)}.c_str(),
                std::string{ToString(state)}.c_str());
  });
}

frc2::CommandPtr Superstructure::AddShootMultiplier(double amt) {
  return frc2::cmd::RunOnce([this, amt] { m_velMultiplier += amt; });
}

bool Superstructure::IsInTrenchApproachZone() const {
  using namespace field_constants;
  const frc::Pose2d pose = m_robotPoseSupplier();
  const double x = pose.X().value();
  const double y = pose.Y().value();
  const double approachMargin = Inches(18.0);
  const double halfTrenchLength = left_trench::width / 2.0;
  const bool nearAllianceTrench =
      std::abs(x - lines_vertical::hubCenter) <= halfTrenchLength + approachMargin;
  const bool nearOppositeTrench =
      std::abs(x - lines_vertical::oppHubCenter) <= halfTrenchLength + approachMargin;
  const bool inLeftOpening = y >= lines_horizontal::leftTrenchOpenEnd;
  const bool inRightOpening = y <= lines_horizontal::rightTrenchOpenStart;
  return (nearAllianceTrench || nearOppositeTrench) && (inLeftOpening || inRightOpening);
}
