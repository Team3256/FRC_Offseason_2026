// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "subsystems/Superstructure.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>

#include <frc/DriverStation.h>
#include <frc2/command/Commands.h>

#include "FieldConstants.h"
#include "commands/MayhemProject.h"
#include "logging/LoggedTracer.h"
#include "subsystems/feeder/Feeder.h"
#include "subsystems/indexer/Indexer.h"
#include "subsystems/intakerollers/IntakeRollers.h"
#include "subsystems/linearslide/LinearSlide.h"
#include "subsystems/shooter/Shooter.h"
#include "subsystems/shooterpivot/ShooterPivot.h"
#include "subsystems/shooterpivot/ShooterPivotConstants.h"
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
      m_target{field_constants::hub::kTarget2d, frc::Rotation2d{}} {
  m_stateTimer.Start();
  m_observer.Describe(robot_mayhem::SuperstructureManifest);
  for (const auto& edge : robot_mayhem::Superstructure::transitions)
    m_observer.RegisterGuard(std::string{edge.id}, "condition");
  m_machine = std::make_unique<mayhem::AutoRunner>(robot_mayhem::Superstructure::routine,
                                                   static_cast<mayhem::RobotHooks&>(*this));
  m_machine->Start();
  auto signals = [this](std::string_view key) { return Signal(key); };
  m_intakeMachine =
      std::make_unique<MayhemMachine>("IntakeSequence", robot_mayhem::IntakeSequence::routine,
                                      robot_mayhem::IntakeSequenceManifest, signals);
  m_shotMachine = std::make_unique<MayhemMachine>("ShotCycle", robot_mayhem::ShotCycle::routine,
                                                  robot_mayhem::ShotCycleManifest, signals);
  for (std::size_t i = 0; i < kInputs.size(); ++i)
    m_inputLogs[i] = std::make_unique<logging::Output<bool>>(
        std::string{"Mayhem/States/Superstructure/inputs/"} + std::string{kInputs[i]});
  ConfigStateTransitions();
}

void Superstructure::ConfigStateTransitions() {
  auto enabled = [] { return frc::DriverStation::IsEnabled(); };
  auto spinning = [this] { return IsShooting() || IsRevving(); };
  auto hub = [this] { return m_inAllianceZone; };
  auto distance = [this] { return m_shotCalculator.GetDistance(); };
  auto multiplier = [this] { return m_velMultiplier; };

  // Each mutually exclusive output group owns a command for its entire lifetime.
  // Leaving the group cancels it and its FinallyDo turns off the actual IO.
  frc2::Trigger{[=] { return enabled() && spinning() && hub(); }}.WhileTrue(
      m_shooter.ShootHub(distance, multiplier));
  frc2::Trigger{[=] { return enabled() && spinning() && !hub(); }}.WhileTrue(
      m_shooter.FeedCorner(distance));
  frc2::Trigger{[=] { return enabled() && !spinning(); }}.OnTrue(m_shooter.Off());

  frc2::Trigger{[=, this] { return enabled() && (IsIntaking() || IsJittering()); }}.WhileTrue(
      m_intakeRollers.SetVoltage(8));
  frc2::Trigger{[=, this] { return enabled() && m_state == SS::OUTTAKE; }}
      .WhileTrue(m_intakeRollers.Outtake())
      .WhileTrue(m_feeder.Outtake());
  frc2::Trigger{[=, this] {
    return enabled() && !IsIntaking() && !IsJittering() && m_state != SS::OUTTAKE;
  }}.OnTrue(m_intakeRollers.Off());

  m_shooterPivot.SetDefaultCommand(m_shooterPivot.SetPosition([this] { return HoodTarget(); }));

  frc2::Trigger{[=, this] {
    const auto phase = IntakePhase();
    return enabled() && NeedsExtension() && (phase == "DEPLOYING" || phase == "READY");
  }}.WhileTrue(m_linearSlide.GoToIntakeSequence());
  frc2::Trigger{[=, this] {
    return enabled() && NeedsExtension() && IntakePhase() == "JITTERING";
  }}.WhileTrue(m_linearSlide.JitterIntake());
  // As before the graphs: only IDLE and CANCEL_ALL turn the slide off; other states hold it.
  frc2::Trigger{[=, this] {
    return enabled() && (m_state == SS::IDLE || m_state == SS::CANCEL_ALL);
  }}.OnTrue(m_linearSlide.Off());

  // FEEDING is entered once the flywheel first reaches speed and is latched until the shot request
  // ends, so the speed dip as each piece goes through never stops the feed.
  frc2::Trigger{[=, this] { return enabled() && ShotPhase() == "FEEDING"; }}
      .Debounce(.02_s)
      .WhileTrue(m_indexer.SetIndexVel())
      .WhileTrue(m_feeder.SetFeedVel());
  frc2::Trigger{[=, this] { return enabled() && !IsShooting(); }}.OnTrue(m_indexer.Off());
  frc2::Trigger{[=, this] { return enabled() && !IsShooting() && m_state != SS::OUTTAKE; }}.OnTrue(
      m_feeder.Off());
}

bool Superstructure::IsShooting() const {
  return m_state == SS::SHOOT || m_state == SS::SHOOT_AND_INTAKE || m_state == SS::JITTER_AND_SHOOT;
}
bool Superstructure::IsIntaking() const {
  return m_state == SS::INTAKE || m_state == SS::SHOOT_AND_INTAKE || m_state == SS::REV_AND_INTAKE;
}
bool Superstructure::IsJittering() const {
  return m_state == SS::JITTER || m_state == SS::JITTER_AND_SHOOT;
}
bool Superstructure::IsRevving() const {
  return m_state == SS::REV || m_state == SS::REV_AND_INTAKE;
}
bool Superstructure::NeedsExtension() const {
  return IsIntaking() || IsJittering() || m_state == SS::HOME;
}
bool Superstructure::SlideClear() const { return !NeedsExtension() || !m_linearSlide.AtPosition(); }
bool Superstructure::PoseValid() const {
  const auto p = m_robotPoseSupplier();
  return std::isfinite(p.X().value()) && std::isfinite(p.Y().value()) &&
         std::isfinite(p.Rotation().Radians().value()) &&
         std::isfinite(m_shotCalculator.GetDistance());
}
bool Superstructure::Aimed() const {
  return PoseValid() && ShotCalculator::IsAimed(m_robotPoseSupplier().Rotation(),
                                                m_shotCalculator.GetDriveAngle(), kAimToleranceRad);
}
double Superstructure::HoodTarget() const {
  // The hood clears the slide whenever the slide is moving through the hood's travel.
  const auto phase = IntakePhase();
  if (m_linearSlide.AtPosition() && (phase == "CLEARING_HOOD" || phase == "DEPLOYING"))
    return kIntakeHoodPosition;
  if (IsShooting() || IsRevving())
    return m_inAllianceZone ? shooter_pivot::kHubLUT.Get(m_shotCalculator.GetDistance())
                            : shooter_pivot::kFeedLUT.Get(m_shotCalculator.GetDistance());
  if (m_state == SS::INTAKE)
    return m_linearSlide.AtPosition() ? kIntakeHoodPosition : .02;
  return .05;
}
bool Superstructure::ReadyToShoot() const {
  // Dashboard light: flywheel at a positive request and hood in place. Aim is logged separately
  // and is not required, matching the driver's original A-button behavior.
  return m_shooter.RequestedVelocity() > 0 && m_shooter.ReachedVelocity() &&
         m_shooterPivot.ReachedPosition();
}

void Superstructure::UpdateTarget() {
  const auto p = m_robotPoseSupplier();
  const double x = p.X().value();
  if (!std::isfinite(x) || !std::isfinite(p.Y().value()))
    return;
  // 30 cm Schmitt band prevents repeated rev/target changes at the alliance-zone line.
  if (GetAllianceBlue()) {
    if (x < 3.85)
      m_inAllianceZone = true;
    else if (x > 4.15)
      m_inAllianceZone = false;
  } else {
    if (x > 12.65)
      m_inAllianceZone = true;
    else if (x < 12.35)
      m_inAllianceZone = false;
  }
  frc::Translation2d target;
  if (m_inAllianceZone)
    target =
        GetAllianceBlue() ? field_constants::hub::kTarget2d : field_constants::hub::kOppTarget2d;
  else if (IsRobotTopHalf())
    target = GetAllianceBlue() ? kTopCorner : field_constants::FlipRotational(kBottomCorner);
  else
    target = GetAllianceBlue() ? kBottomCorner : field_constants::FlipRotational(kTopCorner);
  m_target = frc::Pose2d{target, frc::Rotation2d{}};
  m_shotCalculator.SetTarget(target);
}
void Superstructure::EvaluateState() { m_machine->Tick({}, {}, .02); }

void Superstructure::Periodic() {
  UpdateTarget();
  if (!frc::DriverStation::IsEnabled()) {
    // Clear every old demand. Re-enabling never resumes a latched shot or outtake.
    m_intent.Reset();
    if (m_intakeMachine->State() != "PARKED")
      m_intakeMachine->Reset();
    if (m_shotMachine->State() != "OFF")
      m_shotMachine->Reset();
  }
  EvaluateState();
  m_intakeMachine->Tick();
  m_shotMachine->Tick();
  // Slow phases are reported, never acted on: the driver's request keeps running.
  const auto intakePhase = IntakePhase();
  const double intakeTime = m_intakeMachine->StateTime();
  const bool hoodStuck = intakePhase == "CLEARING_HOOD" && intakeTime > kHoodClearWarnSeconds;
  const bool slideStuck = intakePhase == "DEPLOYING" && intakeTime > kDeployWarnSeconds;
  const bool spinStuck =
      ShotPhase() == "SPINNING_UP" && m_shotMachine->StateTime() > kSpinUpWarnSeconds;
  m_hoodStuckAlert.Set(hoodStuck);
  m_slideStuckAlert.Set(slideStuck);
  m_spinUpAlert.Set(spinStuck);
  m_faultLog.Set(hoodStuck    ? "Hood has not cleared the slide"
                 : slideStuck ? "Slide has not deployed"
                 : spinStuck  ? "Flywheel has not reached speed"
                              : "");
  m_shooterPivot.SetTrenchOverride(IsInTrenchApproachZone());
  m_ready = ReadyToShoot();
  for (std::size_t i = 0; i < kInputs.size(); ++i) {
    const auto v = Signal(kInputs[i]);
    m_inputLogs[i]->Set(std::holds_alternative<bool>(v) && std::get<bool>(v));
  }
  m_stateLog.Set(ToString(m_state));
  m_prevStateLog.Set(ToString(m_prevState));
  m_stateTimeLog.Set(m_stateTimer.Get().value());
  m_velMultiplierLog.Set(m_velMultiplier);
  m_targetLog.Set(m_target);
  m_readyFlywheelLog.Set(m_shooter.RequestedVelocity() > 0 && m_shooter.ReachedVelocity());
  m_readyPivotLog.Set(m_shooterPivot.ReachedPosition());
  m_readyAimedLog.Set(Aimed());
  m_readyLog.Set(m_ready);
  m_feedAllowedLog.Set(m_ready && ShotPhase() == "FEEDING");
  logging::LoggedTracer::Record("Superstructure");
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

mayhem::SignalValue Superstructure::Signal(std::string_view key) const {
  using Mode = SuperstructureIntent::Mode;
  if (key == "enabled")
    return frc::DriverStation::IsEnabled();
  if (key == "poseValid")
    return PoseValid();
  if (key == "cancel")
    return m_intent.mode == Mode::Cancel;
  if (key == "home")
    return m_intent.mode == Mode::Home;
  if (key == "outtake")
    return m_intent.mode == Mode::Outtake;
  if (key == "shoot")
    return m_intent.shoot;
  if (key == "intake")
    return m_intent.intake;
  if (key == "jitter")
    return m_intent.jitter;
  if (key == "rev")
    return m_intent.mode == Mode::Normal && (m_intent.manualRev || m_inAllianceZone);
  if (key == "inAllianceZone")
    return m_inAllianceZone;
  if (key == "ready")
    return ReadyToShoot();
  if (key == "flywheelReady")
    return m_shooter.RequestedVelocity() > 0 && m_shooter.ReachedVelocity();
  if (key == "pivotReady")
    return m_shooterPivot.ReachedPosition();
  if (key == "aimed")
    return Aimed();
  if (key == "trench")
    return IsInTrenchApproachZone();
  if (key == "slideClear")
    return SlideClear();
  if (key == "deploy")
    return NeedsExtension();
  if (key == "jitterDemand")
    return IsJittering();
  if (key == "slideBetween")
    return m_linearSlide.AtPosition();
  if (key == "hoodClear")
    return !m_linearSlide.AtPosition() ||
           (m_shooterPivot.RequestedPosition() >= kIntakeHoodPosition - .005 &&
            m_shooterPivot.ReachedPosition());
  if (key == "slideDeployed")
    return m_linearSlide.Inputs().leftMotorPosition >= .80 &&
           m_linearSlide.Inputs().rightMotorPosition >= .80;
  if (key == "shotDemand")
    return IsShooting();
  return {};
}

void Superstructure::RequestState(StructureState state, RequestSource source) {
  if (!frc::DriverStation::IsEnabled() && state != SS::CANCEL_ALL)
    return;
  if (!m_intent.Request(ToString(state), source != RequestSource::Operator))
    return;
  UpdateTarget();
  EvaluateState();
}

void Superstructure::StateEntered(std::string_view state) {
  for (int i = 0; i < kNumStates; ++i) {
    const auto next = static_cast<SS>(i);
    if (ToString(next) != state)
      continue;
    if (m_state != next)
      m_prevState = m_state;
    m_state = next;
    m_stateTimer.Restart();
    m_observer.Enter("Superstructure", state, frc::Timer::GetFPGATimestamp().value());
    return;
  }
}

void Superstructure::GuardEvaluated(std::string_view edge, bool accepted) {
  m_observer.Guard("Superstructure", edge, "condition", accepted,
                   frc::Timer::GetFPGATimestamp().value());
}
void Superstructure::Transitioned(std::string_view edge, std::string_view from,
                                  std::string_view to) {
  m_observer.Transition("Superstructure", edge, from, to, frc::Timer::GetFPGATimestamp().value());
}

frc2::CommandPtr Superstructure::SetState(StructureState state) {
  return frc2::cmd::RunOnce([this, state] { RequestState(state); });
}

frc2::CommandPtr Superstructure::AddShootMultiplier(double amt) {
  return frc2::cmd::RunOnce(
      [this, amt] { m_velMultiplier = std::clamp(m_velMultiplier + amt, .7, 1.3); });
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
