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

#include "FieldConstants.h"
#include "logging/LoggedTracer.h"
#include "subsystems/feeder/Feeder.h"
#include "subsystems/indexer/Indexer.h"
#include "subsystems/intakerollers/IntakeRollers.h"
#include "subsystems/linearslide/LinearSlide.h"
#include "subsystems/shooter/Shooter.h"
#include "subsystems/shooterpivot/ShooterPivot.h"
#include "subsystems/shooterpivot/ShooterPivotConstants.h"
#include "subsystems/sotm/ShotCalculator.h"

using Shooter_ = Superstructure::ShooterMode;
using Intake = Superstructure::IntakeMode;

std::string_view Superstructure::ToString(ShooterMode mode) {
  switch (mode) {
    case Shooter_::OFF:
      return "OFF";
    case Shooter_::REV:
      return "REV";
    case Shooter_::SHOOTING:
      return "SHOOTING";
  }
  return "?";
}

std::string_view Superstructure::ToString(IntakeMode mode) {
  switch (mode) {
    case Intake::STOWED:
      return "STOWED";
    case Intake::INTAKING:
      return "INTAKING";
    case Intake::JITTERING:
      return "JITTERING";
    case Intake::DEPLOYED:
      return "DEPLOYED";
    case Intake::OUTTAKING:
      return "OUTTAKING";
  }
  return "?";
}

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
      m_target{field_constants::hub::kTarget2d, frc::Rotation2d{}} {
  auto clock = [] { return frc::Timer::GetFPGATimestamp().value(); };
  m_shooterMachine.Attach(m_shooterObserver, clock);
  m_intakeModeMachine.Attach(m_intakeModeObserver, clock);
  m_intakeMachine.Attach(m_intakeObserver, clock);
  m_shotMachine.Attach(m_shotObserver, clock);
  m_lastTick = clock();
  for (std::size_t i = 0; i < kInputs.size(); ++i)
    m_inputLogs[i] = std::make_unique<logging::Output<bool>>(
        std::string{"Mayhem/States/Superstructure/inputs/"} + std::string{kInputs[i]});
}

void Superstructure::ApplyOutputs() {
  // Disabled: everything off (the motors are disabled anyway; this also forgets old setpoints).
  if (!frc::DriverStation::IsEnabled()) {
    m_feedDebouncer.Calculate(false);
    m_shooter.Off();
    m_shooterPivot.Off();
    m_intakeRollers.Off();
    m_linearSlide.Off();
    m_feeder.Off();
    m_indexer.Off();
    return;
  }
  const double distance = m_shotCalculator.GetDistance();

  // Flywheel: spins while SHOOTING or REV, at the hub or the feed speed for the zone. Without a
  // valid shot distance there is no speed to ask for, so it stays off (and an alert says why).
  if ((IsShooting() || IsRevving()) && ShotDistanceValid()) {
    if (m_inAllianceZone) {
      m_shooter.ShootHub(distance, m_velMultiplier);
    } else {
      m_shooter.FeedCorner(distance);
    }
  } else {
    m_shooter.Off();
  }

  m_shooterPivot.SetPosition(HoodTarget());

  if (IsIntaking() || IsJittering()) {
    m_intakeRollers.SetVoltage(8);
  } else if (m_intakeMode == Intake::OUTTAKING) {
    m_intakeRollers.Outtake();
  } else {
    m_intakeRollers.Off();
  }

  // The slide only moves in the intake graph's deploy phases; it is off everywhere else.
  const auto phase = IntakePhase();
  if (NeedsExtension() && (phase == "DEPLOYING" || phase == "READY")) {
    m_linearSlide.GoToIntakeSequence();
  } else if (NeedsExtension() && phase == "JITTERING") {
    m_linearSlide.JitterIntake();
  } else {
    m_linearSlide.Off();
  }

  // FEEDING is entered once the flywheel first reaches speed and is latched until shooting ends, so
  // the speed dip as each piece goes through never stops the feed.
  if (m_feedDebouncer.Calculate(ShotPhase() == "FEEDING")) {
    m_indexer.SetIndexVel();
    m_feeder.SetFeedVel();
  } else {
    m_indexer.Off();
    if (m_intakeMode == Intake::OUTTAKING) {
      m_feeder.Outtake();
    } else {
      m_feeder.Off();
    }
  }
}

bool Superstructure::NeedsExtension() const {
  return IsIntaking() || IsJittering() || m_intakeMode == Intake::DEPLOYED;
}
bool Superstructure::ShotDistanceValid() const {
  return std::isfinite(m_shotCalculator.GetDistance());
}
bool Superstructure::Aimed() const {
  return ShotDistanceValid() && ShotCalculator::IsAimed(m_robotPoseSupplier().Rotation(),
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
  if (IsIntaking())
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
void Superstructure::TickMode(mayhem::StateMachine& machine) {
  // dt 0: the mode machines have no timed guards. A press is used by the first tick only; the next
  // ticks settle the mode (A released inside the alliance zone goes SHOOTING, OFF, then REV).
  machine.Tick(0);
  m_settling = true;
  for (int i = 0; i < 3 && machine.Tick(0); ++i) {
  }
  m_settling = false;
}

void Superstructure::Periodic() {
  UpdateTarget();
  if (!frc::DriverStation::IsEnabled()) {
    if (m_intakeMachine.State() != "PARKED")
      m_intakeMachine.Reset();
    if (m_shotMachine.State() != "OFF")
      m_shotMachine.Reset();
  }
  const double now = frc::Timer::GetFPGATimestamp().value();
  const double dt = now - m_lastTick;
  m_lastTick = now;
  if (!frc::DriverStation::IsEnabled()) {
    m_operator = {};
    m_autoShooting = false;
  }
  TickMode(m_shooterMachine);
  TickMode(m_intakeModeMachine);
  for (auto mode : {Shooter_::OFF, Shooter_::REV, Shooter_::SHOOTING})
    if (ToString(mode) == m_shooterMachine.State())
      m_shooterMode = mode;
  for (auto mode : {Intake::STOWED, Intake::INTAKING, Intake::JITTERING, Intake::DEPLOYED,
                    Intake::OUTTAKING})
    if (ToString(mode) == m_intakeModeMachine.State())
      m_intakeMode = mode;
  m_intakeMachine.Tick(dt);
  m_shotMachine.Tick(dt);
  m_intakeTimeLog.Set(m_intakeMachine.StateTime());
  m_shotTimeLog.Set(m_shotMachine.StateTime());
  // Slow phases are reported, never acted on: the driver's request keeps running.
  const auto intakePhase = IntakePhase();
  const double intakeTime = m_intakeMachine.StateTime();
  const bool hoodStuck = intakePhase == "CLEARING_HOOD" && intakeTime > kHoodClearWarnSeconds;
  const bool slideStuck = intakePhase == "DEPLOYING" && intakeTime > kDeployWarnSeconds;
  const bool spinStuck =
      ShotPhase() == "SPINNING_UP" && m_shotMachine.StateTime() > kSpinUpWarnSeconds;
  m_hoodStuckAlert.Set(hoodStuck);
  m_slideStuckAlert.Set(slideStuck);
  m_spinUpAlert.Set(spinStuck);
  m_faultLog.Set(hoodStuck    ? "Hood has not cleared the slide"
                 : slideStuck ? "Slide has not deployed"
                 : spinStuck  ? "Flywheel has not reached speed"
                              : "");
  m_noShotAlert.Set((IsShooting() || IsRevving()) && !ShotDistanceValid());
  m_shooterPivot.SetTrenchOverride(IsInTrenchApproachZone());
  m_ready = ReadyToShoot();
  for (std::size_t i = 0; i < kInputs.size(); ++i) {
    const auto v = Signal(kInputs[i]);
    m_inputLogs[i]->Set(std::holds_alternative<bool>(v) && std::get<bool>(v));
  }
  // The auto's one-loop requests are used up (the operator's are replaced every loop).
  m_autoIntake = m_autoStow = m_stopAll = false;
  m_shooterModeLog.Set(ToString(m_shooterMode));
  m_intakeModeLog.Set(ToString(m_intakeMode));
  m_velMultiplierLog.Set(m_velMultiplier);
  m_targetLog.Set(m_target);
  m_readyFlywheelLog.Set(m_shooter.RequestedVelocity() > 0 && m_shooter.ReachedVelocity());
  m_readyPivotLog.Set(m_shooterPivot.ReachedPosition());
  m_readyAimedLog.Set(Aimed());
  m_readyLog.Set(m_ready);
  m_feedAllowedLog.Set(m_ready && ShotPhase() == "FEEDING");
  ApplyOutputs();
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
  if (key == "enabled")
    return frc::DriverStation::IsEnabled();
  if (key == "stopAll")
    return m_stopAll && !m_settling;
  if (key == "shootHeld")
    return m_operator.shootHeld;
  if (key == "autoShooting")
    return m_autoShooting;
  if (key == "intakePressed")
    return m_operator.intakePressed && !m_settling;
  if (key == "autoIntake")
    return m_autoIntake && !m_settling;
  if (key == "autoStow")
    return m_autoStow && !m_settling;
  if (key == "jitterHeld")
    return m_operator.jitterHeld;
  if (key == "deployPressed")
    return m_operator.deployPressed && !m_settling;
  if (key == "outtakeHeld")
    return m_operator.outtakeHeld;
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
  if (key == "deploy")
    return NeedsExtension();
  if (key == "jitterDemand")
    return IsJittering();
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

void Superstructure::AddShootMultiplier(double amt) {
  if (frc::DriverStation::IsEnabled()) {
    m_velMultiplier = std::clamp(m_velMultiplier + amt, .7, 1.3);
  }
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
