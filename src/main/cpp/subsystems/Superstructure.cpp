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

using Intake = Superstructure::IntakeMode;

std::string_view Superstructure::ToString(ShooterMode mode) {
  return mayhem_generated::ShooterMode::Machine::Name(mode);
}

std::string_view Superstructure::ToString(IntakeMode mode) {
  return mayhem_generated::IntakeMode::Machine::Name(mode);
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
  // The clock also times Tick() and Settle(); every declared input is logged each loop.
  auto clock = [] { return frc::Timer::GetFPGATimestamp().value(); };
  m_shooterMachine.Attach(m_shooterObserver, clock, mayhem::InputLogging::On);
  m_intakeModeMachine.Attach(m_intakeModeObserver, clock, mayhem::InputLogging::On);
  m_intakeMachine.Attach(m_intakeObserver, clock, mayhem::InputLogging::On);
  m_shotMachine.Attach(m_shotObserver, clock, mayhem::InputLogging::On);
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
  } else if (m_intakeModeMachine.Is(Intake::OUTTAKING)) {
    m_intakeRollers.Outtake();
  } else {
    m_intakeRollers.Off();
  }

  // The slide only moves in the intake graph's deploy phases; it is off everywhere else.
  if (NeedsExtension() &&
      (m_intakeMachine.Is(IntakePhase_::DEPLOYING) || m_intakeMachine.Is(IntakePhase_::READY))) {
    m_linearSlide.GoToIntakeSequence();
  } else if (NeedsExtension() && m_intakeMachine.Is(IntakePhase_::JITTERING)) {
    m_linearSlide.JitterIntake();
  } else {
    m_linearSlide.Off();
  }

  // FEEDING is entered once the flywheel first reaches speed and is latched until shooting ends, so
  // the speed dip as each piece goes through never stops the feed.
  if (m_feedDebouncer.Calculate(m_shotMachine.Is(ShotPhase_::FEEDING))) {
    m_indexer.SetIndexVel();
    m_feeder.SetFeedVel();
  } else {
    m_indexer.Off();
    if (m_intakeModeMachine.Is(Intake::OUTTAKING)) {
      m_feeder.Outtake();
    } else {
      m_feeder.Off();
    }
  }
}

bool Superstructure::NeedsExtension() const {
  return IsIntaking() || IsJittering() || m_intakeModeMachine.Is(Intake::DEPLOYED);
}
bool Superstructure::FlywheelReady() const {
  return m_shooter.RequestedVelocity() > 0 && m_shooter.ReachedVelocity();
}
bool Superstructure::HoodClear() const {
  return !m_linearSlide.AtPosition() ||
         (m_shooterPivot.RequestedPosition() >= kIntakeHoodPosition - .005 &&
          m_shooterPivot.ReachedPosition());
}
bool Superstructure::SlideDeployed() const {
  return m_linearSlide.Inputs().leftMotorPosition >= .80 &&
         m_linearSlide.Inputs().rightMotorPosition >= .80;
}
bool Superstructure::ShotDistanceValid() const {
  return std::isfinite(m_shotCalculator.GetDistance());
}
bool Superstructure::Aimed() const {
  return ShotDistanceValid() &&
         ShotCalculator::IsAimed(m_robotPoseSupplier().Rotation(), m_shotCalculator.GetDriveAngle(),
                                 kAimToleranceRad);
}
double Superstructure::HoodTarget() const {
  // The hood clears the slide whenever the slide is moving through the hood's travel.
  if (m_linearSlide.AtPosition() && (m_intakeMachine.Is(IntakePhase_::CLEARING_HOOD) ||
                                     m_intakeMachine.Is(IntakePhase_::DEPLOYING)))
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
  return FlywheelReady() && m_shooterPivot.ReachedPosition();
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
void Superstructure::Periodic() {
  UpdateTarget();
  const bool enabled = frc::DriverStation::IsEnabled();
  if (!enabled) {
    if (!m_intakeMachine.Is(IntakePhase_::PARKED))
      m_intakeMachine.Reset();
    if (!m_shotMachine.Is(ShotPhase_::OFF))
      m_shotMachine.Reset();
    m_operator = {};
    m_autoShooting = false;
  }
  // Mode machines settle within the loop: A released inside the alliance zone goes SHOOTING, OFF,
  // then REV. Presses and the auto's requests are events, used by the first step only.
  auto& shooter = m_shooterMachine.In();
  shooter.enabled = enabled;
  shooter.stopAll = m_stopAll;
  shooter.shootHeld = m_operator.shootHeld;
  shooter.autoShooting = m_autoShooting;
  shooter.outtakeHeld = m_operator.outtakeHeld;
  shooter.inAllianceZone = m_inAllianceZone;
  m_shooterMachine.Settle();
  auto& intakeMode = m_intakeModeMachine.In();
  intakeMode.enabled = enabled;
  intakeMode.stopAll = m_stopAll;
  intakeMode.intakePressed = m_operator.intakePressed;
  intakeMode.autoIntake = m_autoIntake;
  intakeMode.autoStow = m_autoStow;
  intakeMode.jitterHeld = m_operator.jitterHeld;
  intakeMode.deployPressed = m_operator.deployPressed;
  intakeMode.outtakeHeld = m_operator.outtakeHeld;
  m_intakeModeMachine.Settle();
  // Phase machines take one step per loop, from the modes just decided and the sensors.
  auto& intake = m_intakeMachine.In();
  intake.enabled = enabled;
  intake.deploy = NeedsExtension();
  intake.jitterDemand = IsJittering();
  intake.hoodClear = HoodClear();
  intake.slideDeployed = SlideDeployed();
  m_intakeMachine.Tick();
  auto& shot = m_shotMachine.In();
  shot.enabled = enabled;
  shot.shotDemand = IsShooting();
  shot.trench = IsInTrenchApproachZone();
  shot.flywheelReady = FlywheelReady();
  m_shotMachine.Tick();
  m_intakeTimeLog.Set(m_intakeMachine.StateTime());
  m_shotTimeLog.Set(m_shotMachine.StateTime());
  // Slow phases are reported, never acted on: the driver's request keeps running.
  const double intakeTime = m_intakeMachine.StateTime();
  const bool hoodStuck =
      m_intakeMachine.Is(IntakePhase_::CLEARING_HOOD) && intakeTime > kHoodClearWarnSeconds;
  const bool slideStuck =
      m_intakeMachine.Is(IntakePhase_::DEPLOYING) && intakeTime > kDeployWarnSeconds;
  const bool spinStuck =
      m_shotMachine.Is(ShotPhase_::SPINNING_UP) && m_shotMachine.StateTime() > kSpinUpWarnSeconds;
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
  // The auto's one-loop requests are used up (the operator's are replaced every loop).
  m_autoIntake = m_autoStow = m_stopAll = false;
  m_shooterModeLog.Set(m_shooterMachine.StateName());
  m_intakeModeLog.Set(m_intakeModeMachine.StateName());
  m_velMultiplierLog.Set(m_velMultiplier);
  m_targetLog.Set(m_target);
  m_readyFlywheelLog.Set(FlywheelReady());
  m_readyPivotLog.Set(m_shooterPivot.ReachedPosition());
  m_readyAimedLog.Set(Aimed());
  m_readyLog.Set(m_ready);
  m_feedAllowedLog.Set(m_ready && m_shotMachine.Is(ShotPhase_::FEEDING));
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
