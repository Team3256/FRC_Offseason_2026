// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <array>
#include <functional>
#include <string_view>

#include <frc/Alert.h>
#include <frc/Timer.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/Trigger.h>

#include "commands/MayhemObserver.h"
#include "commands/MayhemProject.h"
#include "logging/Logger.h"
#include "mayhem/state_machine.hpp"
#include "subsystems/SuperstructureIntent.h"

class Indexer;
class ShooterPivot;
class Shooter;
class IntakeRollers;
class LinearSlide;
class Feeder;
class ShotCalculator;

/** Intent-driven Mayhem supervisor with orthogonal, sensor-driven intake and shot
 * phase graphs. The graph chooses operating modes; continuous interlocks own outputs.
 */
class Superstructure {
 public:
  enum class StructureState {
    INTAKE,
    SHOOT,
    IDLE,
    CANCEL_ALL,
    HOME,
    REV,
    SHOOT_AND_INTAKE,
    REV_AND_INTAKE,
    JITTER,
    JITTER_AND_SHOOT,
    OUTTAKE,
  };
  static constexpr int kNumStates = 11;
  static std::string_view ToString(StructureState state);

  Superstructure(Indexer& indexer, ShooterPivot& shooterPivot, Shooter& shooter,
                 IntakeRollers& intakeRollers, LinearSlide& linearSlide, Feeder& feeder,
                 ShotCalculator& shotCalculator, std::function<frc::Pose2d()> robotPoseSupplier);

  void Periodic();

  frc2::CommandPtr SetState(StructureState state);
  /** Synchronous state request for Mayhem hooks; disabled requests are inhibited. */
  enum class RequestSource { Operator, Autonomous, Supervisor };
  void RequestState(StructureState state, RequestSource source = RequestSource::Operator);
  std::string_view IntakePhase() const { return m_intakeMachine.State(); }
  std::string_view ShotPhase() const { return m_shotMachine.State(); }
  frc2::CommandPtr AddShootMultiplier(double amt);

  StructureState GetState() const { return m_state; }
  StructureState GetPrevState() const { return m_prevState; }
  double GetVelMultiplier() const { return m_velMultiplier; }
  const frc::Pose2d& GetTarget() const { return m_target; }

  /** All physical feed interlocks satisfied as of the last Periodic(). */
  bool IsReadyToShoot() const { return m_ready; }

  bool TargetBlueHub() const;
  bool TargetRedHub() const;
  bool IsInTrenchApproachZone() const;

  /** Trigger that is true while the superstructure is in {@code state}. */
  frc2::Trigger& StateTrigger(StructureState state) {
    return m_stateTriggers[static_cast<int>(state)];
  }

  /** Hood position used to clear the slide while it extends for intake. */
  static constexpr double kIntakeHoodPosition = 0.07;
  /** How far the robot's heading can be from the shot's drive angle and still count as aimed. */
  static constexpr double kAimToleranceRad = 5.0 * 3.14159265358979323846 / 180.0;
  static inline const frc::Translation2d kTopCorner{1.5_m, 6.8_m};
  static inline const frc::Translation2d kBottomCorner{1.5_m, 1.5_m};

 private:
  mayhem::SignalValue Signal(std::string_view key) const;
  SuperstructureIntent m_intent;
  // The three graphs from robot.mayhem.json; observers publish them to the log and NT.
  MayhemObserver m_observer{"Superstructure"}, m_intakeObserver{"IntakeSequence"},
      m_shotObserver{"ShotCycle"};
  mayhem::StateMachine m_machine{mayhem_generated::Superstructure::machine,
                                 [this](std::string_view key) { return Signal(key); }};
  mayhem::StateMachine m_intakeMachine{mayhem_generated::IntakeSequence::machine,
                                       [this](std::string_view key) { return Signal(key); }};
  mayhem::StateMachine m_shotMachine{mayhem_generated::ShotCycle::machine,
                                     [this](std::string_view key) { return Signal(key); }};
  double m_lastTick = 0;
  logging::Output<double> m_intakeTimeLog{"Mayhem/States/IntakeSequence/inputs/stateTime"};
  logging::Output<double> m_shotTimeLog{"Mayhem/States/ShotCycle/inputs/stateTime"};
  static constexpr std::array<std::string_view, 22> kInputs = {
      "enabled",      "poseValid", "cancel",        "home",           "outtake", "shoot",
      "intake",       "jitter",    "rev",           "inAllianceZone", "ready",   "flywheelReady",
      "pivotReady",   "aimed",     "trench",        "slideClear",     "deploy",  "jitterDemand",
      "slideBetween", "hoodClear", "slideDeployed", "shotDemand"};
  std::array<std::unique_ptr<logging::Output<bool>>, kInputs.size()> m_inputLogs;
  logging::Output<std::string> m_faultLog{"Superstructure/Fault"};
  static constexpr double kHoodClearWarnSeconds = 2.5;
  static constexpr double kDeployWarnSeconds = 3.0;
  static constexpr double kSpinUpWarnSeconds = 4.0;
  frc::Alert m_hoodStuckAlert{"Intake: hood has not cleared the slide.",
                              frc::Alert::AlertType::kWarning};
  frc::Alert m_slideStuckAlert{"Intake: slide has not deployed.", frc::Alert::AlertType::kWarning};
  frc::Alert m_spinUpAlert{"Shooter: flywheel has not reached speed.",
                           frc::Alert::AlertType::kWarning};
  bool m_inAllianceZone = false;
  void ConfigStateTransitions();
  void EvaluateState();
  void UpdateTarget();
  bool IsShooting() const;
  bool IsIntaking() const;
  bool IsJittering() const;
  bool IsRevving() const;
  bool NeedsExtension() const;
  bool PoseValid() const;
  bool SlideClear() const;
  bool Aimed() const;
  double HoodTarget() const;
  bool ReadyToShoot() const;
  bool GetAllianceBlue() const;
  bool IsRobotTopHalf() const;
  frc2::Trigger& S(StructureState s) { return StateTrigger(s); }

  StructureState m_state = StructureState::IDLE;
  StructureState m_prevState = StructureState::IDLE;
  frc::Timer m_stateTimer;

  Indexer& m_indexer;
  ShooterPivot& m_shooterPivot;
  Shooter& m_shooter;
  IntakeRollers& m_intakeRollers;
  LinearSlide& m_linearSlide;
  Feeder& m_feeder;
  ShotCalculator& m_shotCalculator;
  std::function<frc::Pose2d()> m_robotPoseSupplier;

  std::array<frc2::Trigger, kNumStates> m_stateTriggers;

  double m_velMultiplier = 1;
  bool m_ready = false;
  frc::Pose2d m_target;

  logging::Output<std::string> m_stateLog{"Superstructure/State"};
  logging::Output<std::string> m_prevStateLog{"Superstructure/PrevState"};
  logging::Output<double> m_stateTimeLog{"Superstructure/StateTime"};
  logging::Output<double> m_velMultiplierLog{"Superstructure/VelMultiplier"};
  logging::Output<frc::Pose2d> m_targetLog{"Superstructure/Target"};
  logging::Output<bool> m_feedAllowedLog{"Superstructure/FeedAllowed"};
  logging::Output<bool> m_readyFlywheelLog{"Superstructure/ReadyToShoot/Flywheel"};
  logging::Output<bool> m_readyPivotLog{"Superstructure/ReadyToShoot/Pivot"};
  logging::Output<bool> m_readyAimedLog{"Superstructure/ReadyToShoot/Aimed"};
  logging::Output<bool> m_readyLog{"Superstructure/ReadyToShoot"};
};
