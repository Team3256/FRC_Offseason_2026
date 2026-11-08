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

#include <frc/Timer.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/Trigger.h>

#include "logging/Logger.h"

class Indexer;
class ShooterPivot;
class Shooter;
class IntakeRollers;
class LinearSlide;
class Feeder;
class ShotCalculator;

/**
 * The robot's state machine. Each state is a trigger; the bindings in ConfigStateTransitions()
 * start the mechanisms' commands when states are entered (and stop them when a whileTrue state is
 * left). Binding order matters (it is the order commands are scheduled in a loop), so it follows
 * the Java version line by line.
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
  frc2::CommandPtr AddShootMultiplier(double amt);

  StructureState GetState() const { return m_state; }
  StructureState GetPrevState() const { return m_prevState; }
  double GetVelMultiplier() const { return m_velMultiplier; }
  const frc::Pose2d& GetTarget() const { return m_target; }

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
  void ConfigStateTransitions();
  frc2::CommandPtr ChangeTarget(frc::Translation2d target);
  frc2::CommandPtr ChangeTarget(std::function<frc::Translation2d()> target);
  bool ReadyToShoot();
  bool GetAllianceBlue() const;
  bool IsRobotTopHalf() const;
  frc2::Trigger& S(StructureState s) { return StateTrigger(s); }
  frc2::Trigger& P(StructureState s) { return m_prevStateTriggers[static_cast<int>(s)]; }

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
  std::array<frc2::Trigger, kNumStates> m_prevStateTriggers;

  frc2::Trigger m_targetBlueHub;
  frc2::Trigger m_targetRedHub;
  frc2::Trigger m_feedTopCorner;
  frc2::Trigger m_feedBottomCorner;

  double m_velMultiplier = 1;
  frc::Pose2d m_target;

  logging::Output<std::string> m_stateLog{"Superstructure/State"};
  logging::Output<std::string> m_prevStateLog{"Superstructure/PrevState"};
  logging::Output<double> m_stateTimeLog{"Superstructure/StateTime"};
  logging::Output<double> m_velMultiplierLog{"Superstructure/VelMultiplier"};
  logging::Output<frc::Pose2d> m_targetLog{"Superstructure/Target"};
  logging::Output<bool> m_readyFlywheelLog{"Superstructure/ReadyToShoot/Flywheel"};
  logging::Output<bool> m_readyPivotLog{"Superstructure/ReadyToShoot/Pivot"};
  logging::Output<bool> m_readyAimedLog{"Superstructure/ReadyToShoot/Aimed"};
  logging::Output<bool> m_readyLog{"Superstructure/ReadyToShoot"};
};
