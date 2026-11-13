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
#include <frc/filter/Debouncer.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>

#include "commands/MayhemObserver.h"
#include "commands/MayhemProject.h"
#include "logging/Logger.h"
#include "mayhem/state_machine.hpp"

class Indexer;
class ShooterPivot;
class Shooter;
class IntakeRollers;
class LinearSlide;
class Feeder;
class ShotCalculator;

/**
 * Runs the robot's four Mayhem state machines (robot.mayhem.json) and drives every mechanism from
 * their states. Two mode machines say what the driver or the auto wants:
 *  - ShooterMode: OFF, REV (pre-spin in the alliance zone) or SHOOTING (A held, or the auto).
 *  - IntakeMode: STOWED, INTAKING (X toggles), JITTERING (Y held), DEPLOYED (B toggles: slide out,
 *    rollers off) or OUTTAKING (left trigger held).
 * Two phase machines do the physical sequencing under them: IntakeSequence (clear the hood, then
 * deploy the slide) and ShotCycle (spin up, then feed). Each loop: SetOperatorInputs() and the
 * auto's requests, then Periodic() ticks the machines and applies the outputs (ApplyOutputs).
 */
class Superstructure {
 public:
  enum class ShooterMode { OFF, REV, SHOOTING };
  enum class IntakeMode { STOWED, INTAKING, JITTERING, DEPLOYED, OUTTAKING };
  static std::string_view ToString(ShooterMode mode);
  static std::string_view ToString(IntakeMode mode);

  Superstructure(Indexer& indexer, ShooterPivot& shooterPivot, Shooter& shooter,
                 IntakeRollers& intakeRollers, LinearSlide& linearSlide, Feeder& feeder,
                 ShotCalculator& shotCalculator, std::function<frc::Pose2d()> robotPoseSupplier);

  /** The driver's buttons this loop: presses are rising edges, helds are levels. */
  struct OperatorInputs {
    bool shootHeld = false;      // A
    bool intakePressed = false;  // X
    bool jitterHeld = false;     // Y
    bool deployPressed = false;  // B
    bool outtakeHeld = false;    // left trigger
  };
  /** Call once per loop before Periodic(); all false while disabled or during an auto. */
  void SetOperatorInputs(const OperatorInputs& inputs) { m_operator = inputs; }

  // Autonomous requests, applied by the next Periodic().
  void StartIntaking() { m_autoIntake = true; }
  void StowIntake() { m_autoStow = true; }
  void SetAutoShooting(bool shooting) { m_autoShooting = shooting; }
  /** Everything off (the auto ended or failed, or test mode started). */
  void StopAll() {
    m_stopAll = true;
    m_autoShooting = false;
  }

  /** Ticks the state machines and drives the mechanisms. Call once per loop, after the inputs. */
  void Periodic();

  ShooterMode GetShooterMode() const { return m_shooterMode; }
  IntakeMode GetIntakeMode() const { return m_intakeMode; }
  std::string_view IntakePhase() const { return m_intakeMachine.State(); }
  std::string_view ShotPhase() const { return m_shotMachine.State(); }
  /** Nudges the hub shot speed multiplier (D-pad), clamped to [0.7, 1.3]. Ignored while disabled.
   */
  void AddShootMultiplier(double amt);

  double GetVelMultiplier() const { return m_velMultiplier; }
  const frc::Pose2d& GetTarget() const { return m_target; }

  /** All physical feed interlocks satisfied as of the last Periodic(). */
  bool IsReadyToShoot() const { return m_ready; }

  bool TargetBlueHub() const;
  bool TargetRedHub() const;
  bool IsInTrenchApproachZone() const;

  /** Hood position used to clear the slide while it extends for intake. */
  static constexpr double kIntakeHoodPosition = 0.07;
  /** How far the robot's heading can be from the shot's drive angle and still count as aimed. */
  static constexpr double kAimToleranceRad = 5.0 * 3.14159265358979323846 / 180.0;
  static inline const frc::Translation2d kTopCorner{1.5_m, 6.8_m};
  static inline const frc::Translation2d kBottomCorner{1.5_m, 1.5_m};

 private:
  mayhem::SignalValue Signal(std::string_view key) const;
  /** Ticks a mode machine with this loop's presses, then without them until it settles. */
  void TickMode(mayhem::StateMachine& machine);
  OperatorInputs m_operator;
  /** Hides the one-loop presses and requests while a mode machine settles. */
  bool m_settling = false;
  bool m_autoIntake = false, m_autoStow = false, m_autoShooting = false, m_stopAll = false;
  // The four graphs from robot.mayhem.json; observers publish them to the log and NT.
  MayhemObserver m_shooterObserver{"ShooterMode"}, m_intakeModeObserver{"IntakeMode"},
      m_intakeObserver{"IntakeSequence"}, m_shotObserver{"ShotCycle"};
  mayhem::StateMachine m_shooterMachine{mayhem_generated::ShooterMode::machine,
                                        [this](std::string_view key) { return Signal(key); }};
  mayhem::StateMachine m_intakeModeMachine{mayhem_generated::IntakeMode::machine,
                                           [this](std::string_view key) { return Signal(key); }};
  mayhem::StateMachine m_intakeMachine{mayhem_generated::IntakeSequence::machine,
                                       [this](std::string_view key) { return Signal(key); }};
  mayhem::StateMachine m_shotMachine{mayhem_generated::ShotCycle::machine,
                                     [this](std::string_view key) { return Signal(key); }};
  double m_lastTick = 0;
  logging::Output<double> m_intakeTimeLog{"Mayhem/States/IntakeSequence/inputs/stateTime"};
  logging::Output<double> m_shotTimeLog{"Mayhem/States/ShotCycle/inputs/stateTime"};
  static constexpr std::array<std::string_view, 21> kInputs = {
      "enabled",       "stopAll",       "shootHeld",    "autoShooting",   "intakePressed",
      "autoIntake",    "autoStow",      "jitterHeld",   "deployPressed",  "outtakeHeld",
      "inAllianceZone", "ready",        "flywheelReady", "pivotReady",    "aimed",
      "trench",        "deploy",        "jitterDemand", "hoodClear",      "slideDeployed",
      "shotDemand"};
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
  frc::Alert m_noShotAlert{"Shooter: no valid shot distance (pose); flywheel off.",
                           frc::Alert::AlertType::kWarning};
  bool m_inAllianceZone = false;
  /** Every mechanism's request for this loop, from the states. */
  void ApplyOutputs();
  void UpdateTarget();
  bool IsShooting() const { return m_shooterMode == ShooterMode::SHOOTING; }
  bool IsRevving() const { return m_shooterMode == ShooterMode::REV; }
  bool IsIntaking() const { return m_intakeMode == IntakeMode::INTAKING; }
  bool IsJittering() const { return m_intakeMode == IntakeMode::JITTERING; }
  bool NeedsExtension() const;
  bool ShotDistanceValid() const;
  bool Aimed() const;
  double HoodTarget() const;
  bool ReadyToShoot() const;
  bool GetAllianceBlue() const;
  bool IsRobotTopHalf() const;

  ShooterMode m_shooterMode = ShooterMode::OFF;
  IntakeMode m_intakeMode = IntakeMode::STOWED;

  Indexer& m_indexer;
  ShooterPivot& m_shooterPivot;
  Shooter& m_shooter;
  IntakeRollers& m_intakeRollers;
  LinearSlide& m_linearSlide;
  Feeder& m_feeder;
  ShotCalculator& m_shotCalculator;
  std::function<frc::Pose2d()> m_robotPoseSupplier;

  // FEEDING must hold for 20 ms before the indexer and feeder run.
  frc::Debouncer m_feedDebouncer{20_ms, frc::Debouncer::DebounceType::kRising};

  double m_velMultiplier = 1;
  bool m_ready = false;
  frc::Pose2d m_target;

  logging::Output<std::string> m_shooterModeLog{"Superstructure/ShooterMode"};
  logging::Output<std::string> m_intakeModeLog{"Superstructure/IntakeMode"};
  logging::Output<double> m_velMultiplierLog{"Superstructure/VelMultiplier"};
  logging::Output<frc::Pose2d> m_targetLog{"Superstructure/Target"};
  logging::Output<bool> m_feedAllowedLog{"Superstructure/FeedAllowed"};
  logging::Output<bool> m_readyFlywheelLog{"Superstructure/ReadyToShoot/Flywheel"};
  logging::Output<bool> m_readyPivotLog{"Superstructure/ReadyToShoot/Pivot"};
  logging::Output<bool> m_readyAimedLog{"Superstructure/ReadyToShoot/Aimed"};
  logging::Output<bool> m_readyLog{"Superstructure/ReadyToShoot"};
};
