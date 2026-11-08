// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

#include <frc/event/EventLoop.h>
#include <frc/geometry/Pose2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Subsystem.h>
#include <frc2/command/button/Trigger.h>
#include <networktables/BooleanTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/StringTopic.h>
#include <networktables/StructArrayTopic.h>
#include <networktables/StructTopic.h>
#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableHelper.h>

#include "mayhem/Runner.h"
#include "mayhem/Trajectory.h"

namespace mayhem {

class AutoRoutine;
class AutoTrajectory;

/** Publishes reference/bridge/state to NetworkTables (under /Mayhem). */
class MayhemTelemetry {
 public:
  explicit MayhemTelemetry(std::string_view root);
  void StartTrajectory(const MayhemTrajectory& traj);
  void Update(const TrajectoryRunner& r);
  void Stop();

 private:
  nt::StructPublisher<frc::Pose2d> m_reference;
  nt::StructArrayPublisher<frc::Pose2d> m_path;
  nt::StructArrayPublisher<frc::Pose2d> m_bridge;
  nt::StringPublisher m_state;
  nt::StringPublisher m_name;
  nt::DoublePublisher m_time;
  nt::DoublePublisher m_rate;
  nt::DoublePublisher m_error;
  nt::DoublePublisher m_planMs;
  nt::DoublePublisher m_bridges;
  nt::BooleanPublisher m_terrain;
  nt::BooleanPublisher m_unbeaching;
  nt::DoublePublisher m_tilt;
  nt::DoublePublisher m_unbeaches;
  nt::StructArrayPublisher<frc::Pose2d> m_unbeachTarget;
};

/**
 * Entry point for robot code, modeled on Choreo's AutoFactory. The controller is built in:
 * MayhemLib runs the feedback, time dilation and bump recovery itself, and hands the drivetrain a
 * finished DriveCommand (field speeds plus per-module force feedforward).
 */
class AutoFactory {
 public:
  /**
   * @param poseSupplier vision-fused robot pose (blue-origin field frame on both alliances)
   * @param resetOdometry resets the pose estimate; used by ResetOdometry()
   * @param fieldSpeedsSupplier measured field-relative chassis speeds
   * @param controller applies a DriveCommand to the drivetrain
   * @param useAllianceFlipping run the red-alliance version of each path when the DS reports red
   * @param driveSubsystem the drivetrain; required by every trajectory command
   */
  AutoFactory(std::function<frc::Pose2d()> poseSupplier,
              std::function<void(const frc::Pose2d&)> resetOdometry,
              std::function<frc::ChassisSpeeds()> fieldSpeedsSupplier,
              std::function<void(const DriveCommand&)> controller, bool useAllianceFlipping,
              frc2::Subsystem* driveSubsystem);

  /** Creates a new, empty auto routine. Build its trajectories and triggers, then return it. */
  std::shared_ptr<AutoRoutine> NewRoutine(std::string name);

  /** Binds a factory so each event marker firing gets a fresh command instance. */
  AutoFactory& Bind(std::string name, std::function<frc2::CommandPtr()> cmdFactory);

  AutoFactory& WithFollowerConfig(FollowerConfig cfg);
  AutoFactory& WithRecoveryConfig(RecoveryConfig cfg);
  /** Horizontal acceleration magnitude in g (e.g. from a Pigeon 2) for collision detection. */
  AutoFactory& WithAccelerometer(std::function<double()> accelG);
  /** IMU pitch and roll in degrees (CTRE Pigeon 2 convention), for WithUnbeach. */
  AutoFactory& WithTiltSensor(std::function<double()> pitchDegrees,
                              std::function<double()> rollDegrees);
  AutoFactory& WithUnbeach(std::optional<UnbeachConfig> cfg = UnbeachConfig{});
  /** Called with true right after a detected hit and false afterwards: raise vision trust. */
  AutoFactory& WithVisionBoost(std::function<void(bool)> visionBoost);
  /** Overrides alliance detection: return true to run the red-alliance version. */
  AutoFactory& WithAllianceFlip(std::function<bool()> useRedAlliance);
  /** Publish reference/bridge/state to NetworkTables under /Mayhem. */
  AutoFactory& WithTelemetry(bool enabled);

  bool IsRed() const { return m_useRedAlliance(); }

 private:
  friend class AutoRoutine;
  friend class AutoTrajectory;

  std::function<frc::Pose2d()> m_pose;
  std::function<void(const frc::Pose2d&)> m_resetOdometry;
  std::function<frc::ChassisSpeeds()> m_fieldSpeeds;
  std::function<void(const DriveCommand&)> m_output;
  frc2::Subsystem* m_driveSubsystem;
  std::function<bool()> m_useRedAlliance;
  std::function<double()> m_accelG = [] { return kNaN; };
  std::function<double()> m_pitchDegrees = [] { return kNaN; };
  std::function<double()> m_rollDegrees = [] { return kNaN; };
  std::optional<UnbeachConfig> m_unbeachConfig;
  std::function<void(bool)> m_visionBoost = [](bool) {};
  FollowerConfig m_followerConfig;
  RecoveryConfig m_recoveryConfig;
  std::unique_ptr<MayhemTelemetry> m_telemetry;
  std::unordered_map<std::string, std::function<frc2::CommandPtr()>> m_bindings;
  // commands scheduled from markers are kept alive here
  std::vector<std::unique_ptr<frc2::Command>> m_boundCommands;

  static TrajectoryPtr LoadTrajectory(const std::string& name, int splitIndex);
  frc2::Command* BoundCommand(const std::string& name);
};

/**
 * A trigger-based auto, modeled on Choreo's AutoRoutine. It owns an EventLoop that Cmd() polls,
 * so every trigger made from this routine or its trajectories only reacts while the routine runs.
 * Routines are held by shared_ptr: the command from Cmd() keeps its routine alive.
 */
class AutoRoutine : public std::enable_shared_from_this<AutoRoutine> {
 public:
  AutoRoutine(AutoFactory& factory, std::string name);
  ~AutoRoutine();
  AutoRoutine(const AutoRoutine&) = delete;
  AutoRoutine& operator=(const AutoRoutine&) = delete;

  const std::string& Name() const { return m_name; }

  /**
   * Loads deploy/mayhem/<name>.mtraj as a trajectory of this routine. If it cannot be loaded, a
   * warning goes to the Driver Station and an empty trajectory is returned whose commands do
   * nothing and whose triggers never fire.
   */
  AutoTrajectory& Trajectory(const std::string& trajectoryName, int splitIndex = -1);
  AutoTrajectory& Trajectory(TrajectoryPtr trajectory);

  /** True while the routine's command runs. Bind the first step to its rising edge. */
  frc2::Trigger Active();
  /** A trigger on this routine's event loop, so it is only polled while the routine runs. */
  frc2::Trigger Observe(std::function<bool()> condition);
  /** True while no trajectory of this routine is running. */
  frc2::Trigger Idle();

  frc::EventLoop* Loop() { return &m_loop; }
  /** Polls the routine's triggers once. Cmd() calls this every loop. */
  void Poll();
  /** Stops the routine and cancels its trajectory commands; it can run again later. */
  void Reset();
  /** Stops the routine for good: Cmd() ends and triggers stop firing. */
  void Kill();
  /** Runs the routine until it is killed or cancelled (usually when autonomous ends). */
  frc2::CommandPtr Cmd();
  frc2::CommandPtr Cmd(std::function<bool()> finishCondition);

  int64_t PollCount() const { return m_pollCount; }
  int TrajectoryStarts() const { return m_trajectoryStarts; }
  void OnTrajectoryStart() { m_trajectoryStarts++; }
  bool IsActive() const { return m_isActive; }

 private:
  AutoFactory& m_factory;
  std::string m_name;
  frc::EventLoop m_loop;
  std::vector<std::unique_ptr<AutoTrajectory>> m_trajectories;
  bool m_isActive = false;
  bool m_isKilled = false;
  int64_t m_pollCount = 0;
  int m_trajectoryStarts = 0;
};

/**
 * A trajectory (or one split segment) in an AutoRoutine, modeled on Choreo's AutoTrajectory: a
 * follow command plus triggers for its timeline. One-cycle triggers (Done(), AtTime) are true on
 * the routine poll right after the event happened.
 */
class AutoTrajectory {
 public:
  AutoTrajectory(AutoFactory& f, AutoRoutine& routine, TrajectoryPtr base);
  /** A placeholder for a trajectory that failed to load. */
  AutoTrajectory(AutoFactory& f, AutoRoutine& routine, std::string missingName);
  ~AutoTrajectory();

  const std::string& Name() const { return m_name; }
  const MayhemTrajectory* GetRawTrajectory() const { return m_base.get(); }
  bool IsLoaded() const { return m_base != nullptr; }
  std::optional<frc::Pose2d> GetInitialPose() const;
  std::optional<frc::Pose2d> GetFinalPose() const;

  /** Resets odometry to the alliance-correct start pose (resolved when the command runs). */
  frc2::CommandPtr ResetOdometry();
  /** Follows the trajectory with bump recovery, requires the drivetrain, ends when it finishes. */
  frc2::CommandPtr Cmd();

  /** True while this trajectory's command runs. */
  frc2::Trigger Active();
  frc2::Trigger Inactive();
  /** True for one cycle after the trajectory finishes (not when it is interrupted). */
  frc2::Trigger Done(int cyclesToDelay = 0);
  /** True for {@code seconds} after the trajectory finishes. */
  frc2::Trigger DoneFor(double seconds);
  /** True from when the trajectory finishes until another trajectory in the routine starts. */
  frc2::Trigger RecentlyDone();
  /** True for one cycle when the trajectory clock passes {@code timeSinceStart} seconds. */
  frc2::Trigger AtTime(double timeSinceStart);
  /** True for one cycle when marker {@code eventName} fires, and for the whole of a zone marker. */
  frc2::Trigger AtTime(const std::string& eventName);
  frc2::Trigger Recovering();
  frc2::Trigger Unbeaching();
  frc2::Trigger OnRoughTerrain();
  frc2::Trigger IntakeExtended();

  bool IsActive() const { return m_isActive; }
  bool HasFinished() const { return m_hasFinished; }
  /** Cancels a running command and forgets finished state; called when the routine stops. */
  void Reset();

 private:
  class FollowCommand;
  friend class FollowCommand;

  /** Marks the routine poll an event happened on, so every binding of a trigger sees it once. */
  struct Pulse {
    int64_t at = INT64_MIN / 2;
  };
  void Fire(Pulse& p) { p.at = m_routine.PollCount(); }
  bool High(const Pulse& p, int delayCycles) const {
    return m_routine.PollCount() == p.at + 1 + delayCycles;
  }
  Pulse& MarkerPulse(const std::string& name);
  frc2::CommandPtr MissingCmd(const std::string& what);
  TrajectoryPtr Resolved() const;
  void OnEvent(const TrajectoryEvent& e);
  void OnZoneEnd(const TrajectoryEvent& e);

  AutoFactory& m_f;
  AutoRoutine& m_routine;
  TrajectoryPtr m_base;  // blue alliance; null if the trajectory failed to load
  std::string m_name;
  std::map<std::string, std::unique_ptr<Pulse>> m_markerPulses;
  std::map<std::string, bool> m_zoneActive;
  std::map<std::string, frc2::Command*> m_runningZones;
  std::map<double, std::unique_ptr<Pulse>> m_timePulses;
  Pulse m_donePulse;
  frc2::Command* m_activeCmd = nullptr;
  bool m_isActive = false;
  bool m_recovering = false;
  bool m_unbeaching = false;
  bool m_onTerrain = false;
  bool m_intakeOut = false;
  bool m_hasFinished = false;
  double m_finishedAt = -std::numeric_limits<double>::infinity();
  int m_startsAtFinish = -1;
  double m_time = 0;
  TrajectoryPtr m_running;  // alliance-correct trajectory of the current run
  std::unique_ptr<TrajectoryRunner> m_runner;
};

/**
 * A dashboard auto picker, modeled on Choreo's AutoChooser. Routines are built lazily: only the
 * selected one is generated, when it is picked while the robot is disabled (so trajectory loading
 * happens before the match, not at the start of auto).
 */
class AutoChooser : public wpi::Sendable, public wpi::SendableHelper<AutoChooser> {
 public:
  static constexpr const char* kNoneName = "Nothing";

  AutoChooser();

  /** Adds a routine; {@code generator} runs only when this option is selected. */
  AutoChooser& AddRoutine(const std::string& name,
                          std::function<std::shared_ptr<AutoRoutine>()> generator);
  /** Adds a plain command; {@code generator} runs only when this option is selected. */
  AutoChooser& AddCmd(const std::string& name, std::function<frc2::CommandPtr()> generator);
  /** Selects an option by name, as the dashboard would. Unknown names are ignored. */
  void Select(const std::string& name);
  const std::string& Selected() const { return m_selected; }
  std::vector<std::string> Options() const { return m_order; }
  /** The command for the selected option, generating it if the selection changed. */
  frc2::Command* SelectedCommand();
  /** Runs the selected auto as a proxy; bind it to RobotModeTriggers::Autonomous().WhileTrue. */
  frc2::CommandPtr SelectedCommandScheduler();

  void InitSendable(wpi::SendableBuilder& builder) override;

 private:
  void Build();

  std::map<std::string, std::function<frc2::CommandPtr()>> m_generators;
  std::vector<std::string> m_order;
  int m_instance;
  std::string m_selected = kNoneName;
  std::string m_builtFor = kNoneName;
  std::optional<frc2::CommandPtr> m_built;
};

}  // namespace mayhem
