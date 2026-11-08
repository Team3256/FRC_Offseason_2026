// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <array>
#include <functional>
#include <optional>
#include <vector>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>

#include "mayhem/Trajectory.h"

namespace mayhem {

// ------------------------------------------------------------------------------------- follower
/** Field speeds plus per-module force feedforward for the drivetrain. */
struct DriveCommand {
  frc::ChassisSpeeds fieldSpeeds;
  std::array<double, kMaxModules> wheelForceX{};
  std::array<double, kMaxModules> wheelForceY{};
  int modules = kMaxModules;

  static DriveCommand Stop(int modules) {
    DriveCommand dc;
    dc.modules = modules;
    return dc;
  }
};

struct FollowerConfig {
  double translationKp = 6.0;
  double translationKv = 0.0;
  double rotationKp = 5.0;
  double maxFeedbackVelocity = 1.5;
  double maxFeedbackOmega = 3.0;
  double maxVelocity = 5.0;
  double maxOmega = 12.0;
  bool useForceFeedforward = true;
  int modules = 4;
};

class HolonomicFollower {
 public:
  explicit HolonomicFollower(FollowerConfig cfg) : m_cfg{cfg} {}
  const FollowerConfig& Config() const { return m_cfg; }
  DriveCommand Calculate(const TrajectorySample& ref, const frc::Pose2d& pose,
                         const std::optional<frc::ChassisSpeeds>& measured, double rate,
                         double feedbackScale = 1.0) const;
  static double PositionError(const TrajectorySample& ref, const frc::Pose2d& pose);
  static double HeadingError(const TrajectorySample& ref, const frc::Pose2d& pose);

 private:
  FollowerConfig m_cfg;
};

// ------------------------------------------------------------------------------------- recovery
struct RecoveryConfig {
  bool enabled = true;
  double dilationStartError = 0.08;
  double dilationStopError = 0.40;
  double dilationStartHeading = 0.15;
  double dilationStopHeading = 0.70;
  double bridgeTriggerError = 0.45;
  double bridgeTriggerHeading = 0.8;
  double accelSpikeG = 1.0;
  int persistenceLoops = 3;
  int joinCandidates = 12;
  double maxJoinLookahead = 2.5;
  double maxJoinBehind = 1.0;
  double maxJoinShortcut = 0.25;
  double replanError = 0.5;
  double minReplanInterval = 0.25;
  double limitScale = 1.0;
  double visionBoostSeconds = 1.0;
  double terrainClockGain = 10.0;
  double terrainGraceSeconds = 0.3;
  double endTimeout = 1.0;
  double endTolerance = 0.05;
  double endHeadingTolerance = 0.05;
};

struct UnbeachConfig {
  bool enabled = true;
  double tiltThresholdDegrees = 8.0;
  double detectSeconds = 0.2;
  double flatThresholdDegrees = 4.0;
  double flatSeconds = 0.15;
  double escapeDistance = 1.0;
  double escapeSpeed = 2.0;
  double minEscapeSpeed = 0.8;
  double escapeKp = 4.0;
  double reachTolerance = 0.1;
  double retargetSeconds = 0.75;
  double maxSeconds = 2.5;
  double cooldownSeconds = 0.75;
  double fieldMargin = 0.4;

  static UnbeachConfig Disabled() {
    UnbeachConfig c;
    c.enabled = false;
    return c;
  }
};

class CollisionDetector {
 public:
  explicit CollisionDetector(const RecoveryConfig& cfg) : m_cfg{cfg} {}
  void Reset();
  bool Update(double now, double posError, double headingError, double accelG,
              double expectedAccelG);
  bool VisionBoostActive(double now) const {
    return now - m_lastHitTime <= m_cfg.visionBoostSeconds;
  }

 private:
  const RecoveryConfig& m_cfg;
  int m_overCount = 0;
  double m_lastHitTime = -std::numeric_limits<double>::infinity();
};

class BeachDetector {
 public:
  explicit BeachDetector(UnbeachConfig cfg) : m_cfg{cfg} {}
  void Reset();
  bool Update(double now, double pitchDeg, double rollDeg);
  bool IsBeached() const { return m_beached; }
  static double TiltDegrees(double pitchDeg, double rollDeg);
  static frc::Translation2d Downhill(double pitchDeg, double rollDeg,
                                     const frc::Rotation2d& heading);

 private:
  UnbeachConfig m_cfg;
  bool m_beached = false;
  double m_tiltSince = kNaN;
  double m_flatSince = kNaN;
  double m_tilt = kNaN;
};

class Quintic {
 public:
  Quintic() = default;
  Quintic(double p0, double v0, double a0, double p1, double v1, double a1, double T);
  double P(double t) const { return c0 + t * (c1 + t * (c2 + t * (c3 + t * (c4 + t * c5)))); }
  double V(double t) const { return c1 + t * (2 * c2 + t * (3 * c3 + t * (4 * c4 + t * 5 * c5))); }
  double A(double t) const { return 2 * c2 + t * (6 * c3 + t * (12 * c4 + t * 20 * c5)); }
  double duration = 0;

 private:
  double c0 = 0, c1 = 0, c2 = 0, c3 = 0, c4 = 0, c5 = 0;
};

class Bridge {
 public:
  struct Segment {
    Quintic x, y, heading;
    double duration;
  };
  Bridge(std::vector<Segment> segments, double joinTime);
  double Duration() const { return m_duration; }
  TrajectorySample SampleAt(double tau) const;
  std::vector<frc::Pose2d> Poses(int n) const;
  double joinTime;

 private:
  std::vector<Segment> m_segments;
  double m_duration = 0;
};

class BridgePlanner {
 public:
  struct Result {
    std::optional<Bridge> bridge;
    int candidatesTried = 0;
    double planSeconds = 0;
  };
  explicit BridgePlanner(const RecoveryConfig& cfg) : m_cfg{cfg} {}
  Result Plan(const MayhemTrajectory& traj, double tNow, const frc::Pose2d& pose,
              const std::optional<frc::ChassisSpeeds>& fieldSpeeds) const;

 private:
  bool CutsOffPath(const MayhemTrajectory& traj, double tNow, double tj, const frc::Pose2d& pose,
                   const TrajectorySample& target, double offPath) const;
  const RecoveryConfig& m_cfg;
};

// ------------------------------------------------------------------------------------- runner
/**
 * Runs one trajectory: feedforward + saturated feedback, time dilation, bump detection and bridge
 * recovery, auto unbeach, rough-terrain handling, and event markers. Framework-free so it is unit
 * testable. Call Start() once, then Update() every loop with the latest vision-fused pose.
 */
class TrajectoryRunner {
 public:
  enum class State { IDLE, FOLLOWING, BRIDGING, UNBEACHING, SETTLING, FINISHED };
  static const char* StateName(State s);

  /** Receives event markers. Zone markers get OnEvent at start and OnZoneEnd at end. */
  struct EventListener {
    std::function<void(const TrajectoryEvent&)> onEvent;
    std::function<void(const TrajectoryEvent&)> onZoneEnd;
  };

  TrajectoryRunner(TrajectoryPtr traj, FollowerConfig fc, RecoveryConfig rc,
                   EventListener listener);
  TrajectoryRunner& WithUnbeach(std::optional<UnbeachConfig> config);

  void Start(double now);
  /** Stops the runner early (e.g. command interrupted); closes open zones. */
  void Stop();
  DriveCommand Update(double now, const frc::Pose2d& pose,
                      const std::optional<frc::ChassisSpeeds>& fieldSpeeds, double accelG,
                      double pitchDegrees = kNaN, double rollDegrees = kNaN);

  State GetState() const { return m_state; }
  bool IsFinished() const { return m_state == State::FINISHED; }
  bool IsRecovering() const { return m_state == State::BRIDGING; }
  bool IsUnbeaching() const { return m_state == State::UNBEACHING; }
  const MayhemTrajectory& Trajectory() const { return *m_traj; }
  double TrajectoryTime() const { return m_t; }
  double ClockRate() const { return m_rate; }
  const TrajectorySample& Reference() const { return m_reference; }
  std::optional<frc::Translation2d> UnbeachTarget() const {
    return m_state == State::UNBEACHING ? m_unbeachTarget : std::nullopt;
  }
  int UnbeachCount() const { return m_unbeachCount; }
  double TiltDegrees() const { return BeachDetector::TiltDegrees(m_pitchDeg, m_rollDeg); }
  const std::optional<Bridge>& GetBridge() const { return m_bridge; }
  double PositionError() const { return m_posError; }
  double HeadingError() const { return m_headingError; }
  int BridgesPlanned() const { return m_bridgesPlanned; }
  double LastPlanSeconds() const { return m_lastPlanSeconds; }
  bool IntakeExtended() const;
  bool OnRoughTerrain() const { return m_terrain != nullptr && m_state == State::FOLLOWING; }
  bool VisionBoostActive() const;

 private:
  DriveCommand UpdateFollowing(double now, double dt, const frc::Pose2d& pose,
                               const std::optional<frc::ChassisSpeeds>& v, double accelG);
  DriveCommand UpdateTerrain(double dt, const TerrainSpan& span, const TrajectorySample& ref,
                             const frc::Pose2d& pose, const std::optional<frc::ChassisSpeeds>& v);
  DriveCommand UpdateSettling(double now, const frc::Pose2d& pose,
                              const std::optional<frc::ChassisSpeeds>& v, double accelG);
  DriveCommand UpdateBridging(double now, double dt, const frc::Pose2d& pose,
                              const std::optional<frc::ChassisSpeeds>& v, double accelG);
  DriveCommand UpdateUnbeaching(double now, double dt, const frc::Pose2d& pose,
                                const std::optional<frc::ChassisSpeeds>& v, double accelG);
  bool CheckBeached(double now, const frc::Pose2d& pose);
  frc::Translation2d ClampToField(const frc::Translation2d& p) const;
  bool TryPlan(double now, const frc::Pose2d& pose, const std::optional<frc::ChassisSpeeds>& v);
  void FireCrossing(double from, double to);
  void Fire(size_t i);
  void Finish();
  void CloseAllZones();
  void Measure(const TrajectorySample& ref, const frc::Pose2d& pose);
  bool Pending(size_t i) const;

  TrajectoryPtr m_traj;
  HolonomicFollower m_follower;
  RecoveryConfig m_rc;
  BridgePlanner m_planner;
  CollisionDetector m_detector;
  EventListener m_listener;
  const std::vector<TrajectoryEvent>& m_events;
  std::vector<bool> m_fired;
  std::vector<bool> m_zoneEnded;
  std::vector<size_t> m_pendingAtJoin;
  State m_state = State::IDLE;
  double m_t = 0;
  double m_lastNow = 0;
  double m_rate = 1;
  double m_settleStart = 0;
  std::optional<Bridge> m_bridge;
  double m_bridgeFrom = 0;
  double m_tau = 0;
  double m_lastPlanTime = -std::numeric_limits<double>::infinity();
  TrajectorySample m_reference;
  double m_posError = 0;
  double m_headingError = 0;
  int m_bridgesPlanned = 0;
  double m_lastPlanSeconds = 0;
  double m_lastNowForBoost = -std::numeric_limits<double>::infinity();
  const TerrainSpan* m_terrain = nullptr;
  double m_terrainExitTime = -std::numeric_limits<double>::infinity();
  UnbeachConfig m_unbeachConfig = UnbeachConfig::Disabled();
  BeachDetector m_beach{UnbeachConfig::Disabled()};
  double m_pitchDeg = kNaN;
  double m_rollDeg = kNaN;
  double m_unbeachStart = 0;
  double m_unbeachRetargetAt = 0;
  double m_unbeachCooldownUntil = -std::numeric_limits<double>::infinity();
  std::optional<frc::Translation2d> m_unbeachTarget;
  int m_unbeachCount = 0;
};

}  // namespace mayhem
