// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <string>
#include <vector>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>

/**
 * C++ port of MayhemLib (https://ryanabraham1.github.io/Mayhem), the team's trajectory library.
 * MayhemLib only ships for Java, so the parts this robot uses are ported here with the same
 * behaviour: trajectory model + .mtraj loader, holonomic follower with time dilation, bump
 * recovery (bridge planner), auto unbeach, rough terrain handling, event markers, and the
 * Choreo-style AutoFactory / AutoRoutine / AutoTrajectory / AutoChooser API.
 */
namespace mayhem {

inline constexpr int kMaxModules = 4;
inline constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

enum class FieldSymmetry { ROTATIONAL, MIRROR };

FieldSymmetry FieldSymmetryFromString(const std::string& s);

struct Symmetry {
  static double FlipX(FieldSymmetry, double x, double length) { return length - x; }
  static double FlipY(FieldSymmetry s, double y, double width) {
    return s == FieldSymmetry::ROTATIONAL ? width - y : y;
  }
  static double FlipHeading(FieldSymmetry s, double heading) {
    return s == FieldSymmetry::ROTATIONAL ? heading + std::numbers::pi : std::numbers::pi - heading;
  }
  static std::array<double, 2> FlipVector(FieldSymmetry s, double x, double y) {
    return s == FieldSymmetry::ROTATIONAL ? std::array<double, 2>{-x, -y}
                                          : std::array<double, 2>{-x, y};
  }
  static double FlipAngular(FieldSymmetry s, double w) {
    return s == FieldSymmetry::ROTATIONAL ? w : -w;
  }
};

/**
 * One state along a trajectory. All vectors are field-frame. Heading is continuous (unwrapped).
 * Module forces are field-frame newtons in module order (FL, FR, BL, BR); {@code nForces} of them
 * are valid (0 when the sample has none).
 */
struct TrajectorySample {
  double t = 0, x = 0, y = 0, heading = 0;
  double vx = 0, vy = 0, omega = 0;
  double ax = 0, ay = 0, alpha = 0;
  std::array<double, kMaxModules> fx{};
  std::array<double, kMaxModules> fy{};
  int nForces = 0;

  frc::Pose2d GetPose() const {
    return frc::Pose2d{units::meter_t{x}, units::meter_t{y},
                       frc::Rotation2d{units::radian_t{heading}}};
  }
  frc::ChassisSpeeds GetFieldSpeeds() const {
    return frc::ChassisSpeeds{units::meters_per_second_t{vx}, units::meters_per_second_t{vy},
                              units::radians_per_second_t{omega}};
  }
  double Speed() const { return std::hypot(vx, vy); }
  TrajectorySample WithTime(double newT) const {
    TrajectorySample s = *this;
    s.t = newT;
    return s;
  }
  TrajectorySample Flipped(FieldSymmetry sym, double length, double width) const;

  /** State at time t inside [start, end], using the optimizer's constant-acceleration model. */
  static TrajectorySample Integrate(const TrajectorySample& start, const TrajectorySample& end,
                                    double t);
};

struct TrajectoryEvent {
  enum class RecoveryPolicy { FIRE_AT_JOIN, FIRE_IMMEDIATELY, SKIP };
  static RecoveryPolicy PolicyFromString(const std::string& s);

  TrajectoryEvent(std::string name, std::string command, double t, double endT,
                  RecoveryPolicy policy, bool mustHit);

  std::string name;
  std::string command;
  double t;
  double endT;
  RecoveryPolicy policy;
  bool mustHit;

  bool IsZone() const { return !std::isnan(endT); }
  TrajectoryEvent Shifted(double dt) const {
    return TrajectoryEvent{name, command, t + dt, IsZone() ? endT + dt : endT, policy, mustHit};
  }
};

struct TerrainSpan {
  double t;
  double endT;
  double expectedSpeed;
  double feedbackScale;
  bool Contains(double time) const { return time >= t && time <= endT; }
};

struct IntakeSpan {
  double t;
  double endT;
  bool Contains(double time) const { return time >= t && time <= endT; }
  bool Overlaps(double a, double b) const { return t <= std::max(a, b) && endT >= std::min(a, b); }
};

class ConvexPolygon {
 public:
  ConvexPolygon(std::vector<double> xs, std::vector<double> ys);
  bool Contains(double px, double py) const;
  ConvexPolygon Flipped(FieldSymmetry sym, double length, double width) const;
  size_t Size() const { return m_xs.size(); }

 private:
  std::vector<double> m_xs;
  std::vector<double> m_ys;
};

struct RecoveryData {
  std::vector<frc::Translation2d> bumper;
  std::vector<ConvexPolygon> obstacles;
  double fieldLength = 16.541;
  double fieldWidth = 8.069;
  FieldSymmetry symmetry = FieldSymmetry::ROTATIONAL;
  std::vector<frc::Translation2d> roadmapNodes;
  std::vector<std::array<int, 2>> roadmapEdges;
  double maxVelocity = 3.0;
  double maxAcceleration = 4.0;
  double maxAngularVelocity = 6.0;
  double maxAngularAcceleration = 10.0;
  std::vector<double> mustHitTimes;

  RecoveryData Flipped() const;
};

/** A solved Mayhem trajectory (or one split segment of it). Immutable. */
class MayhemTrajectory {
 public:
  MayhemTrajectory(std::string name, std::vector<TrajectorySample> samples,
                   std::vector<TrajectoryEvent> events, std::vector<TerrainSpan> terrain,
                   std::vector<IntakeSpan> intake, std::vector<int> splits,
                   std::vector<double> waypointTimes, std::shared_ptr<const RecoveryData> recovery,
                   std::string inputHash, bool flipped);

  const std::string& Name() const { return m_name; }
  const std::vector<TrajectorySample>& Samples() const { return m_samples; }
  const std::vector<TrajectoryEvent>& Events() const { return m_events; }
  const std::vector<TerrainSpan>& Terrain() const { return m_terrain; }
  /** The rough-terrain span covering time t (weakest feedback wins where they overlap). */
  const TerrainSpan* TerrainAt(double t) const;
  const std::vector<IntakeSpan>& IntakeSpans() const { return m_intake; }
  bool IntakeExtendedAt(double t) const;
  bool IntakeExtendedBetween(double a, double b) const;
  const RecoveryData* Recovery() const { return m_recovery.get(); }
  const std::string& InputHash() const { return m_inputHash; }
  bool IsFlipped() const { return m_flipped; }
  const std::vector<double>& WaypointTimes() const { return m_waypointTimes; }
  double TotalTime() const { return m_samples.back().t; }
  const TrajectorySample& InitialSample() const { return m_samples.front(); }
  const TrajectorySample& FinalSample() const { return m_samples.back(); }
  frc::Pose2d InitialPose() const { return InitialSample().GetPose(); }
  frc::Pose2d FinalPose() const { return FinalSample().GetPose(); }

  /** Interpolated state at time t (clamped to the trajectory's time range). */
  TrajectorySample SampleAt(double t) const;

  int SegmentCount() const { return static_cast<int>(m_splits.size()) + 1; }
  /** Split segment i as its own trajectory with time starting at 0. */
  std::shared_ptr<const MayhemTrajectory> Segment(int i) const;
  /** The red-alliance version of this trajectory (computed once, then cached). */
  std::shared_ptr<const MayhemTrajectory> Flipped() const;

 private:
  std::string m_name;
  std::vector<TrajectorySample> m_samples;
  std::vector<TrajectoryEvent> m_events;
  std::vector<TerrainSpan> m_terrain;
  std::vector<IntakeSpan> m_intake;
  std::vector<int> m_splits;
  std::vector<double> m_waypointTimes;
  std::shared_ptr<const RecoveryData> m_recovery;
  std::string m_inputHash;
  bool m_flipped;
  mutable std::shared_ptr<const MayhemTrajectory> m_flippedCache;
};

using TrajectoryPtr = std::shared_ptr<const MayhemTrajectory>;

/** Loads .mtraj files written by the Mayhem app (deploy/mayhem/<name>.mtraj). */
namespace trajectory_loader {
inline constexpr const char* kDeploySubdir = "mayhem";
inline constexpr const char* kExtension = ".mtraj";
inline constexpr int kSupportedFormat = 1;

/** Loads deploy/mayhem/<name>.mtraj, caching the result. Throws std::invalid_argument on errors. */
TrajectoryPtr Load(const std::string& name);
/** Loads a file. Throws std::invalid_argument on errors. */
TrajectoryPtr LoadFile(const std::string& path);
TrajectoryPtr Parse(const std::string& json);
void ClearCache();
/** The deploy directory (ROBOT_DEPLOY_DIR env var, else frc::filesystem::GetDeployDirectory()). */
std::string DeployDirectory();
}  // namespace trajectory_loader

}  // namespace mayhem
