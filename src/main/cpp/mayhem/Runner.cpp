// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "mayhem/Runner.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <numbers>

#include <frc/MathUtil.h>

namespace mayhem {

namespace {
double AngleModulus(double a) { return frc::AngleModulus(units::radian_t{a}).value(); }
double Clamp(double v, double lo, double hi) { return std::max(lo, std::min(hi, v)); }
double ToRadians(double deg) { return deg * std::numbers::pi / 180.0; }
double ToDegrees(double rad) { return rad * 180.0 / std::numbers::pi; }
}  // namespace

// ------------------------------------------------------------------------------------- follower
DriveCommand HolonomicFollower::Calculate(const TrajectorySample& ref, const frc::Pose2d& pose,
                                          const std::optional<frc::ChassisSpeeds>& measured,
                                          double rate, double feedbackScale) const {
  const double ex = ref.x - pose.X().value();
  const double ey = ref.y - pose.Y().value();
  const double eth = AngleModulus(ref.heading - pose.Rotation().Radians().value());

  double fbx = m_cfg.translationKp * ex;
  double fby = m_cfg.translationKp * ey;
  if (measured && m_cfg.translationKv != 0) {
    fbx += m_cfg.translationKv * (ref.vx * rate - measured->vx.value());
    fby += m_cfg.translationKv * (ref.vy * rate - measured->vy.value());
  }
  const double fbn = std::hypot(fbx, fby);
  if (fbn > m_cfg.maxFeedbackVelocity) {
    fbx *= m_cfg.maxFeedbackVelocity / fbn;
    fby *= m_cfg.maxFeedbackVelocity / fbn;
  }
  double fbw = Clamp(m_cfg.rotationKp * eth, -m_cfg.maxFeedbackOmega, m_cfg.maxFeedbackOmega);

  fbx *= feedbackScale;
  fby *= feedbackScale;
  fbw *= feedbackScale;

  double vx = ref.vx * rate + fbx;
  double vy = ref.vy * rate + fby;
  const double n = std::hypot(vx, vy);
  if (n > m_cfg.maxVelocity) {
    vx *= m_cfg.maxVelocity / n;
    vy *= m_cfg.maxVelocity / n;
  }
  const double w = Clamp(ref.omega * rate + fbw, -m_cfg.maxOmega, m_cfg.maxOmega);

  DriveCommand dc;
  dc.modules = m_cfg.modules;
  if (m_cfg.useForceFeedforward && ref.nForces == m_cfg.modules) {
    const double r2 = rate * rate;
    for (int i = 0; i < m_cfg.modules && i < kMaxModules; i++) {
      dc.wheelForceX[i] = ref.fx[i] * r2;
      dc.wheelForceY[i] = ref.fy[i] * r2;
    }
  }
  dc.fieldSpeeds =
      frc::ChassisSpeeds{units::meters_per_second_t{vx}, units::meters_per_second_t{vy},
                         units::radians_per_second_t{w}};
  return dc;
}

double HolonomicFollower::PositionError(const TrajectorySample& ref, const frc::Pose2d& pose) {
  return std::hypot(ref.x - pose.X().value(), ref.y - pose.Y().value());
}

double HolonomicFollower::HeadingError(const TrajectorySample& ref, const frc::Pose2d& pose) {
  return std::abs(AngleModulus(ref.heading - pose.Rotation().Radians().value()));
}

// ------------------------------------------------------------------------------------ detectors
void CollisionDetector::Reset() {
  m_overCount = 0;
  m_lastHitTime = -std::numeric_limits<double>::infinity();
}

bool CollisionDetector::Update(double now, double posError, double headingError, double accelG,
                               double expectedAccelG) {
  // A traction-limited swerve can legitimately pull >1 g, so only acceleration beyond what the
  // plan asked for counts as a hit.
  const bool spike = !std::isnan(accelG) && accelG - expectedAccelG >= m_cfg.accelSpikeG;
  if (spike) {
    m_lastHitTime = now;
  }
  const bool over =
      posError >= m_cfg.bridgeTriggerError || headingError >= m_cfg.bridgeTriggerHeading;
  m_overCount = over ? m_overCount + 1 : 0;
  const bool sustained = m_overCount >= m_cfg.persistenceLoops;
  if (sustained) {
    m_lastHitTime = now;
  }
  // a spike alone only matters if it actually pushed us off (some error)
  return sustained || (spike && posError >= m_cfg.dilationStartError);
}

void BeachDetector::Reset() {
  m_beached = false;
  m_tiltSince = kNaN;
  m_flatSince = kNaN;
}

double BeachDetector::TiltDegrees(double pitchDeg, double rollDeg) {
  const double p = ToRadians(pitchDeg);
  const double r = ToRadians(rollDeg);
  const double c = std::max(-1.0, std::min(1.0, std::cos(p) * std::cos(r)));
  return ToDegrees(std::acos(c));
}

frc::Translation2d BeachDetector::Downhill(double pitchDeg, double rollDeg,
                                           const frc::Rotation2d& heading) {
  const double p = ToRadians(pitchDeg);
  const double r = ToRadians(rollDeg);
  const double fwd = std::sin(p) * std::cos(r);
  const double left = -std::sin(r);
  const double n = std::hypot(fwd, left);
  if (std::isnan(n) || n < 1e-9) {
    return frc::Translation2d{};
  }
  return frc::Translation2d{units::meter_t{fwd / n}, units::meter_t{left / n}}.RotateBy(heading);
}

bool BeachDetector::Update(double now, double pitchDeg, double rollDeg) {
  m_tilt = TiltDegrees(pitchDeg, rollDeg);
  if (std::isnan(m_tilt)) {
    Reset();
    return false;
  }
  if (!m_beached) {
    if (m_tilt >= m_cfg.tiltThresholdDegrees) {
      if (std::isnan(m_tiltSince)) {
        m_tiltSince = now;
      }
      if (now - m_tiltSince >= m_cfg.detectSeconds) {
        m_beached = true;
        m_flatSince = kNaN;
      }
    } else {
      m_tiltSince = kNaN;
    }
  } else if (m_tilt <= m_cfg.flatThresholdDegrees) {
    if (std::isnan(m_flatSince)) {
      m_flatSince = now;
    }
    if (now - m_flatSince >= m_cfg.flatSeconds) {
      Reset();
    }
  } else {
    m_flatSince = kNaN;
  }
  return m_beached;
}

// --------------------------------------------------------------------------------------- bridge
Quintic::Quintic(double p0, double v0, double a0, double p1, double v1, double a1, double T)
    : duration{T} {
  const double T2 = T * T;
  const double T3 = T2 * T;
  c0 = p0;
  c1 = v0;
  c2 = 0.5 * a0;
  c3 = (20 * (p1 - p0) - (8 * v1 + 12 * v0) * T - (3 * a0 - a1) * T2) / (2 * T3);
  c4 = (30 * (p0 - p1) + (14 * v1 + 16 * v0) * T + (3 * a0 - 2 * a1) * T2) / (2 * T3 * T);
  c5 = (12 * (p1 - p0) - 6 * (v1 + v0) * T - (a0 - a1) * T2) / (2 * T3 * T2);
}

Bridge::Bridge(std::vector<Segment> segments, double joinTime_)
    : joinTime{joinTime_}, m_segments{std::move(segments)} {
  for (const auto& s : m_segments) {
    m_duration += s.duration;
  }
}

TrajectorySample Bridge::SampleAt(double tau) const {
  tau = std::max(0.0, std::min(tau, m_duration));
  double acc = 0;
  const Segment* seg = &m_segments.back();
  double local = seg->duration;
  for (const auto& s : m_segments) {
    if (tau <= acc + s.duration) {
      seg = &s;
      local = tau - acc;
      break;
    }
    acc += s.duration;
  }
  TrajectorySample out;
  out.t = tau;
  out.x = seg->x.P(local);
  out.y = seg->y.P(local);
  out.heading = seg->heading.P(local);
  out.vx = seg->x.V(local);
  out.vy = seg->y.V(local);
  out.omega = seg->heading.V(local);
  out.ax = seg->x.A(local);
  out.ay = seg->y.A(local);
  out.alpha = seg->heading.A(local);
  return out;
}

std::vector<frc::Pose2d> Bridge::Poses(int n) const {
  std::vector<frc::Pose2d> out;
  out.reserve(n + 1);
  for (int i = 0; i <= n; i++) {
    out.push_back(SampleAt(m_duration * i / n).GetPose());
  }
  return out;
}

// -------------------------------------------------------------------------------------- planner
namespace {
constexpr double kSkipCheckStep = 0.05;
constexpr double kPathAccelHeadroom = 1.3;

struct Limits {
  double v, a, w, al;
};

struct PlanState {
  double x, y, th, vx, vy, w, ax, ay, al;
  static PlanState Of(const TrajectorySample& s) {
    return {s.x, s.y, s.heading, s.vx, s.vy, s.omega, s.ax, s.ay, s.alpha};
  }
};

Bridge::Segment MakeSegment(const PlanState& s, const PlanState& e, double T) {
  return Bridge::Segment{Quintic{s.x, s.vx, s.ax, e.x, e.vx, e.ax, T},
                         Quintic{s.y, s.vy, s.ay, e.y, e.vy, e.ay, T},
                         Quintic{s.th, s.w, s.al, e.th, e.w, e.al, T}, T};
}

bool Feasible(const PlanState& s, const PlanState& e, double T, const Limits& lim, double vCap,
              double aCap, double wCap) {
  const Bridge::Segment seg = MakeSegment(s, e, T);
  constexpr int m = 24;
  const double alCap = std::max(lim.al, std::max(std::abs(s.al), std::abs(e.al))) * 1.01;
  for (int i = 0; i <= m; i++) {
    const double t = T * i / m;
    const double vx = seg.x.V(t);
    const double vy = seg.y.V(t);
    if (vx * vx + vy * vy > vCap * vCap) {
      return false;
    }
    const double ax = seg.x.A(t);
    const double ay = seg.y.A(t);
    if (ax * ax + ay * ay > aCap * aCap) {
      return false;
    }
    if (std::abs(seg.heading.V(t)) > wCap || std::abs(seg.heading.A(t)) > alCap) {
      return false;
    }
    // combined wheel-speed budget: translation + rotation share the same wheels
    if (std::hypot(vx, vy) / vCap + std::abs(seg.heading.V(t)) / (wCap * 2) > 1.25) {
      return false;
    }
  }
  return true;
}

std::optional<Bridge::Segment> FastestSegment(const PlanState& s, const PlanState& e,
                                              const Limits& lim, double pathV, double pathA) {
  const double dist = std::hypot(e.x - s.x, e.y - s.y);
  const double dth = std::abs(e.th - s.th);
  const double tLow = std::max(0.05, std::max(dist / lim.v, dth / lim.w));
  // velocities at the ends may legitimately exceed the limit (robot was shoved)
  const double vCap =
      std::max(std::max(lim.v, pathV), std::max(std::hypot(s.vx, s.vy), std::hypot(e.vx, e.vy))) *
      1.01;
  const double aCap = std::max(std::max(lim.a, pathA * kPathAccelHeadroom),
                               std::max(std::hypot(s.ax, s.ay), std::hypot(e.ax, e.ay))) *
                      1.01;
  const double wCap = std::max(lim.w, std::max(std::abs(s.w), std::abs(e.w))) * 1.01;

  double prev = -1;
  double feasibleT = -1;
  double Tk = tLow;
  for (int k = 0; k < 28; k++, Tk *= 1.2) {
    if (Feasible(s, e, Tk, lim, vCap, aCap, wCap)) {
      feasibleT = Tk;
      break;
    }
    prev = Tk;
    if (Tk > 8) {
      break;
    }
  }
  if (feasibleT < 0) {
    return std::nullopt;
  }
  if (prev > 0) {
    double a = prev;
    double b = feasibleT;
    for (int i = 0; i < 10; i++) {
      const double mid = 0.5 * (a + b);
      if (Feasible(s, e, mid, lim, vCap, aCap, wCap)) {
        b = mid;
      } else {
        a = mid;
      }
    }
    feasibleT = b;
  }
  return MakeSegment(s, e, feasibleT);
}

double DistanceToSegment(double px, double py, double ax, double ay, double bx, double by) {
  const double dx = bx - ax;
  const double dy = by - ay;
  const double len2 = dx * dx + dy * dy;
  const double u = len2 < 1e-9 ? 0 : Clamp(((px - ax) * dx + (py - ay) * dy) / len2, 0, 1);
  return std::hypot(px - (ax + u * dx), py - (ay + u * dy));
}
}  // namespace

bool BridgePlanner::CutsOffPath(const MayhemTrajectory& traj, double tNow, double tj,
                                const frc::Pose2d& pose, const TrajectorySample& target,
                                double offPath) const {
  for (double t = tNow; t < tj; t += kSkipCheckStep) {
    const TrajectorySample s = traj.SampleAt(t);
    const double allowance = m_cfg.maxJoinShortcut + offPath * (tj - t) / (tj - tNow);
    if (DistanceToSegment(s.x, s.y, pose.X().value(), pose.Y().value(), target.x, target.y) >
        allowance) {
      return true;
    }
  }
  return false;
}

BridgePlanner::Result BridgePlanner::Plan(
    const MayhemTrajectory& traj, double tNow, const frc::Pose2d& pose,
    const std::optional<frc::ChassisSpeeds>& fieldSpeeds) const {
  const auto t0 = std::chrono::steady_clock::now();
  const RecoveryData* rec = traj.Recovery();
  if (rec == nullptr) {
    return {};
  }
  const Limits lim{rec->maxVelocity * m_cfg.limitScale, rec->maxAcceleration * m_cfg.limitScale,
                   rec->maxAngularVelocity * m_cfg.limitScale,
                   rec->maxAngularAcceleration * m_cfg.limitScale};
  const double T = traj.TotalTime();

  // join window bounded by must-hit points
  double prevMust = 0;
  double nextMust = T;
  for (double m : rec->mustHitTimes) {
    if (m <= tNow + 1e-6) {
      prevMust = std::max(prevMust, m);
    } else {
      nextMust = std::min(nextMust, m);
    }
  }
  const double lo = std::max(prevMust, tNow - m_cfg.maxJoinBehind);
  double hi = std::min(nextMust, std::min(T, tNow + m_cfg.maxJoinLookahead));
  if (hi < lo) {
    hi = lo;
  }
  const int n = std::max(2, m_cfg.joinCandidates);
  const double vx0 = fieldSpeeds ? fieldSpeeds->vx.value() : 0;
  const double vy0 = fieldSpeeds ? fieldSpeeds->vy.value() : 0;
  const double w0 = fieldSpeeds ? fieldSpeeds->omega.value() : 0;
  const double th0Raw = pose.Rotation().Radians().value();

  // fastest dynamically-feasible bridge per join time; keep the earliest arrival at the end
  std::optional<Bridge> best;
  double bestCost = std::numeric_limits<double>::infinity();
  int tried = 0;
  const TrajectorySample here = traj.SampleAt(tNow);
  const double offPath = std::hypot(pose.X().value() - here.x, pose.Y().value() - here.y);
  // the extra last candidate is the clock itself, which can never skip anything
  for (int i = 0; i <= n; i++) {
    const double tj = i < n ? lo + (hi - lo) * i / (n - 1) : std::min(tNow, hi);
    const TrajectorySample target = traj.SampleAt(tj);
    if (CutsOffPath(traj, tNow, tj, pose, target, offPath)) {
      continue;
    }
    tried++;
    const double th0 = target.heading - AngleModulus(target.heading - th0Raw);
    // carry on from the plan's acceleration: starting from zero mid-curve forces a wide swing back
    PlanState start{pose.X().value(), pose.Y().value(), th0,       vx0, vy0, w0,
                    here.ax,          here.ay,          here.alpha};
    // the bridge replaces this stretch of path, so whatever the plan does there is achievable
    double peakV = 0;
    double peakA = 0;
    for (double t = std::min(tj, tNow); t <= std::max(tj, tNow); t += kSkipCheckStep) {
      const TrajectorySample s = traj.SampleAt(t);
      peakV = std::max(peakV, s.Speed());
      peakA = std::max(peakA, std::hypot(s.ax, s.ay));
    }
    auto seg = FastestSegment(start, PlanState::Of(target), lim, peakV, peakA);
    if (!seg) {
      // hit hard enough that the plan's acceleration no longer applies
      start = PlanState{pose.X().value(), pose.Y().value(), th0, vx0, vy0, w0, 0, 0, 0};
      seg = FastestSegment(start, PlanState::Of(target), lim, peakV, peakA);
    }
    if (seg && seg->duration - tj < bestCost) {
      bestCost = seg->duration - tj;
      best.emplace(std::vector<Bridge::Segment>{*seg}, tj);
    }
  }
  Result r;
  r.bridge = std::move(best);
  r.candidatesTried = tried;
  r.planSeconds = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
  return r;
}

// --------------------------------------------------------------------------------------- runner
const char* TrajectoryRunner::StateName(State s) {
  switch (s) {
    case State::IDLE:
      return "IDLE";
    case State::FOLLOWING:
      return "FOLLOWING";
    case State::BRIDGING:
      return "BRIDGING";
    case State::UNBEACHING:
      return "UNBEACHING";
    case State::SETTLING:
      return "SETTLING";
    case State::FINISHED:
      return "FINISHED";
  }
  return "?";
}

TrajectoryRunner::TrajectoryRunner(TrajectoryPtr traj, FollowerConfig fc, RecoveryConfig rc,
                                   EventListener listener)
    : m_traj{std::move(traj)},
      m_follower{fc},
      m_rc{rc},
      m_planner{m_rc},
      m_detector{m_rc},
      m_listener{std::move(listener)},
      m_events{m_traj->Events()},
      m_fired(m_events.size(), false),
      m_zoneEnded(m_events.size(), false),
      m_reference{m_traj->InitialSample()} {}

TrajectoryRunner& TrajectoryRunner::WithUnbeach(std::optional<UnbeachConfig> config) {
  m_unbeachConfig = config ? *config : UnbeachConfig::Disabled();
  m_beach = BeachDetector{m_unbeachConfig};
  return *this;
}

void TrajectoryRunner::Start(double now) {
  m_state = State::FOLLOWING;
  m_t = 0;
  m_lastNow = now;
  m_rate = 1;
  m_bridge.reset();
  m_terrain = nullptr;
  m_terrainExitTime = -std::numeric_limits<double>::infinity();
  m_detector.Reset();
  m_beach.Reset();
  m_unbeachTarget.reset();
  m_unbeachCooldownUntil = -std::numeric_limits<double>::infinity();
  std::fill(m_fired.begin(), m_fired.end(), false);
  std::fill(m_zoneEnded.begin(), m_zoneEnded.end(), false);
  m_pendingAtJoin.clear();
  FireCrossing(-1e-9, 0);
}

void TrajectoryRunner::Stop() {
  if (m_state != State::FINISHED && m_state != State::IDLE) {
    CloseAllZones();
  }
  m_state = State::FINISHED;
}

DriveCommand TrajectoryRunner::Update(double now, const frc::Pose2d& pose,
                                      const std::optional<frc::ChassisSpeeds>& fieldSpeeds,
                                      double accelG, double pitchDegrees, double rollDegrees) {
  m_pitchDeg = pitchDegrees;
  m_rollDeg = rollDegrees;
  const double dt = std::max(0.0, std::min(now - m_lastNow, 0.1));
  m_lastNow = now;
  m_lastNowForBoost = now;
  switch (m_state) {
    case State::IDLE:
    case State::FINISHED:
      return DriveCommand::Stop(m_follower.Config().modules);
    case State::BRIDGING:
      return UpdateBridging(now, dt, pose, fieldSpeeds, accelG);
    case State::UNBEACHING:
      return UpdateUnbeaching(now, dt, pose, fieldSpeeds, accelG);
    case State::SETTLING:
      return UpdateSettling(now, pose, fieldSpeeds, accelG);
    case State::FOLLOWING:
    default:
      return UpdateFollowing(now, dt, pose, fieldSpeeds, accelG);
  }
}

namespace {
double Ramp(double err, double start, double stop) {
  if (err <= start) {
    return 1;
  }
  if (err >= stop) {
    return 0;
  }
  return 1 - (err - start) / (stop - start);
}
double AccelG(const TrajectorySample& s) { return std::hypot(s.ax, s.ay) / 9.81; }
}  // namespace

DriveCommand TrajectoryRunner::UpdateFollowing(double now, double dt, const frc::Pose2d& pose,
                                               const std::optional<frc::ChassisSpeeds>& v,
                                               double accelG) {
  const TrajectorySample ref = m_traj->SampleAt(m_t);
  Measure(ref, pose);
  const TerrainSpan* span = m_traj->TerrainAt(m_t);
  if (span == nullptr && m_terrain != nullptr) {
    m_terrainExitTime = now;
  }
  m_terrain = span;
  if (span != nullptr) {
    m_beach.Reset();  // a marked bump tilts the robot on purpose
    return UpdateTerrain(dt, *span, ref, pose, v);
  }
  const bool grace = now - m_terrainExitTime <= m_rc.terrainGraceSeconds;
  if (grace) {
    m_detector.Reset();
    m_beach.Reset();
  } else if (CheckBeached(now, pose)) {
    return UpdateUnbeaching(now, 0, pose, v, accelG);
  } else if (m_rc.enabled &&
             m_detector.Update(now, m_posError, m_headingError, accelG, AccelG(ref)) &&
             TryPlan(now, pose, v)) {
    return UpdateBridging(now, 0, pose, v, accelG);
  }
  m_rate = m_rc.enabled
               ? std::min(Ramp(m_posError, m_rc.dilationStartError, m_rc.dilationStopError),
                          Ramp(m_headingError, m_rc.dilationStartHeading, m_rc.dilationStopHeading))
               : 1.0;
  const double tNew = std::min(m_t + m_rate * dt, m_traj->TotalTime());
  FireCrossing(m_t, tNew);
  m_t = tNew;
  m_reference = m_traj->SampleAt(m_t);
  if (m_t >= m_traj->TotalTime()) {
    m_state = State::SETTLING;
    m_settleStart = now;
  }
  return m_follower.Calculate(m_reference, pose, v, m_rate);
}

DriveCommand TrajectoryRunner::UpdateTerrain(double dt, const TerrainSpan& span,
                                             const TrajectorySample& ref, const frc::Pose2d& pose,
                                             const std::optional<frc::ChassisSpeeds>& v) {
  m_detector.Reset();
  const double speed = ref.Speed();
  double lag = 0;
  if (speed > 1e-3) {
    // along-track offset in seconds of path; negative when the robot is behind the reference
    const double along =
        ((pose.X().value() - ref.x) * ref.vx + (pose.Y().value() - ref.y) * ref.vy) / speed;
    lag = along / std::max(speed, 0.3);
  }
  m_rate = std::max(0.0, std::min(1.0, 1 + m_rc.terrainClockGain * lag));
  const double tNew = std::min(m_t + m_rate * dt, m_traj->TotalTime());
  FireCrossing(m_t, tNew);
  m_t = tNew;
  m_reference = m_traj->SampleAt(m_t);
  if (m_t >= m_traj->TotalTime()) {
    m_state = State::SETTLING;
    m_settleStart = m_lastNow;
  }
  return m_follower.Calculate(m_reference, pose, v, 1.0, span.feedbackScale);
}

DriveCommand TrajectoryRunner::UpdateSettling(double now, const frc::Pose2d& pose,
                                              const std::optional<frc::ChassisSpeeds>& v,
                                              double accelG) {
  const TrajectorySample& end = m_traj->FinalSample();
  Measure(end, pose);
  const bool endsMoving = end.Speed() > 0.05 || std::abs(end.omega) > 0.05;
  if (m_rc.enabled && !endsMoving &&
      m_detector.Update(now, m_posError, m_headingError, accelG, 0.0) && TryPlan(now, pose, v)) {
    return UpdateBridging(now, 0, pose, v, accelG);
  }
  const bool settled =
      m_posError <= m_rc.endTolerance && m_headingError <= m_rc.endHeadingTolerance;
  if (endsMoving || settled || now - m_settleStart >= m_rc.endTimeout) {
    Finish();
  }
  m_reference = end;
  m_rate = 1;
  return m_follower.Calculate(end, pose, v, 1.0);
}

DriveCommand TrajectoryRunner::UpdateBridging(double now, double dt, const frc::Pose2d& pose,
                                              const std::optional<frc::ChassisSpeeds>& v,
                                              double accelG) {
  if (CheckBeached(now, pose)) {
    // stuck on fuel mid-bridge: a straight bridge cannot help, so drop it and unbeach first
    m_bridge.reset();
    return UpdateUnbeaching(now, 0, pose, v, accelG);
  }
  m_tau += dt;
  TrajectorySample bs = m_bridge->SampleAt(m_tau);
  Measure(bs, pose);
  m_detector.Update(now, m_posError, m_headingError, accelG, AccelG(bs));
  if (m_posError > m_rc.replanError && now - m_lastPlanTime >= m_rc.minReplanInterval) {
    if (TryPlan(now, pose, v)) {
      bs = m_bridge->SampleAt(0);
      Measure(bs, pose);
    }
  }
  m_reference = bs;
  m_rate = 1;
  DriveCommand cmd = m_follower.Calculate(bs, pose, v, 1.0);
  if (m_tau >= m_bridge->Duration()) {
    // rejoined
    const double tj = m_bridge->joinTime;
    m_bridge.reset();
    for (size_t i : m_pendingAtJoin) {
      Fire(i);
    }
    m_pendingAtJoin.clear();
    // close zones whose end we jumped over
    for (size_t i = 0; i < m_events.size(); i++) {
      const auto& e = m_events[i];
      if (e.IsZone() && m_fired[i] && !m_zoneEnded[i] && e.endT <= tj) {
        m_zoneEnded[i] = true;
        if (m_listener.onZoneEnd)
          m_listener.onZoneEnd(e);
      }
    }
    m_t = tj;
    m_detector.Reset();
    if (m_t >= m_traj->TotalTime()) {
      m_state = State::SETTLING;
      m_settleStart = now;
    } else {
      m_state = State::FOLLOWING;
    }
  }
  return cmd;
}

bool TrajectoryRunner::CheckBeached(double now, const frc::Pose2d& pose) {
  if (!m_unbeachConfig.enabled || now < m_unbeachCooldownUntil ||
      !m_beach.Update(now, m_pitchDeg, m_rollDeg)) {
    return false;
  }
  m_unbeachCount++;
  m_unbeachStart = now;
  m_unbeachTarget.reset();
  m_unbeachRetargetAt = now;
  m_state = State::UNBEACHING;
  return true;
}

DriveCommand TrajectoryRunner::UpdateUnbeaching(double now, double dt, const frc::Pose2d& pose,
                                                const std::optional<frc::ChassisSpeeds>& v,
                                                double accelG) {
  Measure(m_reference, pose);
  const UnbeachConfig& uc = m_unbeachConfig;
  const bool flat = !m_beach.Update(now, m_pitchDeg, m_rollDeg);
  if (flat || now - m_unbeachStart >= uc.maxSeconds) {
    m_state = State::FOLLOWING;
    m_unbeachTarget.reset();
    m_unbeachCooldownUntil = now + uc.cooldownSeconds;
    m_beach.Reset();
    m_detector.Reset();
    m_rate = 0;
    return m_follower.Calculate(m_reference, pose, v, 0);
  }
  const frc::Translation2d here = pose.Translation();
  const bool reached =
      m_unbeachTarget && here.Distance(*m_unbeachTarget).value() <= uc.reachTolerance;
  if (!m_unbeachTarget || reached || now >= m_unbeachRetargetAt) {
    const frc::Translation2d dir = BeachDetector::Downhill(m_pitchDeg, m_rollDeg, pose.Rotation());
    if (dir.Norm().value() > 0) {
      m_unbeachTarget = ClampToField(here + dir * uc.escapeDistance);
      m_unbeachRetargetAt = now + uc.retargetSeconds;
    }
  }
  m_rate = 0;
  const int nMod = m_follower.Config().modules;
  if (!m_unbeachTarget) {
    return DriveCommand::Stop(nMod);
  }
  const frc::Translation2d delta = *m_unbeachTarget - here;
  const double dist = delta.Norm().value();
  if (dist < 1e-6) {
    return DriveCommand::Stop(nMod);
  }
  const double speed = std::max(uc.minEscapeSpeed, std::min(uc.escapeSpeed, uc.escapeKp * dist));
  // hold heading: only translate off the pile
  DriveCommand dc = DriveCommand::Stop(nMod);
  dc.fieldSpeeds =
      frc::ChassisSpeeds{units::meters_per_second_t{delta.X().value() / dist * speed},
                         units::meters_per_second_t{delta.Y().value() / dist * speed}, 0_rad_per_s};
  return dc;
}

frc::Translation2d TrajectoryRunner::ClampToField(const frc::Translation2d& p) const {
  const RecoveryData* rd = m_traj->Recovery();
  if (rd == nullptr || rd->fieldLength <= 0 || rd->fieldWidth <= 0) {
    return p;
  }
  const double m = m_unbeachConfig.fieldMargin;
  return frc::Translation2d{
      units::meter_t{std::max(m, std::min(rd->fieldLength - m, p.X().value()))},
      units::meter_t{std::max(m, std::min(rd->fieldWidth - m, p.Y().value()))}};
}

bool TrajectoryRunner::Pending(size_t i) const {
  return std::find(m_pendingAtJoin.begin(), m_pendingAtJoin.end(), i) != m_pendingAtJoin.end();
}

bool TrajectoryRunner::TryPlan(double now, const frc::Pose2d& pose,
                               const std::optional<frc::ChassisSpeeds>& v) {
  if (now - m_lastPlanTime < m_rc.minReplanInterval) {
    return false;
  }
  m_lastPlanTime = now;
  // the clock is frozen during a bridge, so a replan starts from how far the bridge has come
  const double from = !m_bridge ? m_t
                                : m_bridgeFrom + (m_bridge->joinTime - m_bridgeFrom) *
                                                     std::min(1.0, m_tau / m_bridge->Duration());
  auto res = m_planner.Plan(*m_traj, from, pose, v);
  m_lastPlanSeconds = res.planSeconds;
  if (!res.bridge) {
    return false;
  }
  Bridge b = std::move(*res.bridge);
  m_bridgesPlanned++;
  // events between the current clock and the join time: apply their recovery policy
  if (b.joinTime > m_t) {
    for (size_t i = 0; i < m_events.size(); i++) {
      const auto& e = m_events[i];
      if (m_fired[i] || Pending(i) || e.t <= m_t || e.t > b.joinTime) {
        continue;
      }
      switch (e.policy) {
        case TrajectoryEvent::RecoveryPolicy::FIRE_IMMEDIATELY:
          Fire(i);
          break;
        case TrajectoryEvent::RecoveryPolicy::SKIP:
          m_fired[i] = true;
          m_zoneEnded[i] = !e.IsZone() || e.endT <= b.joinTime;
          break;
        case TrajectoryEvent::RecoveryPolicy::FIRE_AT_JOIN:
        default:
          m_pendingAtJoin.push_back(i);
          break;
      }
    }
  }
  m_bridge.emplace(std::move(b));
  m_bridgeFrom = from;
  m_tau = 0;
  m_state = State::BRIDGING;
  return true;
}

void TrajectoryRunner::FireCrossing(double from, double to) {
  for (size_t i = 0; i < m_events.size(); i++) {
    const auto& e = m_events[i];
    if (!m_fired[i] && e.t > from && e.t <= to) {
      Fire(i);
    }
    if (e.IsZone() && m_fired[i] && !m_zoneEnded[i] && e.endT > from && e.endT <= to) {
      m_zoneEnded[i] = true;
      if (m_listener.onZoneEnd)
        m_listener.onZoneEnd(e);
    }
  }
}

void TrajectoryRunner::Fire(size_t i) {
  if (m_fired[i]) {
    return;
  }
  m_fired[i] = true;
  const auto& e = m_events[i];
  if (m_listener.onEvent)
    m_listener.onEvent(e);
  if (e.IsZone() && e.endT <= e.t) {
    m_zoneEnded[i] = true;
    if (m_listener.onZoneEnd)
      m_listener.onZoneEnd(e);
  }
}

void TrajectoryRunner::Finish() {
  // anything left at the very end (e.g. markers at T) fires now
  FireCrossing(m_t - 1e-9, m_traj->TotalTime() + 1e-6);
  for (size_t i : m_pendingAtJoin) {
    Fire(i);
  }
  m_pendingAtJoin.clear();
  CloseAllZones();
  m_state = State::FINISHED;
}

void TrajectoryRunner::CloseAllZones() {
  for (size_t i = 0; i < m_events.size(); i++) {
    if (m_events[i].IsZone() && m_fired[i] && !m_zoneEnded[i]) {
      m_zoneEnded[i] = true;
      if (m_listener.onZoneEnd)
        m_listener.onZoneEnd(m_events[i]);
    }
  }
}

void TrajectoryRunner::Measure(const TrajectorySample& ref, const frc::Pose2d& pose) {
  m_posError = HolonomicFollower::PositionError(ref, pose);
  m_headingError = HolonomicFollower::HeadingError(ref, pose);
}

bool TrajectoryRunner::IntakeExtended() const {
  if (m_state == State::BRIDGING && m_bridge) {
    return m_traj->IntakeExtendedBetween(m_bridgeFrom, m_bridge->joinTime);
  }
  return (m_state == State::FOLLOWING || m_state == State::SETTLING) &&
         m_traj->IntakeExtendedAt(m_t);
}

bool TrajectoryRunner::VisionBoostActive() const {
  return m_detector.VisionBoostActive(m_lastNowForBoost) ||
         m_lastNowForBoost - m_terrainExitTime <= m_rc.visionBoostSeconds;
}

}  // namespace mayhem
