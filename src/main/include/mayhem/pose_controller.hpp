#pragma once
/*
 * Portions ported from BLine-Lib (https://github.com/edanliahovetsky/BLine-Lib):
 *
 * BSD 3-Clause License
 *
 * Copyright (c) 2025, Edan Liahovetsky
 * Copyright (c) 2025, Rebel Robotics (FRC 2638)
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <string_view>

namespace mayhem {
// All values use meters, radians, seconds; velocities are field-relative.
struct Pose { double x = 0, y = 0, heading = 0; };
struct Velocity { double vx = 0, vy = 0, omega = 0; };
struct Constraints {
  // BLine-Lib defaults: 4.5 m/s, 12 m/s^2, 720 deg/s, 1500 deg/s^2; README gains 5 / 3.
  double maxVelocity = 4.5, maxAcceleration = 12, maxAngularVelocity = 12.566370614359172;
  double maxAngularAcceleration = 26.179938779914941, translationKp = 5, rotationKp = 3;
  // Default intermediate handoff radius, cross-track gain, and speed floor (0 = off).
  double handoffRadius = .45, crossTrackKp = 2, minVelocity = 0;
  // Slow down before a slower step ahead (speed <= sqrt(cap^2 + 2 a d), d = distance to
  // its handoff point) instead of after its handoff. Needs per-anchor caps; off = BLine.
  bool anticipateSpeedCaps = false;
};
enum class HeadingMode { Fixed, PointAt, Motion, Free };
enum class MarkerKind { Rotation, Event };
// Rotation target or event trigger on the segment leading into its anchor,
// placed by projected progress t in [0, 1].
struct PathMarker {
  MarkerKind kind = MarkerKind::Event; double t = .5; double heading = 0;
  std::string_view key; bool profiled = true;
};
struct Goal {
  Pose pose;
  double translationTolerance = .03, rotationTolerance = 0.03490658503988659;
  double velocityTolerance = .15, settleTime = 0, handoffRadius = 0;
  HeadingMode headingMode = HeadingMode::Fixed;
  double aimX = 0, aimY = 0;
  bool stop = false, projectedHandoff = false, profiled = true;
  const PathMarker* markers = nullptr; std::size_t markerCount = 0;
  // Radians added to the Motion or PointAt heading (pi: face backward while moving).
  double headingOffset = 0;
};
struct ControlState {
  Velocity previous; double settled = 0;
  double sx = 0, sy = 0, rotHeading = 0, rotRemaining = NAN;
  std::size_t events = 0; bool started = false; double acceptedRemaining = INFINITY;
};
struct ControlResult {
  Velocity command; bool reached = false; bool valid = true; bool handoff = false;
  // Markers [eventsBegin, eventsEnd) of the active anchor were passed this tick.
  std::size_t eventsBegin = 0, eventsEnd = 0;
  double progress = 0, remaining = 0;
  // Distance to the active anchor, and |heading error| against the heading the
  // follower is steering to this tick (PointAt, Motion, markers included).
  double translationError = 0, rotationError = 0;
};
inline double Wrap(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }
// Path follower ported from BLine-Lib's FollowPath, RotationProgress and ChassisRateLimiter
// (BSD 3-Clause, Copyright (c) 2025 Edan Liahovetsky and Rebel Robotics (FRC 2638); see
// THIRD_PARTY_NOTICES.md). Mirrors follow() in app/src/suite/core/runtime.ts.
// anchors[0] is the active translation target; `final` means anchors[count-1] ends the
// path. No scheduler, vendor, network, file IO, allocation, or agent dependency.
// A dt above MaxDt() (a loop overrun) is integrated as MaxDt() instead of failing.
class PathFollower {
 public:
  static constexpr double kDefaultMaxDt = .1;
  // Below this distance to the target, Motion heading follows the segment direction.
  static constexpr double kMotionMinDistance = .1;
  // Ignored unless finite and positive.
  void SetMaxDt(double maxDt) { if (std::isfinite(maxDt) && maxDt > 0) maxDt_ = maxDt; }
  double MaxDt() const { return maxDt_; }
  void Reset(Pose pose, Velocity measured = {}) {
    state_ = {measured, 0, pose.x, pose.y, pose.heading, NAN, 0, true, INFINITY};
  }
  // caps (optional, `count` entries): max velocity of the step owning each anchor; used
  // only with c.anticipateSpeedCaps.
  ControlResult Calculate(Pose pose, Velocity measured, const Goal* const* anchors,
                          std::size_t count, bool final, const Constraints& c, double dt,
                          const double* caps = nullptr) {
    if (count == 0 || !anchors[0]) return Invalid();
    if (!state_.started) Reset(pose, state_.previous);
    const Goal& T = *anchors[0];
    const double nums[] = {pose.x, pose.y, pose.heading, measured.vx, measured.vy, measured.omega,
      state_.previous.vx, state_.previous.vy, state_.previous.omega, state_.settled, state_.sx,
      state_.sy, dt, c.maxVelocity, c.maxAcceleration, c.maxAngularVelocity,
      c.maxAngularAcceleration, c.translationKp, c.rotationKp, c.handoffRadius, c.crossTrackKp,
      c.minVelocity};
    for (double v : nums) if (!std::isfinite(v)) return Invalid();
    if (dt <= 0 || c.maxVelocity <= 0 || c.maxAcceleration <= 0 ||
        c.maxAngularVelocity <= 0 || c.maxAngularAcceleration <= 0 || c.translationKp <= 0 ||
        c.rotationKp <= 0 || c.handoffRadius <= 0 || c.crossTrackKp <= 0 || c.minVelocity < 0)
      return Invalid();
    dt = std::min(dt, maxDt_);
    for (std::size_t i = 0; i < count; ++i) {
      const Goal* g = anchors[i];
      if (!g) return Invalid();
      const double gv[] = {g->pose.x, g->pose.y, g->pose.heading, g->translationTolerance,
        g->rotationTolerance, g->velocityTolerance, g->settleTime, g->handoffRadius, g->headingOffset};
      for (double v : gv) if (!std::isfinite(v)) return Invalid();
      if (g->translationTolerance <= 0 || g->rotationTolerance <= 0 || g->velocityTolerance <= 0 ||
          g->settleTime < 0 || g->handoffRadius < 0 ||
          (g->headingMode == HeadingMode::PointAt && (!std::isfinite(g->aimX) || !std::isfinite(g->aimY))))
        return Invalid();
      for (std::size_t m = 0; m < g->markerCount; ++m)
        if (!std::isfinite(g->markers[m].t) ||
            (g->markers[m].kind == MarkerKind::Rotation && !std::isfinite(g->markers[m].heading)))
          return Invalid();
    }
    const double sx = state_.sx, sy = state_.sy;
    const double segX = T.pose.x - sx, segY = T.pose.y - sy, L = std::hypot(segX, segY);
    const double ux = L > 1e-9 ? segX / L : 0, uy = L > 1e-9 ? segY / L : 0;
    const double rx = pose.x - sx, ry = pose.y - sy;
    const double progress = L > 1e-9 ? std::clamp((rx * ux + ry * uy) / L, 0.0, 1.0) : 1;
    const double crossTrack = L > 1e-9 ? ux * ry - uy * rx : 0;
    const double dx = T.pose.x - pose.x, dy = T.pose.y - pose.y, distance = std::hypot(dx, dy);
    double tail = 0;
    for (std::size_t i = 1; i < count; ++i)
      tail += std::hypot(anchors[i]->pose.x - anchors[i - 1]->pose.x, anchors[i]->pose.y - anchors[i - 1]->pose.y);
    const bool isFinal = final && count == 1;
    const double remaining = final ? distance + tail : INFINITY;
    const double along = (1 - progress) * L + tail;
    const double radius = T.handoffRadius > 0 ? T.handoffRadius : c.handoffRadius;
    const bool handoff = !isFinal && (L <= 1e-9 || distance <= radius ||
      (T.projectedHandoff && L > 1e-9 && progress >= std::clamp(1 - radius / L, 0.0, 1.0)));
    ControlResult out;
    out.progress = progress; out.remaining = remaining;
    out.eventsBegin = state_.events;
    std::size_t events = state_.events;
    for (; events < T.markerCount; ++events)
      if (T.markers[events].kind == MarkerKind::Event && T.markers[events].t > progress && !handoff) break;
    out.eventsEnd = events;
    const double progressAlong = std::min(along, state_.acceptedRemaining);
    double fromHeading = state_.rotHeading;
    double fromRemaining = std::isfinite(state_.rotRemaining) ? state_.rotRemaining : along;
    for (std::size_t m = 0; m < T.markerCount; ++m)
      if (T.markers[m].kind == MarkerKind::Rotation && T.markers[m].t <= progress) {
        fromHeading = T.markers[m].heading; fromRemaining = (1 - T.markers[m].t) * L + tail;
      }
    bool haveNext = false; double nextHeading = 0, nextRemaining = 0; bool nextProfiled = true;
    {
      double segStartX = sx, segStartY = sy, after = tail + L;
      for (std::size_t i = 0; i < count && !haveNext; ++i) {
        const Goal& g = *anchors[i];
        const double len = i == 0 ? L : std::hypot(g.pose.x - segStartX, g.pose.y - segStartY);
        after -= len;
        for (std::size_t m = 0; m < g.markerCount; ++m) {
          const auto& mk = g.markers[m];
          if (mk.kind == MarkerKind::Rotation && (i > 0 || mk.t > progress)) {
            haveNext = true; nextHeading = mk.heading; nextRemaining = (1 - mk.t) * len + after;
            nextProfiled = mk.profiled; break;
          }
        }
        if (!haveNext && g.headingMode == HeadingMode::Fixed) {
          haveNext = true; nextHeading = g.pose.heading; nextRemaining = after; nextProfiled = g.profiled;
        }
        // A Motion step ahead: face its direction of travel by the handoff into its segment
        // (the previous anchor's handoff circle), so the heading never steps at the handoff.
        if (!haveNext && i > 0 && g.headingMode == HeadingMode::Motion && len > 1e-9) {
          const Goal& before = *anchors[i - 1];
          haveNext = true; nextHeading = std::atan2(g.pose.y - segStartY, g.pose.x - segStartX) + g.headingOffset;
          nextRemaining = after + len + (before.handoffRadius > 0 ? before.handoffRadius : c.handoffRadius);
          nextProfiled = true;
        }
        segStartX = g.pose.x; segStartY = g.pose.y;
      }
    }
    double desiredHeading = fromHeading;
    bool rotationTargeted = haveNext;
    for (std::size_t m = 0; m < T.markerCount; ++m) rotationTargeted |= T.markers[m].kind == MarkerKind::Rotation;
    if (haveNext) {
      if (!nextProfiled) desiredHeading = nextHeading;
      else {
        const double span = fromRemaining - nextRemaining;
        const double f = span > 1e-9 ? std::clamp((fromRemaining - progressAlong) / span, 0.0, 1.0) : 1;
        desiredHeading = fromHeading + Wrap(nextHeading - fromHeading) * f;
      }
    }
    const double interpolated = desiredHeading;
    if (T.headingMode == HeadingMode::PointAt) desiredHeading = std::atan2(T.aimY - pose.y, T.aimX - pose.x) + T.headingOffset;
    else if (T.headingMode == HeadingMode::Motion) {
      // Toward the target; near it, along the segment (stable at the end of a path).
      double motion = interpolated;
      if (distance > std::max(T.translationTolerance, kMotionMinDistance)) motion = std::atan2(dy, dx);
      else if (L > 1e-9) motion = std::atan2(segY, segX);
      desiredHeading = motion + T.headingOffset;
    }
    if (isFinal && T.headingMode == HeadingMode::Fixed && distance <= T.translationTolerance) desiredHeading = T.pose.heading;
    const double angle = Wrap(desiredHeading - pose.heading);
    out.translationError = distance; out.rotationError = std::abs(angle);
    const bool inPose = isFinal && distance <= T.translationTolerance &&
                        ((T.headingMode == HeadingMode::Free && !rotationTargeted) || std::abs(angle) <= T.rotationTolerance);
    const bool slow = std::hypot(measured.vx, measured.vy) <= T.velocityTolerance &&
                      std::abs(measured.omega) <= T.velocityTolerance;
    const double settled = inPose && (T.settleTime <= 0 || slow) ? state_.settled + dt : 0;
    if (inPose && (T.settleTime <= 0 || settled >= T.settleTime)) {
      state_.previous = {}; state_.settled = settled; state_.events = events;
      out.reached = true; return out;
    }
    // Translation: P on remaining distance clamped to max velocity, optional floor,
    // aimed at the target, plus cross-track correction along the segment's right normal.
    double speed = std::min(c.translationKp * remaining, c.maxVelocity);
    if (c.anticipateSpeedCaps && caps) {
      // Distance to the slower step's handoff point, less one step of travel (the rate
      // limiter applies a speed change one tick after it is requested).
      double d = distance - radius - speed * dt;
      for (std::size_t i = 1; i < count; ++i) {
        if (std::isfinite(caps[i]) && caps[i] < speed)
          speed = std::min(speed, std::sqrt(caps[i] * caps[i] + 2 * c.maxAcceleration * std::max(0.0, d)));
        d += std::hypot(anchors[i]->pose.x - anchors[i - 1]->pose.x, anchors[i]->pose.y - anchors[i - 1]->pose.y);
      }
    }
    if (c.minVelocity > 0 && remaining > T.translationTolerance)
      speed = std::max(speed, std::min(c.minVelocity, c.maxVelocity));
    double vx = distance > 1e-9 ? speed * dx / distance : 0;
    double vy = distance > 1e-9 ? speed * dy / distance : 0;
    if (L > 1e-9) { vx += c.crossTrackKp * crossTrack * uy; vy -= c.crossTrackKp * crossTrack * ux; }
    // Rotation: P on heading error clamped to max angular velocity.
    double omega = std::clamp(c.rotationKp * angle, -c.maxAngularVelocity, c.maxAngularVelocity);
    // Rate limiting: cap speed, then limit the change of each command by the accelerations.
    const double mag = std::hypot(vx, vy);
    if (mag > c.maxVelocity) { vx *= c.maxVelocity / mag; vy *= c.maxVelocity / mag; }
    const auto& p = state_.previous;
    const double change = std::hypot(vx - p.vx, vy - p.vy), maxChange = c.maxAcceleration * dt;
    if (change > maxChange) {
      vx = p.vx + (vx - p.vx) * maxChange / change; vy = p.vy + (vy - p.vy) * maxChange / change;
    }
    omega = p.omega + std::clamp(omega - p.omega, -c.maxAngularAcceleration * dt, c.maxAngularAcceleration * dt);
    if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(omega)) return Invalid();
    state_.previous = {vx, vy, omega};
    if (handoff) {
      // Continue from the heading requested now (Motion/PointAt included), so it never jumps.
      state_.rotHeading = desiredHeading; state_.rotRemaining = tail;
      state_.sx = T.pose.x; state_.sy = T.pose.y; state_.events = 0; state_.settled = 0;
      state_.acceptedRemaining = std::min(progressAlong, tail);
    } else {
      if (!std::isfinite(state_.rotRemaining)) state_.rotRemaining = along;
      state_.events = events; state_.settled = settled; state_.acceptedRemaining = progressAlong;
    }
    out.command = state_.previous; out.reached = handoff; out.handoff = handoff;
    return out;
  }
  // Start a new path from the live pose on the next Calculate call.
  void Restart(Velocity measured = {}) { state_ = {}; state_.previous = measured; }
  const ControlState& State() const { return state_; }
 private:
  ControlResult Invalid() { state_ = {}; ControlResult r; r.valid = false; return r; }
  ControlState state_; double maxDt_ = kDefaultMaxDt;
};

// Single-pose convenience wrapper: drive to one goal (a one-anchor path). A goal
// with a handoff radius passes through instead of stopping.
class PoseController {
 public:
  void Reset(Velocity measured = {}) { follower_.Restart(); previous_ = measured; fresh_ = true; }
  // Longest dt integrated per call; a longer loop is clamped (see PathFollower).
  void SetMaxDt(double maxDt) { follower_.SetMaxDt(maxDt); }
  ControlResult Calculate(Pose pose, Velocity measured, const Goal& goal,
                          const Constraints& c, double dt) {
    if (fresh_) { follower_.Reset(pose, previous_); fresh_ = false; }
    const Goal* anchors[] = {&goal};
    const auto r = follower_.Calculate(pose, measured, anchors, 1, !(goal.handoffRadius > 0), c, dt);
    if (!r.valid || r.reached) fresh_ = true, previous_ = r.command;
    return r;
  }
 private:
  PathFollower follower_; Velocity previous_; bool fresh_ = true;
};
}  // namespace mayhem
