#pragma once
// mayhem::sim sphere game-piece engine. Header-only, C++17, no dependencies.
//
// N balls, each OnGround (rolling or sliding on the terrain), Flying, Held (owned by a robot
// or storage), Scored or Removed. Per Step (1 ms):
// - Flying: gravity, quadratic drag (Cd, frontal area), Magnus lift 0.5 rho A r Cm (spin x v),
//   spin decay; integrated exactly for gravity (x += v dt + a dt^2 / 2).
// - Terrain contact (Terrain::SphereContact): bounce with restitution above a rest speed,
//   tangential friction impulse limited by Coulomb and by the rolling condition (a sliding solid
//   sphere loses 2/7 of its slip), then rolling on the surface with slope gravity (5/7 g sin for
//   a solid sphere) and rolling resistance. A ball that leaves the surface flies again.
// - Ball-ball: spatial hash (counting sort over a fixed table, ~O(N)), impulse with restitution
//   and positional correction, deterministic order.
// - Ball vs robot: each robot's bumper box (up to its height); the ball is pushed out with
//   restitution relative to the robot's contact-point velocity and the reaction is applied to the
//   robot (SwerveSim::ApplyExternalForce, used in its next Step). Intake zones (robot-relative
//   boxes) are checked first and capture balls into Held when enabled and not full.
// - Ball vs Obstacles: the same prisms (with their z ranges) and perimeter walls as robots.
// - Goals: box or vertical cylinder volumes; a ball that enters one while moving along its
//   entry direction (e.g. falling, {0, 0, -1}) is marked Scored and the goal's callback runs;
//   season code decides what happens next (respawn, count points, Remove).
// Call order per 1 ms: robots Step, then GamePieces::Step (its forces reach the robots' next Step).
#include "mayhem/sim/field.hpp"
#include "mayhem/sim/swerve.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace mayhem::sim {

struct BallParams {
  double radius = 0.075, mass = 0.215;
  double restitution = 0.5;        // ball-floor, normal
  double ballRestitution = 0.6;    // ball-ball
  double robotRestitution = 0.3, obstacleRestitution = 0.4;
  double rollingFriction = 0.03;   // rolling decel = rollingFriction * g
  double surfaceFriction = 0.5;    // Coulomb at bounces and against walls/robots
  double dragCd = 0.47;            // sphere
  double magnus = 0.5;             // lift slope Cm: F = 0.5 rho A r Cm (spin x v)
  double spinDamping = 0.5;        // 1/s, exponential spin decay in flight
  double airDensity = 1.225;
  double rollingInertiaFactor = 5.0 / 7.0;  // 1 / (1 + I/(m r^2)); 5/7 solid, 3/5 hollow
  double restSpeed = 0.3;          // m/s: slower impacts do not bounce
};

enum class PieceState { OnGround, Flying, Held, Scored, Removed };

struct Ball {
  Vec3 position{0, 0, 0}, velocity{0, 0, 0}, spin{0, 0, 0};
  PieceState state = PieceState::Removed;
  int owner = -1;            // Held: robot index (or any storage id)
  std::uint64_t heldOrder = 0;
  int goal = -1;             // Scored: goal index
};

/** Robot-relative capture box: x forward, y left, z above the robot's floor reference. */
struct IntakeZone {
  std::string name;
  int robot = 0;
  double xMin = 0, xMax = 0, yMin = 0, yMax = 0, zMin = 0, zMax = 0.3;
  bool enabled = false;
  std::function<bool()> canAccept;     // empty: always accept
  std::function<void(int ball)> onIntake;
};

struct GoalVolume {
  enum class Shape { Box, Cylinder };
  std::string name;
  Shape shape = Shape::Cylinder;
  double cx = 0, cy = 0, yaw = 0;     // centre (Box rotated by yaw)
  double halfX = 0.5, halfY = 0.5;    // Box
  double radius = 0.5;                // Cylinder
  double zMin = 0, zMax = 1;
  Vec3 entryDirection{0, 0, -1};      // required velocity direction on entry; {0,0,0}: any
  bool removeOnScore = false;         // else the ball stays Scored until season code moves it
  std::function<void(int ball, int goal)> onScore;

  bool Contains(Vec3 p) const {
    if (p[2] < zMin || p[2] > zMax) return false;
    const double dx = p[0] - cx, dy = p[1] - cy;
    if (shape == Shape::Cylinder) return dx * dx + dy * dy <= radius * radius;
    const double c = std::cos(yaw), s = std::sin(yaw);
    return std::abs(c * dx + s * dy) <= halfX && std::abs(-s * dx + c * dy) <= halfY;
  }
};

class GamePieces {
 public:
  explicit GamePieces(BallParams params = {}) : p_(params) {}
  const BallParams& Params() const { return p_; }

  /** New ball at rest (or moving) at (x, y, z); z below the surface is lifted onto it. */
  int Spawn(double x, double y, double z = -1, Vec3 velocity = {0, 0, 0}) {
    const int i = Slot();
    Ball& b = balls_[static_cast<std::size_t>(i)];
    b = Ball{};
    b.position = {x, y, z < 0 ? p_.radius : z};
    b.velocity = velocity;
    b.state = z < 0 && velocity[2] == 0 ? PieceState::OnGround : PieceState::Flying;
    prev_[static_cast<std::size_t>(i)] = b.position;
    return i;
  }
  /** Launch a ball (a Held one by index, or a new one if ball < 0) from position. */
  int Launch(Vec3 position, Vec3 velocity, Vec3 spin = {0, 0, 0}, int ball = -1) {
    const int i = ball >= 0 && ball < static_cast<int>(balls_.size()) ? ball : Slot();
    Ball& b = balls_[static_cast<std::size_t>(i)];
    b.position = position;
    b.velocity = velocity;
    b.spin = spin;
    b.state = PieceState::Flying;
    b.owner = -1;
    b.goal = -1;
    prev_[static_cast<std::size_t>(i)] = position;
    return i;
  }
  void SetHeld(int ball, int owner) {
    Ball& b = balls_.at(static_cast<std::size_t>(ball));
    b.state = PieceState::Held;
    b.owner = owner;
    b.heldOrder = ++heldCounter_;
    b.velocity = b.spin = {0, 0, 0};
  }
  /** Oldest ball held by owner (FIFO), or -1. The ball stays Held until Launch/Remove/Spawn. */
  int PeekHeld(int owner) const {
    int best = -1;
    for (std::size_t i = 0; i < balls_.size(); ++i)
      if (balls_[i].state == PieceState::Held && balls_[i].owner == owner &&
          (best < 0 || balls_[i].heldOrder < balls_[static_cast<std::size_t>(best)].heldOrder))
        best = static_cast<int>(i);
    return best;
  }
  int HeldCount(int owner) const {
    int n = 0;
    for (const auto& b : balls_) n += b.state == PieceState::Held && b.owner == owner;
    return n;
  }
  /** Launch the owner's oldest held ball; returns its index or -1 if none is held. */
  int LaunchHeld(int owner, Vec3 position, Vec3 velocity, Vec3 spin = {0, 0, 0}) {
    const int i = PeekHeld(owner);
    return i < 0 ? -1 : Launch(position, velocity, spin, i);
  }
  void Remove(int ball) { balls_.at(static_cast<std::size_t>(ball)).state = PieceState::Removed; }
  /** Put a ball back on the field at (x, y), at rest. */
  void Place(int ball, double x, double y, double z = -1) {
    Ball& b = balls_.at(static_cast<std::size_t>(ball));
    b = Ball{};
    b.position = {x, y, z < 0 ? p_.radius : z};
    b.state = z < 0 ? PieceState::OnGround : PieceState::Flying;
    prev_[static_cast<std::size_t>(ball)] = b.position;
  }

  int AddRobot(SwerveSim* robot) { robots_.push_back(robot); return static_cast<int>(robots_.size()) - 1; }
  int AddIntake(IntakeZone zone) { intakes_.push_back(std::move(zone)); return static_cast<int>(intakes_.size()) - 1; }
  IntakeZone& Intake(int i) { return intakes_.at(static_cast<std::size_t>(i)); }
  int AddGoal(GoalVolume goal) { goals_.push_back(std::move(goal)); return static_cast<int>(goals_.size()) - 1; }
  GoalVolume& Goal(int i) { return goals_.at(static_cast<std::size_t>(i)); }

  const std::vector<Ball>& Balls() const { return balls_; }
  Ball& Get(int i) { return balls_.at(static_cast<std::size_t>(i)); }
  std::size_t Count(PieceState s) const {
    std::size_t n = 0;
    for (const auto& b : balls_) n += b.state == s;
    return n;
  }
  std::vector<Vec3> GroundPositions() const { return Positions(PieceState::OnGround); }
  std::vector<Vec3> FlyingPositions() const { return Positions(PieceState::Flying); }

  void Step(double dt, const Terrain* terrain = nullptr, const Obstacles* obstacles = nullptr) {
    for (std::size_t i = 0; i < balls_.size(); ++i) prev_[i] = balls_[i].position;
    Integrate(dt, terrain);
    Collide(dt);
    Robots(dt);
    if (obstacles) Static(obstacles);
    Goals();
    for (auto& b : balls_)
      if ((b.state == PieceState::Flying || b.state == PieceState::OnGround) &&
          (b.position[2] < -1 || std::abs(b.position[0]) > 1e3 || std::abs(b.position[1]) > 1e3))
        b.state = PieceState::Removed;
  }

 private:
  BallParams p_;
  std::vector<Ball> balls_;
  std::vector<Vec3> prev_;
  std::vector<SwerveSim*> robots_;
  std::vector<IntakeZone> intakes_;
  std::vector<GoalVolume> goals_;
  std::uint64_t heldCounter_ = 0;
  // Spatial hash scratch.
  std::vector<std::uint32_t> cellStart_, cellOf_, sorted_;

  static bool Active(const Ball& b) { return b.state == PieceState::OnGround || b.state == PieceState::Flying; }

  int Slot() {
    for (std::size_t i = 0; i < balls_.size(); ++i)
      if (balls_[i].state == PieceState::Removed) return static_cast<int>(i);
    balls_.emplace_back();
    prev_.push_back({0, 0, 0});
    return static_cast<int>(balls_.size()) - 1;
  }
  std::vector<Vec3> Positions(PieceState s) const {
    std::vector<Vec3> out;
    for (const auto& b : balls_) if (b.state == s) out.push_back(b.position);
    return out;
  }

  void Integrate(double dt, const Terrain* terrain) {
    const double area = kPi * p_.radius * p_.radius;
    const double kDrag = 0.5 * p_.airDensity * p_.dragCd * area / p_.mass;
    const double kMagnus = 0.5 * p_.airDensity * area * p_.radius * p_.magnus / p_.mass;
    const double spinDecay = std::exp(-p_.spinDamping * dt);
    for (auto& b : balls_) {
      if (!Active(b)) continue;
      Vec3& x = b.position;
      Vec3& v = b.velocity;
      const double speed = vec::Norm(v);
      Vec3 a = vec::Add({0, 0, -kGravity}, vec::Scale(v, -kDrag * speed));
      if (b.state == PieceState::Flying) {
        a = vec::Add(a, vec::Scale(vec::Cross(b.spin, v), kMagnus));
        b.spin = vec::Scale(b.spin, spinDecay);
        x = vec::Add(x, vec::Add(vec::Scale(v, dt), vec::Scale(a, 0.5 * dt * dt)));
        v = vec::Add(v, vec::Scale(a, dt));
      } else {
        // Rolling on the surface under the ball.
        SurfaceContact c = Surface(terrain, x);
        const Vec3 n = c.normal;
        const Vec3 g{0, 0, -kGravity};
        Vec3 aT = vec::Scale(vec::Sub(g, vec::Scale(n, vec::Dot(g, n))), p_.rollingInertiaFactor);
        Vec3 vT = vec::Sub(v, vec::Scale(n, vec::Dot(v, n)));
        const double vt = vec::Norm(vT);
        const double decel = p_.rollingFriction * kGravity * std::max(0.0, n[2]);
        vT = vec::Add(vT, vec::Scale(aT, dt));
        vT = vec::Add(vT, vec::Scale(vT, -kDrag * vt * dt));
        const double vtNew = vec::Norm(vT);
        if (vtNew <= decel * dt) vT = {0, 0, 0};
        else vT = vec::Scale(vT, (vtNew - decel * dt) / vtNew);
        v = vT;
        x = vec::Add(x, vec::Scale(v, dt));
        b.spin = {0, 0, 0};
      }
      // Surface contact.
      const SurfaceContact c = Surface(terrain, x);
      if (c.depth > 0) {
        x = vec::Add(x, vec::Scale(c.normal, c.depth));
        const double vn = vec::Dot(v, c.normal);
        if (vn < 0) {
          Vec3 vT = vec::Sub(v, vec::Scale(c.normal, vn));
          const double vt = vec::Norm(vT);
          const bool bounce = -vn > p_.restSpeed;
          const double jn = bounce ? -(1 + p_.restitution) * vn : -vn;
          // Friction: at most Coulomb, at most what reaches rolling.
          const double dvt = std::min(p_.surfaceFriction * jn, (1 - p_.rollingInertiaFactor) * vt);
          if (vt > 1e-12) vT = vec::Scale(vT, (vt - dvt) / vt);
          v = vec::Add(vT, vec::Scale(c.normal, bounce ? -p_.restitution * vn : 0));
          if (!bounce) b.state = PieceState::OnGround;
        }
      } else if (b.state == PieceState::OnGround && c.depth < -0.002) {
        b.state = PieceState::Flying;  // rolled off an edge
      }
      if (b.state == PieceState::OnGround && vec::Dot(v, c.normal) > p_.restSpeed) b.state = PieceState::Flying;
    }
  }

  SurfaceContact Surface(const Terrain* terrain, Vec3 x) const {
    if (terrain) return terrain->SphereContact(x[0], x[1], x[2], p_.radius);
    SurfaceContact c;
    c.depth = p_.radius - x[2];
    c.normal = {0, 0, 1};
    c.point = {x[0], x[1], 0};
    return c;
  }

  void Collide(double /*dt*/) {
    const std::size_t n = balls_.size();
    if (n < 2) return;
    const double cell = 2 * p_.radius;
    std::size_t table = 1;
    while (table < 2 * n) table <<= 1;
    cellStart_.assign(table + 1, 0);
    cellOf_.resize(n);
    sorted_.resize(n);
    auto hash = [&](long long ix, long long iy) {
      const std::uint64_t h = static_cast<std::uint64_t>(ix) * 73856093ULL ^ static_cast<std::uint64_t>(iy) * 19349663ULL;
      return static_cast<std::uint32_t>(h & (table - 1));
    };
    auto cellX = [&](double v) { return static_cast<long long>(std::floor(v / cell)); };
    for (std::size_t i = 0; i < n; ++i) {
      if (!Active(balls_[i])) { cellOf_[i] = static_cast<std::uint32_t>(table); continue; }
      cellOf_[i] = hash(cellX(balls_[i].position[0]), cellX(balls_[i].position[1]));
      ++cellStart_[cellOf_[i]];
    }
    std::uint32_t sum = 0;
    for (std::size_t c = 0; c <= table; ++c) { const std::uint32_t k = cellStart_[c]; cellStart_[c] = sum; sum += k; }
    std::vector<std::uint32_t> fill(cellStart_.begin(), cellStart_.end());
    for (std::size_t i = 0; i < n; ++i)
      if (cellOf_[i] < table) sorted_[fill[cellOf_[i]]++] = static_cast<std::uint32_t>(i);
    const double minDist = 2 * p_.radius, minDist2 = minDist * minDist;
    for (std::size_t i = 0; i < n; ++i) {
      if (!Active(balls_[i])) continue;
      const long long ix = cellX(balls_[i].position[0]), iy = cellX(balls_[i].position[1]);
      std::uint32_t seen[9];
      int nseen = 0;
      for (long long dx = -1; dx <= 1; ++dx)
        for (long long dy = -1; dy <= 1; ++dy) {
          const std::uint32_t c = hash(ix + dx, iy + dy);
          bool dup = false;
          for (int k = 0; k < nseen; ++k) dup = dup || seen[k] == c;
          if (dup) continue;
          seen[nseen++] = c;
          for (std::uint32_t s = cellStart_[c]; s < cellStart_[c + 1]; ++s) {
            const std::uint32_t j = sorted_[s];
            if (j <= i) continue;
            Pair(balls_[i], balls_[j], minDist, minDist2);
          }
        }
    }
  }

  void Pair(Ball& a, Ball& b, double minDist, double minDist2) {
    Vec3 d = vec::Sub(b.position, a.position);
    const bool bothGround = a.state == PieceState::OnGround && b.state == PieceState::OnGround;
    if (bothGround) d[2] = 0;
    const double dist2 = vec::Dot(d, d);
    if (dist2 >= minDist2) return;
    const double dist = std::sqrt(dist2);
    const Vec3 n = dist > 1e-9 ? vec::Scale(d, 1 / dist) : Vec3{1, 0, 0};
    const double vrel = vec::Dot(vec::Sub(b.velocity, a.velocity), n);
    if (vrel < 0) {
      const double j = -(1 + p_.ballRestitution) * vrel / 2;
      a.velocity = vec::Sub(a.velocity, vec::Scale(n, j));
      b.velocity = vec::Add(b.velocity, vec::Scale(n, j));
    }
    const double push = 0.5 * (minDist - dist);
    a.position = vec::Sub(a.position, vec::Scale(n, push));
    b.position = vec::Add(b.position, vec::Scale(n, push));
  }

  void Robots(double dt) {
    for (std::size_t r = 0; r < robots_.size(); ++r) {
      SwerveSim* robot = robots_[r];
      if (!robot) continue;
      const auto pose = robot->Pose3();
      const auto vel = robot->FieldVelocity();
      const auto& rp = robot->Params();
      const double c = std::cos(pose.yaw), s = std::sin(pose.yaw);
      const double hx = rp.bumperLengthX / 2, hy = rp.bumperLengthY / 2;
      const double top = pose.z + robot->RobotHeight();
      for (std::size_t i = 0; i < balls_.size(); ++i) {
        Ball& b = balls_[i];
        if (!Active(b)) continue;
        const double dx = b.position[0] - pose.x, dy = b.position[1] - pose.y;
        const double u = c * dx + s * dy, v = -s * dx + c * dy, h = b.position[2] - pose.z;
        if (std::abs(u) > hx + 1.0 || std::abs(v) > hy + 1.0) continue;
        // Intakes first.
        bool taken = false;
        for (auto& zone : intakes_) {
          if (zone.robot != static_cast<int>(r) || !zone.enabled) continue;
          if (u < zone.xMin || u > zone.xMax || v < zone.yMin || v > zone.yMax || h < zone.zMin || h > zone.zMax) continue;
          if (zone.canAccept && !zone.canAccept()) continue;
          SetHeld(static_cast<int>(i), static_cast<int>(r));
          if (zone.onIntake) zone.onIntake(static_cast<int>(i));
          taken = true;
          break;
        }
        if (taken) continue;
        if (b.position[2] - p_.radius > top) continue;
        // Bumper box (2D, robot frame).
        const double cu = std::clamp(u, -hx, hx), cv = std::clamp(v, -hy, hy);
        double nu = u - cu, nv = v - cv;
        double dist = std::hypot(nu, nv), depth;
        if (dist > 1e-9) {
          if (dist >= p_.radius) continue;
          depth = p_.radius - dist;
          nu /= dist; nv /= dist;
        } else {  // centre inside the box: leave through the nearest side
          const double f[4] = {hx - u, hx + u, hy - v, hy + v};
          int k = 0;
          for (int q = 1; q < 4; ++q) if (f[q] < f[k]) k = q;
          depth = p_.radius + f[k];
          nu = k == 0 ? 1 : (k == 1 ? -1 : 0);
          nv = k == 2 ? 1 : (k == 3 ? -1 : 0);
        }
        const double nx = c * nu - s * nv, ny = s * nu + c * nv;
        const double px = pose.x + c * cu - s * cv, py = pose.y + s * cu + c * cv;
        // Robot surface velocity at the contact point.
        const double rvx = vel.vx - vel.omega * (py - pose.y), rvy = vel.vy + vel.omega * (px - pose.x);
        // A ball inside the robot (spawned under it, or a teleport) is pushed out at no more
        // than 3 m/s rather than jumping out in one step.
        const double move = depth > p_.radius ? std::min(depth, 3.0 * dt) : depth;
        b.position[0] += nx * move;
        b.position[1] += ny * move;
        const double vrel = (b.velocity[0] - rvx) * nx + (b.velocity[1] - rvy) * ny;
        if (vrel < 0) {
          const double dv = -(1 + p_.robotRestitution) * vrel;
          b.velocity[0] += dv * nx;
          b.velocity[1] += dv * ny;
          const double impulse = p_.mass * dv;
          robot->ApplyExternalForce(-impulse * nx / dt, -impulse * ny / dt, px, py);
        }
      }
    }
  }

  void Static(const Obstacles* obstacles) {
    for (auto& b : balls_) {
      if (!Active(b)) continue;
      auto respond = [&](const SurfaceContact& c) {
        if (c.depth <= 0) return;
        b.position = vec::Add(b.position, vec::Scale(c.normal, c.depth));
        const double vn = vec::Dot(b.velocity, c.normal);
        if (vn >= 0) return;
        Vec3 vT = vec::Sub(b.velocity, vec::Scale(c.normal, vn));
        const double vt = vec::Norm(vT);
        const double jn = -(1 + p_.obstacleRestitution) * vn;
        const double dvt = std::min(p_.surfaceFriction * jn, (1 - p_.rollingInertiaFactor) * vt);
        if (vt > 1e-12) vT = vec::Scale(vT, (vt - dvt) / vt);
        b.velocity = vec::Add(vT, vec::Scale(c.normal, -p_.obstacleRestitution * vn));
        if (b.state == PieceState::OnGround && c.normal[2] > 0.7) return;  // landed on a top
        if (c.normal[2] > 0.7 && -vn < p_.restSpeed) b.state = PieceState::OnGround;
      };
      for (std::size_t w = 0; w < obstacles->Walls().size(); ++w)
        respond(obstacles->SphereVsWall(b.position[0], b.position[1], b.position[2], p_.radius, w));
      for (std::size_t i = 0; i < obstacles->Prisms().size(); ++i)
        respond(obstacles->SphereVsPrism(b.position[0], b.position[1], b.position[2], p_.radius, i));
    }
  }

  void Goals() {
    for (std::size_t i = 0; i < balls_.size(); ++i) {
      Ball& b = balls_[i];
      if (!Active(b)) continue;
      for (std::size_t g = 0; g < goals_.size(); ++g) {
        GoalVolume& goal = goals_[g];
        if (!goal.Contains(b.position) || goal.Contains(prev_[i])) continue;
        if (vec::Dot(b.velocity, goal.entryDirection) < 0 ||
            (vec::Norm(goal.entryDirection) > 0 && vec::Dot(b.velocity, goal.entryDirection) == 0))
          continue;
        b.state = goal.removeOnScore ? PieceState::Removed : PieceState::Scored;
        b.goal = static_cast<int>(g);
        if (goal.onScore) goal.onScore(static_cast<int>(i), static_cast<int>(g));
        break;
      }
    }
  }
};

}  // namespace mayhem::sim
