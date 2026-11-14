#pragma once
// mayhem::sim generic field geometry: Terrain (what robots and pieces drive and roll over) and
// Obstacles (what they bump into). Header-only, C++17, no dependencies.
//
// Field frame: x, y on the carpet, z up, metres; yaw counter-clockwise from +x. A season builds
// its field from these primitives (see docs/SIMULATION.md).
//
// Terrain is a composition of features over the z = 0 carpet:
//   AddBox      flat-topped block (an x-y rectangle at any yaw, from z = 0 up to `height`)
//   AddRamp     a straight incline across a rectangle
//   AddProfile  any cross-section: a polyline of (distance along yaw, height) extruded over a
//               width, e.g. a bump with ramps on both sides (AddBump is a shortcut)
// Height()/Normal() answer the height field (the highest feature wins). SphereContact() is the
// exact distance from a sphere (wheel or ball) to the surface, so a wheel rolling over a crest
// sees a smoothly rotating normal instead of a kink. Use boxes only for things that are driven
// onto (platforms, low steps); anything taller than about a wheel radius that should stop a
// robot belongs in Obstacles too, since a terrain step only ever pushes up.
//
// Obstacles are vertical prisms with a z range: the field perimeter (a rectangle with optional
// chamfered corners and a wall height), and named convex polygons. A robot collides with a
// prism only if their height ranges overlap, so a short robot drives under an overhead
// structure whose zMin is above it. Contact geometry: separating-axis test between the robot's
// bumper rectangle and each polygon (penetration depth, push-out normal, and the centroid of the
// overlap as the contact point), and closest-point tests for spheres.
#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

namespace mayhem::sim {

using Vec2 = std::array<double, 2>;
using Vec3 = std::array<double, 3>;

/** One obstacle contact during a Step: total normal impulse (N*s) and the last contact point. */
struct ContactEvent {
  std::string_view name;  // points into the Obstacles (or a robot's) storage
  double impulse = 0, x = 0, y = 0, z = 0;
};

/** Result of a sphere-vs-surface query: penetration depth (> 0 overlapping) and push-out normal. */
struct SurfaceContact {
  double depth = -std::numeric_limits<double>::infinity();
  Vec3 normal{0, 0, 1};
  Vec3 point{0, 0, 0};  // closest surface point
};

class Terrain {
 public:
  /** Flat block from z = 0 to height, lx by ly, centred at (cx, cy), rotated by yaw. */
  int AddBox(double cx, double cy, double yaw, double lx, double ly, double height) {
    Feature f = Make(cx, cy, yaw, ly);
    f.kind = Kind::Box;
    f.halfX = lx / 2;
    f.top = height;
    return Push(f);
  }
  /** Straight incline over length (along yaw) by width, from hStart at the -yaw end to hEnd. */
  int AddRamp(double cx, double cy, double yaw, double length, double width, double hStart, double hEnd) {
    return AddProfile(cx, cy, yaw, width, {{-length / 2, hStart}, {length / 2, hEnd}});
  }
  /**
   * Arbitrary cross-section: points are (u, h) with u the distance from (cx, cy) along yaw,
   * strictly increasing; extruded over `width` across yaw. Outside [u0, uN] the feature is
   * absent (height 0), so start and end at h = 0 for a bump, or add a box for a raised floor.
   */
  int AddProfile(double cx, double cy, double yaw, double width, std::vector<Vec2> points) {
    Feature f = Make(cx, cy, yaw, width);
    f.kind = Kind::Profile;
    f.points = std::move(points);
    f.halfX = std::max(std::abs(f.points.front()[0]), std::abs(f.points.back()[0]));
    for (const auto& p : f.points) f.top = std::max(f.top, p[1]);
    return Push(f);
  }
  /** Symmetric bump: ramps up over (depth - topDepth)/2, flat top of topDepth, ramps down. */
  int AddBump(double cx, double cy, double yaw, double depth, double width, double height, double topDepth = 0) {
    return AddProfile(cx, cy, yaw, width,
                      {{-depth / 2, 0}, {-topDepth / 2, height}, {topDepth / 2, height}, {depth / 2, 0}});
  }
  void Clear() { features_.clear(); }
  bool Empty() const { return features_.empty(); }

  /** Surface height at (x, y) (0 on the carpet). */
  double Height(double x, double y) const {
    double best = 0;
    for (const auto& f : features_) {
      if (!InBounds(f, x, y, 0)) continue;
      double slope;
      best = std::max(best, FeatureHeight(f, x, y, slope));
    }
    return best;
  }
  /** Unit surface normal at (x, y) of the highest feature there. */
  Vec3 Normal(double x, double y) const {
    double best = 0;
    Vec3 n{0, 0, 1};
    for (const auto& f : features_) {
      if (!InBounds(f, x, y, 0)) continue;
      double slope = 0;
      const double h = FeatureHeight(f, x, y, slope);
      if (h > best) {
        best = h;
        const double len = std::sqrt(1 + slope * slope);
        n = {-slope * f.c / len, -slope * f.s / len, 1 / len};
      }
    }
    return n;
  }
  /** Sphere of radius r centred at (x, y, z) against the terrain: the deepest contact. */
  SurfaceContact SphereContact(double x, double y, double z, double r) const {
    SurfaceContact best;
    best.depth = r - z;
    best.normal = {0, 0, 1};
    best.point = {x, y, 0};
    for (const auto& f : features_) {
      if (!InBounds(f, x, y, r) || z - r > f.top) continue;
      const SurfaceContact c = f.kind == Kind::Box ? BoxContact(f, x, y, z, r) : ProfileContact(f, x, y, z, r);
      if (c.depth > best.depth) best = c;
    }
    return best;
  }

 private:
  enum class Kind { Box, Profile };
  struct Feature {
    Kind kind = Kind::Box;
    double cx = 0, cy = 0, c = 1, s = 0, halfX = 0, halfY = 0, top = 0;
    std::vector<Vec2> points;
  };
  std::vector<Feature> features_;

  static Feature Make(double cx, double cy, double yaw, double width) {
    Feature f;
    f.cx = cx; f.cy = cy; f.c = std::cos(yaw); f.s = std::sin(yaw); f.halfY = width / 2;
    return f;
  }
  int Push(const Feature& f) { features_.push_back(f); return static_cast<int>(features_.size()) - 1; }
  static void Local(const Feature& f, double x, double y, double& u, double& v) {
    const double dx = x - f.cx, dy = y - f.cy;
    u = f.c * dx + f.s * dy;
    v = -f.s * dx + f.c * dy;
  }
  static bool InBounds(const Feature& f, double x, double y, double margin) {
    double u, v;
    Local(f, x, y, u, v);
    if (std::abs(v) > f.halfY + margin) return false;
    if (f.kind == Kind::Box) return std::abs(u) <= f.halfX + margin;
    return u >= f.points.front()[0] - margin && u <= f.points.back()[0] + margin;
  }
  static double FeatureHeight(const Feature& f, double x, double y, double& slope) {
    double u, v;
    Local(f, x, y, u, v);
    slope = 0;
    if (std::abs(v) > f.halfY) return 0;
    if (f.kind == Kind::Box) return std::abs(u) <= f.halfX ? f.top : 0;
    const auto& p = f.points;
    if (u < p.front()[0] || u > p.back()[0]) return 0;
    for (std::size_t i = 1; i < p.size(); ++i) {
      if (u <= p[i][0]) {
        const double du = p[i][0] - p[i - 1][0];
        slope = du > 0 ? (p[i][1] - p[i - 1][1]) / du : 0;
        return p[i - 1][1] + slope * (u - p[i - 1][0]);
      }
    }
    return p.back()[1];
  }
  static SurfaceContact BoxContact(const Feature& f, double x, double y, double z, double r) {
    double u, v;
    Local(f, x, y, u, v);
    const double cu = std::clamp(u, -f.halfX, f.halfX), cv = std::clamp(v, -f.halfY, f.halfY);
    const double cz = std::clamp(z, 0.0, f.top);
    double du = u - cu, dv = v - cv, dz = z - cz;
    double dist = std::sqrt(du * du + dv * dv + dz * dz);
    SurfaceContact c;
    if (dist > 1e-12) {
      c.depth = r - dist;
      du /= dist; dv /= dist; dz /= dist;
    } else {  // centre inside: leave through the nearest face (top preferred on ties)
      const double faces[5] = {f.top - z, f.halfX - u, f.halfX + u, f.halfY - v, f.halfY + v};
      int k = 0;
      for (int i = 1; i < 5; ++i) if (faces[i] < faces[k]) k = i;
      c.depth = r + faces[k];
      du = k == 1 ? 1 : (k == 2 ? -1 : 0);
      dv = k == 3 ? 1 : (k == 4 ? -1 : 0);
      dz = k == 0 ? 1 : 0;
    }
    c.normal = {f.c * du - f.s * dv, f.s * du + f.c * dv, dz};
    c.point = {f.cx + f.c * cu - f.s * cv, f.cy + f.s * cu + f.c * cv, cz};
    return c;
  }
  static SurfaceContact ProfileContact(const Feature& f, double x, double y, double z, double r) {
    double u, v;
    Local(f, x, y, u, v);
    const auto& p = f.points;
    double bestD2 = std::numeric_limits<double>::infinity(), bu = 0, bz = 0;
    for (std::size_t i = 1; i < p.size(); ++i) {
      const double ax = p[i - 1][0], az = p[i - 1][1], ex = p[i][0] - ax, ez = p[i][1] - az;
      const double len2 = ex * ex + ez * ez;
      const double t = len2 > 0 ? std::clamp(((u - ax) * ex + (z - az) * ez) / len2, 0.0, 1.0) : 0.0;
      const double qu = ax + t * ex, qz = az + t * ez;
      const double d2 = (u - qu) * (u - qu) + (z - qz) * (z - qz);
      if (d2 < bestD2) { bestD2 = d2; bu = qu; bz = qz; }
    }
    const double cv = std::clamp(v, -f.halfY, f.halfY);
    double du = u - bu, dv = v - cv, dz = z - bz;
    double dist = std::sqrt(du * du + dv * dv + dz * dz);
    double slope;
    const double surface = FeatureHeight(f, x, y, slope);
    const bool below = std::abs(v) <= f.halfY && z < surface;
    SurfaceContact c;
    if (dist < 1e-12) {
      const double len = std::sqrt(1 + slope * slope);
      du = -slope / len; dv = 0; dz = 1 / len;
      c.depth = r;
    } else {
      du /= dist; dv /= dist; dz /= dist;
      if (below) { du = -du; dv = -dv; dz = -dz; }
      c.depth = below ? r + dist : r - dist;
    }
    c.normal = {f.c * du - f.s * dv, f.s * du + f.c * dv, dz};
    c.point = {f.cx + f.c * bu - f.s * cv, f.cy + f.s * bu + f.c * cv, bz};
    return c;
  }
};

/** A rotated rectangle in the x-y plane (a robot's bumpers or frame). */
struct OrientedBox {
  double cx = 0, cy = 0, yaw = 0, halfX = 0.4, halfY = 0.4;
  std::array<Vec2, 4> Corners() const {
    const double c = std::cos(yaw), s = std::sin(yaw);
    std::array<Vec2, 4> out{};
    const double sx[4] = {1, -1, -1, 1}, sy[4] = {1, 1, -1, -1};
    for (int i = 0; i < 4; ++i)
      out[i] = {cx + c * sx[i] * halfX - s * sy[i] * halfY, cy + s * sx[i] * halfX + c * sy[i] * halfY};
    return out;  // counter-clockwise
  }
};

/** Penetration of a body into an obstacle: depth (m), unit normal pushing the body out, point. */
struct Penetration {
  bool hit = false;
  double depth = 0;
  Vec2 normal{0, 0};
  Vec2 point{0, 0};
};

namespace geometry {

inline double Cross(Vec2 a, Vec2 b) { return a[0] * b[1] - a[1] * b[0]; }
inline double Dot(Vec2 a, Vec2 b) { return a[0] * b[0] + a[1] * b[1]; }

/** Make a convex polygon counter-clockwise. */
inline void MakeCcw(std::vector<Vec2>& poly) {
  double area = 0;
  for (std::size_t i = 0; i < poly.size(); ++i) area += Cross(poly[i], poly[(i + 1) % poly.size()]);
  if (area < 0) std::reverse(poly.begin(), poly.end());
}

/** Clip a convex polygon by another convex (CCW) polygon (Sutherland-Hodgman). */
inline std::vector<Vec2> Clip(std::vector<Vec2> subject, const std::vector<Vec2>& clip) {
  for (std::size_t i = 0; i < clip.size() && !subject.empty(); ++i) {
    const Vec2 a = clip[i], b = clip[(i + 1) % clip.size()];
    const Vec2 e{b[0] - a[0], b[1] - a[1]};
    std::vector<Vec2> out;
    for (std::size_t j = 0; j < subject.size(); ++j) {
      const Vec2 p = subject[j], q = subject[(j + 1) % subject.size()];
      const double sp = Cross(e, {p[0] - a[0], p[1] - a[1]}), sq = Cross(e, {q[0] - a[0], q[1] - a[1]});
      if (sp >= 0) out.push_back(p);
      if ((sp >= 0) != (sq >= 0)) {
        const double t = sp / (sp - sq);
        out.push_back({p[0] + t * (q[0] - p[0]), p[1] + t * (q[1] - p[1])});
      }
    }
    subject = std::move(out);
  }
  return subject;
}

inline Vec2 Centroid(const std::vector<Vec2>& poly) {
  double a = 0, x = 0, y = 0;
  for (std::size_t i = 0; i < poly.size(); ++i) {
    const Vec2 p = poly[i], q = poly[(i + 1) % poly.size()];
    const double c = Cross(p, q);
    a += c; x += (p[0] + q[0]) * c; y += (p[1] + q[1]) * c;
  }
  if (std::abs(a) < 1e-14) {
    Vec2 m{0, 0};
    for (const auto& p : poly) { m[0] += p[0]; m[1] += p[1]; }
    const double n = poly.empty() ? 1.0 : static_cast<double>(poly.size());
    return {m[0] / n, m[1] / n};
  }
  return {x / (3 * a), y / (3 * a)};
}

/**
 * Separating-axis penetration of convex polygon `body` into convex polygon `obstacle` (both
 * CCW). The normal points from the obstacle toward the body. `hint` (a previous contact
 * normal, or {0,0}) is kept while it is within 2 cm of the minimum, so thin obstacles are not
 * pushed through.
 */
inline Penetration Polygons(const std::vector<Vec2>& body, const std::vector<Vec2>& obstacle, Vec2 hint = {0, 0}) {
  Penetration best;
  best.depth = std::numeric_limits<double>::infinity();
  Vec2 cb{0, 0}, co{0, 0};
  for (const auto& p : body) { cb[0] += p[0]; cb[1] += p[1]; }
  for (const auto& p : obstacle) { co[0] += p[0]; co[1] += p[1]; }
  cb = {cb[0] / body.size(), cb[1] / body.size()};
  co = {co[0] / obstacle.size(), co[1] / obstacle.size()};
  auto overlapOn = [&](Vec2 n) {
    double bmin = 1e300, bmax = -1e300, omin = 1e300, omax = -1e300;
    for (const auto& p : body) { const double d = Dot(p, n); bmin = std::min(bmin, d); bmax = std::max(bmax, d); }
    for (const auto& p : obstacle) { const double d = Dot(p, n); omin = std::min(omin, d); omax = std::max(omax, d); }
    return std::make_pair(omax - bmin, bmax - omin);  // push body along +n, along -n
  };
  auto test = [&](Vec2 n) {
    const auto [plus, minus] = overlapOn(n);
    if (plus <= 0 || minus <= 0) return false;
    // Prefer the side the body's centre is on.
    const bool towardPlus = Dot({cb[0] - co[0], cb[1] - co[1]}, n) >= 0;
    const double depth = towardPlus ? plus : minus;
    if (depth < best.depth) { best.depth = depth; best.normal = towardPlus ? n : Vec2{-n[0], -n[1]}; }
    return true;
  };
  for (const auto* poly : {&body, &obstacle})
    for (std::size_t i = 0; i < poly->size(); ++i) {
      const Vec2 a = (*poly)[i], b = (*poly)[(i + 1) % poly->size()];
      Vec2 n{b[1] - a[1], a[0] - b[0]};
      const double len = std::hypot(n[0], n[1]);
      if (len < 1e-12) continue;
      if (!test({n[0] / len, n[1] / len})) return {};
    }
  if (hint[0] != 0 || hint[1] != 0) {
    const auto [plus, minus] = overlapOn(hint);
    (void)minus;
    if (plus > 0 && plus < best.depth + 0.02) { best.depth = plus; best.normal = hint; }
  }
  best.hit = true;
  const auto overlap = Clip(body, obstacle);
  best.point = overlap.empty() ? Vec2{(cb[0] + co[0]) / 2, (cb[1] + co[1]) / 2} : Centroid(overlap);
  return best;
}

/** Closest point on a convex CCW polygon's boundary; inside reports whether p is inside. */
inline Vec2 ClosestOnBoundary(const std::vector<Vec2>& poly, Vec2 p, bool& inside, Vec2& edgeNormal) {
  inside = true;
  double best = 1e300;
  Vec2 out = p;
  for (std::size_t i = 0; i < poly.size(); ++i) {
    const Vec2 a = poly[i], b = poly[(i + 1) % poly.size()];
    const Vec2 e{b[0] - a[0], b[1] - a[1]};
    const double len2 = Dot(e, e);
    if (Cross(e, {p[0] - a[0], p[1] - a[1]}) < 0) inside = false;
    const double t = len2 > 0 ? std::clamp(Dot({p[0] - a[0], p[1] - a[1]}, e) / len2, 0.0, 1.0) : 0.0;
    const Vec2 q{a[0] + t * e[0], a[1] + t * e[1]};
    const double d2 = (p[0] - q[0]) * (p[0] - q[0]) + (p[1] - q[1]) * (p[1] - q[1]);
    if (d2 < best) {
      best = d2;
      out = q;
      const double len = std::sqrt(len2);
      edgeNormal = len > 0 ? Vec2{e[1] / len, -e[0] / len} : Vec2{0, 0};
    }
  }
  return out;
}

}  // namespace geometry

class Obstacles {
 public:
  struct Prism {
    std::string name;
    std::vector<Vec2> vertices;  // convex, CCW
    double zMin = 0, zMax = std::numeric_limits<double>::infinity();
    double minX = 0, minY = 0, maxX = 0, maxY = 0;  // bounding box
  };
  struct Wall {
    std::string name;
    Vec2 point{0, 0}, inward{0, 0};  // half-plane: Dot(p - point, inward) >= 0 is inside
    double height = 0.5;
  };

  /**
   * Field perimeter: the inside of [xMin, xMax] x [yMin, yMax] with 45-degree chamfers of leg
   * `chamfer` at each corner (0 = square corners). Walls are wallHeight tall (balls above
   * them fly out; robots always hit them).
   */
  void SetPerimeter(double xMin, double yMin, double xMax, double yMax, double chamfer = 0,
                    double wallHeight = 0.5, const std::string& name = "perimeter") {
    walls_.clear();
    auto add = [&](const std::string& n, Vec2 p, Vec2 inward) {
      const double len = std::hypot(inward[0], inward[1]);
      walls_.push_back({n, p, {inward[0] / len, inward[1] / len}, wallHeight});
    };
    add(name + "/xmin", {xMin, 0}, {1, 0});
    add(name + "/xmax", {xMax, 0}, {-1, 0});
    add(name + "/ymin", {0, yMin}, {0, 1});
    add(name + "/ymax", {0, yMax}, {0, -1});
    if (chamfer > 0) {
      add(name + "/corner_xmin_ymin", {xMin + chamfer, yMin}, {1, 1});
      add(name + "/corner_xmax_ymin", {xMax - chamfer, yMin}, {-1, 1});
      add(name + "/corner_xmin_ymax", {xMin + chamfer, yMax}, {1, -1});
      add(name + "/corner_xmax_ymax", {xMax - chamfer, yMax}, {-1, -1});
    }
  }
  /** Convex polygon (any winding) occupying zMin..zMax. Returns its index. */
  int AddPolygon(const std::string& name, std::vector<Vec2> vertices, double zMin = 0,
                 double zMax = std::numeric_limits<double>::infinity()) {
    geometry::MakeCcw(vertices);
    Prism p{name, std::move(vertices), zMin, zMax};
    p.minX = p.minY = 1e300; p.maxX = p.maxY = -1e300;
    for (const auto& v : p.vertices) {
      p.minX = std::min(p.minX, v[0]); p.maxX = std::max(p.maxX, v[0]);
      p.minY = std::min(p.minY, v[1]); p.maxY = std::max(p.maxY, v[1]);
    }
    prisms_.push_back(std::move(p));
    return static_cast<int>(prisms_.size()) - 1;
  }
  int AddBox(const std::string& name, double cx, double cy, double yaw, double lx, double ly,
             double zMin = 0, double zMax = std::numeric_limits<double>::infinity()) {
    const auto corners = OrientedBox{cx, cy, yaw, lx / 2, ly / 2}.Corners();
    return AddPolygon(name, {corners.begin(), corners.end()}, zMin, zMax);
  }
  /** Vertical cylinder approximated by a regular polygon. */
  int AddCylinder(const std::string& name, double cx, double cy, double radius, double zMin = 0,
                  double zMax = std::numeric_limits<double>::infinity(), int sides = 16) {
    std::vector<Vec2> v;
    // Circumscribed so the flat faces sit at `radius`.
    const double rr = radius / std::cos(kPiObs / sides);
    for (int i = 0; i < sides; ++i) {
      const double a = 2 * kPiObs * i / sides;
      v.push_back({cx + rr * std::cos(a), cy + rr * std::sin(a)});
    }
    return AddPolygon(name, std::move(v), zMin, zMax);
  }
  void Clear() { prisms_.clear(); walls_.clear(); }
  const std::vector<Prism>& Prisms() const { return prisms_; }
  const std::vector<Wall>& Walls() const { return walls_; }
  /** Enable or disable a prism (e.g. a gate that opens); disabled prisms are ignored. */
  void SetEnabled(int index, bool enabled) {
    if (index >= 0 && static_cast<std::size_t>(index) < prisms_.size()) {
      if (disabled_.size() < prisms_.size()) disabled_.resize(prisms_.size(), false);
      disabled_[static_cast<std::size_t>(index)] = !enabled;
    }
  }
  bool Enabled(std::size_t index) const { return index >= disabled_.size() || !disabled_[index]; }

  /** Penetration of a convex body polygon (CCW) spanning [zLo, zHi] into prism i. */
  Penetration BodyVsPrism(const std::vector<Vec2>& body, double zLo, double zHi, std::size_t i, Vec2 hint = {0, 0}) const {
    const Prism& p = prisms_[i];
    if (!Enabled(i) || p.zMin >= zHi || p.zMax <= zLo) return {};
    double bx0 = 1e300, by0 = 1e300, bx1 = -1e300, by1 = -1e300;
    for (const auto& v : body) {
      bx0 = std::min(bx0, v[0]); bx1 = std::max(bx1, v[0]);
      by0 = std::min(by0, v[1]); by1 = std::max(by1, v[1]);
    }
    if (bx1 < p.minX || bx0 > p.maxX || by1 < p.minY || by0 > p.maxY) return {};
    return geometry::Polygons(body, p.vertices, hint);
  }
  /** Penetration of a body polygon through perimeter wall w (deepest corner). */
  Penetration BodyVsWall(const std::vector<Vec2>& body, std::size_t w) const {
    const Wall& wall = walls_[w];
    Penetration out;
    double sum = 0, px = 0, py = 0;
    for (const auto& v : body) {
      const double d = -geometry::Dot({v[0] - wall.point[0], v[1] - wall.point[1]}, wall.inward);
      if (d > 0) {
        out.hit = true;
        out.depth = std::max(out.depth, d);
        sum += d; px += d * v[0]; py += d * v[1];
      }
    }
    if (out.hit) {
      out.normal = wall.inward;
      out.point = {px / sum, py / sum};
    }
    return out;
  }
  /** Sphere (centre x, y, z, radius r) vs prism i: depth > 0 overlapping, 3D push-out normal. */
  SurfaceContact SphereVsPrism(double x, double y, double z, double r, std::size_t i) const {
    const Prism& p = prisms_[i];
    SurfaceContact c;
    if (!Enabled(i) || x + r < p.minX || x - r > p.maxX || y + r < p.minY || y - r > p.maxY ||
        z + r < p.zMin || z - r > p.zMax)
      return c;
    bool inside;
    Vec2 edgeNormal;
    const Vec2 q = geometry::ClosestOnBoundary(p.vertices, {x, y}, inside, edgeNormal);
    if (inside && z >= p.zMin && z <= p.zMax) {
      // Centre inside: leave through the nearest side, top or bottom.
      const double side = std::hypot(x - q[0], y - q[1]), top = p.zMax - z, bottom = z - p.zMin;
      if (top <= side && top <= bottom) { c.depth = r + top; c.normal = {0, 0, 1}; c.point = {x, y, p.zMax}; }
      else if (bottom <= side) { c.depth = r + bottom; c.normal = {0, 0, -1}; c.point = {x, y, p.zMin}; }
      else { c.depth = r + side; c.normal = {edgeNormal[0], edgeNormal[1], 0}; c.point = {q[0], q[1], z}; }
      return c;
    }
    const Vec2 h = inside ? Vec2{x, y} : q;
    const double cz = std::clamp(z, p.zMin, p.zMax);
    double dx = x - h[0], dy = y - h[1], dz = z - cz;
    const double dist = std::sqrt(dx * dx + dy * dy + dz * dz);
    if (dist < 1e-12) { c.depth = r; c.normal = {edgeNormal[0], edgeNormal[1], 0}; }
    else { c.depth = r - dist; c.normal = {dx / dist, dy / dist, dz / dist}; }
    c.point = {h[0], h[1], cz};
    return c;
  }
  /** Sphere vs perimeter wall w (only below the wall top). */
  SurfaceContact SphereVsWall(double x, double y, double z, double r, std::size_t w) const {
    const Wall& wall = walls_[w];
    SurfaceContact c;
    if (z - r > wall.height) return c;
    const double d = geometry::Dot({x - wall.point[0], y - wall.point[1]}, wall.inward);
    c.depth = r - d;
    c.normal = {wall.inward[0], wall.inward[1], 0};
    c.point = {x - d * wall.inward[0], y - d * wall.inward[1], z};
    return c;
  }

 private:
  static constexpr double kPiObs = 3.14159265358979323846;
  std::vector<Prism> prisms_;
  std::vector<Wall> walls_;
  std::vector<bool> disabled_;
};

}  // namespace mayhem::sim
