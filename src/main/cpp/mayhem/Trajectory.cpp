// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "mayhem/Trajectory.h"

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <unordered_map>

#include <frc/Filesystem.h>
#include <wpi/json.h>

namespace mayhem {

namespace {
constexpr double kMustHitEps = 1e-3;
}

FieldSymmetry FieldSymmetryFromString(const std::string& s) {
  std::string lower = s;
  std::transform(lower.begin(), lower.end(), lower.begin(), ::tolower);
  return lower == "mirror" ? FieldSymmetry::MIRROR : FieldSymmetry::ROTATIONAL;
}

// --------------------------------------------------------------------------------------- sample
TrajectorySample TrajectorySample::Flipped(FieldSymmetry sym, double length, double width) const {
  TrajectorySample s;
  s.t = t;
  s.x = Symmetry::FlipX(sym, x, length);
  s.y = Symmetry::FlipY(sym, y, width);
  s.heading = Symmetry::FlipHeading(sym, heading);
  auto v = Symmetry::FlipVector(sym, vx, vy);
  s.vx = v[0];
  s.vy = v[1];
  s.omega = Symmetry::FlipAngular(sym, omega);
  auto a = Symmetry::FlipVector(sym, ax, ay);
  s.ax = a[0];
  s.ay = a[1];
  s.alpha = Symmetry::FlipAngular(sym, alpha);
  s.nForces = nForces;
  for (int i = 0; i < nForces; i++) {
    auto f = Symmetry::FlipVector(sym, fx[i], fy[i]);
    s.fx[i] = f[0];
    s.fy[i] = f[1];
  }
  if (sym == FieldSymmetry::MIRROR && nForces == 4) {
    // Mirroring swaps left and right modules: FL<->FR, BL<->BR.
    std::swap(s.fx[0], s.fx[1]);
    std::swap(s.fx[2], s.fx[3]);
    std::swap(s.fy[0], s.fy[1]);
    std::swap(s.fy[2], s.fy[3]);
  }
  return s;
}

TrajectorySample TrajectorySample::Integrate(const TrajectorySample& start,
                                             const TrajectorySample& end, double t) {
  const double tau = t - start.t;
  const double span = end.t - start.t;
  const double u = span > 1e-9 ? std::min(std::max(tau / span, 0.0), 1.0) : 0;
  TrajectorySample s;
  s.t = t;
  s.x = start.x + start.vx * tau + 0.5 * start.ax * tau * tau;
  s.y = start.y + start.vy * tau + 0.5 * start.ay * tau * tau;
  s.heading = start.heading + start.omega * tau + 0.5 * start.alpha * tau * tau;
  s.vx = start.vx + start.ax * tau;
  s.vy = start.vy + start.ay * tau;
  s.omega = start.omega + start.alpha * tau;
  s.ax = start.ax;
  s.ay = start.ay;
  s.alpha = start.alpha;
  s.nForces = std::min(start.nForces, end.nForces);
  for (int i = 0; i < s.nForces; i++) {
    s.fx[i] = start.fx[i] + (end.fx[i] - start.fx[i]) * u;
    s.fy[i] = start.fy[i] + (end.fy[i] - start.fy[i]) * u;
  }
  return s;
}

// ---------------------------------------------------------------------------------------- event
TrajectoryEvent::RecoveryPolicy TrajectoryEvent::PolicyFromString(const std::string& s) {
  if (s == "fireImmediately") {
    return RecoveryPolicy::FIRE_IMMEDIATELY;
  }
  if (s == "skip") {
    return RecoveryPolicy::SKIP;
  }
  return RecoveryPolicy::FIRE_AT_JOIN;
}

TrajectoryEvent::TrajectoryEvent(std::string name_, std::string command_, double t_, double endT_,
                                 RecoveryPolicy policy_, bool mustHit_)
    : name{std::move(name_)},
      command{command_.empty() ? name : std::move(command_)},
      t{t_},
      endT{endT_},
      policy{policy_},
      mustHit{mustHit_} {}

// -------------------------------------------------------------------------------------- polygon
ConvexPolygon::ConvexPolygon(std::vector<double> xs, std::vector<double> ys) {
  if (xs.size() != ys.size() || xs.size() < 3) {
    throw std::invalid_argument("A polygon needs at least 3 vertices");
  }
  // enforce CCW
  double area2 = 0;
  for (size_t i = 0; i < xs.size(); i++) {
    size_t j = (i + 1) % xs.size();
    area2 += xs[i] * ys[j] - xs[j] * ys[i];
  }
  if (area2 < 0) {
    std::reverse(xs.begin(), xs.end());
    std::reverse(ys.begin(), ys.end());
  }
  m_xs = std::move(xs);
  m_ys = std::move(ys);
}

bool ConvexPolygon::Contains(double px, double py) const {
  const size_t n = m_xs.size();
  for (size_t i = 0; i < n; i++) {
    size_t j = (i + 1) % n;
    double cross = (m_xs[j] - m_xs[i]) * (py - m_ys[i]) - (m_ys[j] - m_ys[i]) * (px - m_xs[i]);
    if (cross < 0) {
      return false;
    }
  }
  return true;
}

ConvexPolygon ConvexPolygon::Flipped(FieldSymmetry sym, double length, double width) const {
  std::vector<double> nx(m_xs.size());
  std::vector<double> ny(m_ys.size());
  for (size_t i = 0; i < m_xs.size(); i++) {
    nx[i] = Symmetry::FlipX(sym, m_xs[i], length);
    ny[i] = Symmetry::FlipY(sym, m_ys[i], width);
  }
  return ConvexPolygon{std::move(nx), std::move(ny)};  // constructor restores CCW order
}

RecoveryData RecoveryData::Flipped() const {
  RecoveryData r = *this;
  r.obstacles.clear();
  for (const auto& p : obstacles) {
    r.obstacles.push_back(p.Flipped(symmetry, fieldLength, fieldWidth));
  }
  r.roadmapNodes.clear();
  for (const auto& n : roadmapNodes) {
    r.roadmapNodes.emplace_back(
        units::meter_t{Symmetry::FlipX(symmetry, n.X().value(), fieldLength)},
        units::meter_t{Symmetry::FlipY(symmetry, n.Y().value(), fieldWidth)});
  }
  return r;
}

// ----------------------------------------------------------------------------------- trajectory
MayhemTrajectory::MayhemTrajectory(std::string name, std::vector<TrajectorySample> samples,
                                   std::vector<TrajectoryEvent> events,
                                   std::vector<TerrainSpan> terrain, std::vector<IntakeSpan> intake,
                                   std::vector<int> splits, std::vector<double> waypointTimes,
                                   std::shared_ptr<const RecoveryData> recovery,
                                   std::string inputHash, bool flipped)
    : m_name{std::move(name)},
      m_samples{std::move(samples)},
      m_events{std::move(events)},
      m_terrain{std::move(terrain)},
      m_intake{std::move(intake)},
      m_splits{std::move(splits)},
      m_waypointTimes{std::move(waypointTimes)},
      m_recovery{std::move(recovery)},
      m_inputHash{std::move(inputHash)},
      m_flipped{flipped} {
  if (m_samples.empty()) {
    throw std::invalid_argument("Trajectory '" + m_name + "' has no samples");
  }
}

const TerrainSpan* MayhemTrajectory::TerrainAt(double t) const {
  const TerrainSpan* best = nullptr;
  for (const auto& s : m_terrain) {
    if (s.t > t) {
      break;
    }
    if (s.Contains(t) && (best == nullptr || s.feedbackScale < best->feedbackScale)) {
      best = &s;
    }
  }
  return best;
}

bool MayhemTrajectory::IntakeExtendedAt(double t) const {
  for (const auto& s : m_intake) {
    if (s.Contains(t)) {
      return true;
    }
  }
  return false;
}

bool MayhemTrajectory::IntakeExtendedBetween(double a, double b) const {
  for (const auto& s : m_intake) {
    if (s.Overlaps(a, b)) {
      return true;
    }
  }
  return false;
}

TrajectorySample MayhemTrajectory::SampleAt(double t) const {
  const auto& first = m_samples.front();
  const auto& last = m_samples.back();
  if (t <= first.t) {
    return first.WithTime(t);
  }
  if (t >= last.t) {
    // hold the final pose, at rest if the trajectory ends stopped
    return last.WithTime(t);
  }
  size_t lo = 0;
  size_t hi = m_samples.size() - 1;
  while (hi - lo > 1) {
    size_t mid = (lo + hi) >> 1;
    if (m_samples[mid].t <= t) {
      lo = mid;
    } else {
      hi = mid;
    }
  }
  return TrajectorySample::Integrate(m_samples[lo], m_samples[hi], t);
}

std::shared_ptr<const MayhemTrajectory> MayhemTrajectory::Segment(int i) const {
  if (i < 0 || i >= SegmentCount()) {
    throw std::out_of_range("Segment " + std::to_string(i) + " of " +
                            std::to_string(SegmentCount()));
  }
  const int nSplits = static_cast<int>(m_splits.size());
  const int start = i == 0 ? 0 : m_splits[i - 1];
  const int end = i == nSplits ? static_cast<int>(m_samples.size()) - 1 : m_splits[i];
  const double t0 = m_samples[start].t;
  const double t1 = m_samples[end].t;
  std::vector<TrajectorySample> sub;
  for (int k = start; k <= end; k++) {
    sub.push_back(m_samples[k].WithTime(m_samples[k].t - t0));
  }
  std::vector<TrajectoryEvent> ev;
  const bool last = i == nSplits;
  for (const auto& e : m_events) {
    // A zone belongs to every segment it overlaps, even if its start was in an earlier one.
    if (e.IsZone() && e.endT > e.t && e.t < t1 - 1e-9 && e.endT > t0 + 1e-9) {
      ev.emplace_back(e.name, e.command, std::max(e.t, t0) - t0, std::min(e.endT, t1) - t0,
                      e.policy, e.mustHit);
      continue;
    }
    if (e.t >= t0 - 1e-9 && (e.t < t1 - 1e-9 || (last && e.t <= t1 + 1e-9))) {
      TrajectoryEvent s = e.Shifted(-t0);
      if (s.IsZone() && s.endT > t1 - t0) {
        s = TrajectoryEvent{s.name, s.command, s.t, t1 - t0, s.policy, s.mustHit};
      }
      ev.push_back(s);
    }
  }
  std::vector<TerrainSpan> ter;
  for (const auto& s : m_terrain) {
    const double a = std::max(s.t, t0);
    const double b = std::min(s.endT, t1);
    if (b > a) {
      ter.push_back({a - t0, b - t0, s.expectedSpeed, s.feedbackScale});
    }
  }
  std::vector<IntakeSpan> itk;
  for (const auto& s : m_intake) {
    const double a = std::max(s.t, t0);
    const double b = std::min(s.endT, t1);
    if (b >= a) {
      itk.push_back({a - t0, b - t0});
    }
  }
  std::vector<double> wt;
  for (double w : m_waypointTimes) {
    if (w >= t0 - 1e-9 && w <= t1 + 1e-9) {
      wt.push_back(w - t0);
    }
  }
  std::shared_ptr<const RecoveryData> rec = m_recovery;
  if (rec) {
    std::vector<double> mh;
    for (double m : rec->mustHitTimes) {
      if (m > t0 + kMustHitEps && m < t1 - kMustHitEps) {
        mh.push_back(m - t0);
      }
    }
    mh.push_back(t1 - t0);
    std::sort(mh.begin(), mh.end());
    mh.erase(std::unique(mh.begin(), mh.end()), mh.end());
    auto copy = std::make_shared<RecoveryData>(*rec);
    copy->mustHitTimes = std::move(mh);
    rec = copy;
  }
  return std::make_shared<MayhemTrajectory>(
      m_name + "[" + std::to_string(i) + "]", std::move(sub), std::move(ev), std::move(ter),
      std::move(itk), std::vector<int>{}, std::move(wt), rec, m_inputHash, m_flipped);
}

std::shared_ptr<const MayhemTrajectory> MayhemTrajectory::Flipped() const {
  if (m_flippedCache) {
    return m_flippedCache;
  }
  if (!m_recovery) {
    throw std::logic_error("Trajectory has no field data to flip with");
  }
  const FieldSymmetry sym = m_recovery->symmetry;
  std::vector<TrajectorySample> fs;
  fs.reserve(m_samples.size());
  for (const auto& s : m_samples) {
    fs.push_back(s.Flipped(sym, m_recovery->fieldLength, m_recovery->fieldWidth));
  }
  m_flippedCache = std::make_shared<MayhemTrajectory>(
      m_name, std::move(fs), m_events, m_terrain, m_intake, m_splits, m_waypointTimes,
      std::make_shared<RecoveryData>(m_recovery->Flipped()), m_inputHash, !m_flipped);
  return m_flippedCache;
}

// ------------------------------------------------------------------------------------- loader
namespace trajectory_loader {

namespace {
std::unordered_map<std::string, TrajectoryPtr>& Cache() {
  static std::unordered_map<std::string, TrajectoryPtr> cache;
  return cache;
}

double D(const wpi::json& node, const char* key, double def = 0.0) {
  auto it = node.find(key);
  if (it == node.end() || it->is_null()) {
    return def;
  }
  if (it->is_number()) {
    return it->get<double>();
  }
  if (it->is_boolean()) {
    return it->get<bool>() ? 1.0 : 0.0;
  }
  if (it->is_string()) {
    try {
      return std::stod(it->get<std::string>());
    } catch (...) {
      return def;
    }
  }
  return def;
}

std::string S(const wpi::json& node, const char* key, const std::string& def = "") {
  auto it = node.find(key);
  if (it == node.end() || it->is_null()) {
    return def;
  }
  if (it->is_string()) {
    return it->get<std::string>();
  }
  return it->dump();
}

const wpi::json& Path(const wpi::json& node, const char* key) {
  static const wpi::json kMissing;
  if (!node.is_object()) {
    return kMissing;
  }
  auto it = node.find(key);
  return it == node.end() ? kMissing : *it;
}

std::vector<double> Doubles(const wpi::json& arr) {
  std::vector<double> out;
  if (arr.is_array()) {
    for (const auto& v : arr) {
      out.push_back(v.is_number() ? v.get<double>() : 0.0);
    }
  }
  return out;
}

std::vector<int> Ints(const wpi::json& arr) {
  std::vector<int> out;
  if (arr.is_array()) {
    for (const auto& v : arr) {
      out.push_back(v.is_number() ? static_cast<int>(v.get<double>()) : 0);
    }
  }
  return out;
}

std::vector<frc::Translation2d> Points(const wpi::json& arr) {
  std::vector<frc::Translation2d> out;
  if (arr.is_array()) {
    for (const auto& p : arr) {
      out.emplace_back(units::meter_t{p.at(0).get<double>()},
                       units::meter_t{p.at(1).get<double>()});
    }
  }
  return out;
}

std::shared_ptr<const RecoveryData> ParseRecovery(const wpi::json& r) {
  if (r.is_null() || !r.is_object()) {
    return nullptr;
  }
  auto rec = std::make_shared<RecoveryData>();
  rec->bumper = Points(Path(r, "bumper"));
  const auto& obstacles = Path(r, "obstacles");
  if (obstacles.is_array()) {
    for (const auto& poly : obstacles) {
      auto pts = Points(poly);
      if (pts.size() >= 3) {
        std::vector<double> xs, ys;
        for (const auto& p : pts) {
          xs.push_back(p.X().value());
          ys.push_back(p.Y().value());
        }
        rec->obstacles.emplace_back(std::move(xs), std::move(ys));
      }
    }
  }
  const auto& edges = Path(r, "roadmapEdges");
  if (edges.is_array()) {
    for (const auto& e : edges) {
      rec->roadmapEdges.push_back(
          {static_cast<int>(e.at(0).get<double>()), static_cast<int>(e.at(1).get<double>())});
    }
  }
  const auto& lim = Path(r, "limits");
  rec->fieldLength = D(r, "fieldLength", 16.541);
  rec->fieldWidth = D(r, "fieldWidth", 8.069);
  rec->symmetry = FieldSymmetryFromString(S(r, "symmetry"));
  rec->roadmapNodes = Points(Path(r, "roadmapNodes"));
  rec->maxVelocity = D(lim, "maxVelocity", 3.0);
  rec->maxAcceleration = D(lim, "maxAcceleration", 4.0);
  rec->maxAngularVelocity = D(lim, "maxAngularVelocity", 6.0);
  rec->maxAngularAcceleration = D(lim, "maxAngularAcceleration", 10.0);
  rec->mustHitTimes = Doubles(Path(r, "mustHitTimes"));
  return rec;
}
}  // namespace

std::string DeployDirectory() {
  if (const char* env = std::getenv("ROBOT_DEPLOY_DIR"); env != nullptr && *env != '\0') {
    return env;
  }
  return frc::filesystem::GetDeployDirectory();
}

TrajectoryPtr Load(const std::string& name) {
  auto& cache = Cache();
  if (auto it = cache.find(name); it != cache.end()) {
    return it->second;
  }
  auto t = LoadFile(DeployDirectory() + "/" + kDeploySubdir + "/" + name + kExtension);
  cache.emplace(name, t);
  return t;
}

void ClearCache() { Cache().clear(); }

TrajectoryPtr LoadFile(const std::string& path) {
  std::ifstream in{path};
  if (!in) {
    throw std::invalid_argument("Trajectory file not found: " + path);
  }
  std::stringstream ss;
  ss << in.rdbuf();
  return Parse(ss.str());
}

TrajectoryPtr Parse(const std::string& json) {
  wpi::json root;
  try {
    root = wpi::json::parse(json);
  } catch (const std::exception& e) {
    throw std::invalid_argument(std::string{"Invalid trajectory JSON: "} + e.what());
  }
  const std::string name = S(root, "name", "unnamed");
  const int format = static_cast<int>(D(root, "formatVersion", 1));
  if (format > kSupportedFormat) {
    throw std::invalid_argument("Trajectory '" + name + "' uses format " + std::to_string(format) +
                                " but this MayhemLib supports up to " +
                                std::to_string(kSupportedFormat) + ". Update MayhemLib.");
  }
  const auto& out = Path(root, "output");
  if (out.is_null() || !out.is_object()) {
    throw std::invalid_argument("Trajectory '" + name + "' has not been generated yet.");
  }

  std::vector<TrajectorySample> samples;
  const auto& jsamples = Path(out, "samples");
  if (jsamples.is_array()) {
    samples.reserve(jsamples.size());
    for (const auto& s : jsamples) {
      TrajectorySample ts;
      ts.t = D(s, "t");
      ts.x = D(s, "x");
      ts.y = D(s, "y");
      ts.heading = D(s, "heading");
      ts.vx = D(s, "vx");
      ts.vy = D(s, "vy");
      ts.omega = D(s, "omega");
      ts.ax = D(s, "ax");
      ts.ay = D(s, "ay");
      ts.alpha = D(s, "alpha");
      auto fx = Doubles(Path(s, "fx"));
      auto fy = Doubles(Path(s, "fy"));
      ts.nForces =
          static_cast<int>(std::min({fx.size(), fy.size(), static_cast<size_t>(kMaxModules)}));
      // (Java kept both arrays at full length; the follower only uses them when there are exactly
      // as many as modules, which is preserved by nForces)
      if (fx.size() != fy.size() || fx.size() > static_cast<size_t>(kMaxModules)) {
        ts.nForces = 0;
      }
      for (int i = 0; i < ts.nForces; i++) {
        ts.fx[i] = fx[i];
        ts.fy[i] = fy[i];
      }
      samples.push_back(ts);
    }
  }

  std::vector<TrajectoryEvent> events;
  const auto& jevents = Path(out, "events");
  if (jevents.is_array()) {
    for (const auto& e : jevents) {
      const auto& end = Path(e, "endT");
      events.emplace_back(S(e, "name"), S(e, "command", ""), D(e, "t"),
                          end.is_number() ? end.get<double>() : kNaN,
                          TrajectoryEvent::PolicyFromString(S(e, "recoveryPolicy", "fireAtJoin")),
                          D(e, "mustHit", 0.0) != 0.0);
    }
  }
  std::vector<TerrainSpan> terrain;
  const auto& jterrain = Path(out, "terrain");
  if (jterrain.is_array()) {
    for (const auto& s : jterrain) {
      terrain.push_back(
          {D(s, "t"), D(s, "endT"), D(s, "expectedSpeed", 0.7), D(s, "feedbackScale", 0.3)});
    }
  }
  std::stable_sort(terrain.begin(), terrain.end(),
                   [](const TerrainSpan& a, const TerrainSpan& b) { return a.t < b.t; });
  std::vector<IntakeSpan> intake;
  const auto& jintake = Path(out, "intake");
  if (jintake.is_array()) {
    for (const auto& s : jintake) {
      intake.push_back({D(s, "t"), D(s, "endT")});
    }
  }
  std::stable_sort(intake.begin(), intake.end(),
                   [](const IntakeSpan& a, const IntakeSpan& b) { return a.t < b.t; });

  return std::make_shared<MayhemTrajectory>(
      name, std::move(samples), std::move(events), std::move(terrain), std::move(intake),
      Ints(Path(out, "splits")), Doubles(Path(out, "waypointTimes")),
      ParseRecovery(Path(out, "recovery")), S(out, "inputHash", ""), false);
}

}  // namespace trajectory_loader

}  // namespace mayhem
