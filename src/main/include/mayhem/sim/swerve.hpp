#pragma once
// mayhem::sim swerve drivetrain plant: a 6-DOF rigid chassis on per-wheel terrain contacts.
// Header-only, C++17, no dependencies.
//
// Model (all world-frame SI; robot frame x forward, y left, z up):
// - Chassis: rigid body with mass, a diagonal inertia (roll, pitch, yaw) about the centre of
//   mass, gravity along -z. Semi-implicit Euler at an internal sub-step chosen from the
//   stiffnesses (typically 2-4 per 1 ms Step).
// - Wheels: each module's wheel is a sphere of the wheel radius at the module location,
//   cgHeight - wheelRadius below the centre of mass. Terrain::SphereContact gives depth and
//   normal; the normal force is a stiff spring-damper (carpet/tread compliance), clamped >= 0,
//   so a wheel can lift off and loads redistribute by themselves (load transfer under
//   acceleration, a side climbing a bump, etc.).
// - Tires: forces act in the local terrain plane. The drive direction (steer angle in the
//   chassis) is projected onto that plane. Friction is elasto-plastic: a tangential "bristle"
//   spring holds true static friction up to mu_s * N (a parked or X-locked robot does not
//   creep), and above it the contact slides with a Stribeck curve
//   mu(s) = mu_k + (mu_s - mu_k) exp(-|s| / slipVelocity) on the combined (longitudinal +
//   lateral) slip velocity, i.e. a friction circle. The wheel's spin is integrated implicitly
//   against the tire and motor damping, so it is stable at any sub-step.
// - Drive: TalonFx torque * driveGearing * driveEfficiency at the wheel; rolling resistance
//   (rollingResistance * N); the coupling ratio makes azimuth rotation turn the drive rotor
//   (rotor turns = driveGearing * wheel turns + couplingRatio * azimuth turns) and feeds the
//   drive torque back into the azimuth, as on a real coaxial module.
// - Steer: azimuth inertia driven by steer TalonFx torque * steerGearing, with tread scrub
//   (Coulomb, mu_s * N * scrubRadius).
// - Chassis contacts: the belly pan (a grid over the frame footprint at frameClearanceM) and
//   the bumpers (their bottom and top edges) are sampled against the Terrain with penalty
//   springs and sliding friction, so a robot can scrape or high-centre ("beach") on a crest
//   with its wheels unloaded, or catch its bumper on a ramp face. The bumper rectangle is also
//   collided with Obstacles (walls and prisms whose height range overlaps the robot), with
//   penalty spring-dampers and Coulomb friction; ContactEvents record each obstacle touched.
//
// Motors: DriveMotor(i) / SteerMotor(i) are TalonFx emulators that the plant steps once per
// Step() (the 1 kHz device rate) and samples between sub-steps with TalonFx::TorqueAt. Their
// "device frame" is the plant frame: positive drive rolls the wheel forward along its steer
// angle, positive steer turns the azimuth counter-clockwise (seen from above). Configure them
// as on the robot; for Phoenix's FusedCANcoder steer set rotorToSensorRatio = steerGearing so
// steer mechanism units are azimuth rotations (see swerve_requests.hpp).
//
// Pose conventions: Pose()/Pose3() report the robot origin (frame centre at floor level,
// AdvantageScope's robot origin) rather than the centre of mass. Roll/pitch/yaw follow WPILib's
// Rotation3d (right-handed about x, y, z; positive pitch = nose down).
#include "mayhem/sim/field.hpp"
#include "mayhem/sim/mechanisms.hpp"
#include "mayhem/sim/talonfx.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace mayhem::sim {

struct SwerveParams {
  double massKg = 60;
  double moiKgM2 = 6;     // yaw inertia about the centre of mass
  double cgHeightM = 0.2; // centre of mass above the floor
  std::vector<std::array<double, 2>> modulePositions;  // robot frame, from the frame centre
  double wheelRadius = 0.0508;
  double driveGearing = 6.75, steerGearing = 150.0 / 7.0, couplingRatio = 0;
  DcMotor driveMotor = DcMotor::KrakenX60(), steerMotor = DcMotor::KrakenX60();
  double wheelInertia = 0.004;  // kg*m^2 at the wheel: wheel + reflected drive rotor
  double steerInertia = 0.004;  // kg*m^2 at the azimuth: module + reflected steer rotor
  double muStatic = 1.2, muKinetic = 1.0, slipVelocity = 0.05;  // m/s, Stribeck decay
  double rollingResistance = 0.01, driveEfficiency = 0.97;
  double bumperLengthX = 0.9, bumperLengthY = 0.9;
  // Beyond the required set (0 means "derive a default"):
  DcMotor driveMotorFoc{}, steerMotorFoc{};  // FOC curves; stallCurrent 0 = same as above
  double comX = 0, comY = 0;           // centre of mass offset from the frame centre
  double rollMoiKgM2 = 0, pitchMoiKgM2 = 0;  // 0: solid box of the bumper footprint
  double frameLengthX = 0, frameLengthY = 0; // belly-pan footprint; 0: bumpers - 0.15 m
  double frameClearanceM = 0.041;      // belly pan above the wheel contact plane
  double bumperBottomM = 0.04, bumperTopM = 0.17, robotHeightM = 0.6;
  double tireNormalStiffness = 2e5;    // N/m per wheel (carpet + tread)
  double tireNormalDampingRatio = 0.6;
  double tireTangentialStiffness = 1e5;  // N/m per wheel, static-friction bristle
  double scrubRadius = 0.01;           // m, tread scrub torque arm when steering
  double framePointStiffness = 1e5;    // N/m per belly-pan / bumper sample
  double frameFriction = 0.3;          // belly pan (aluminium/polycarbonate) on carpet/HDPE
  double bumperStiffness = 1e6, bumperDampingRatio = 0.4, bumperFriction = 0.4;  // vs obstacles
  int frameSamplesX = 5, frameSamplesY = 5;
  double beachedFrameFraction = 0.25;  // belly pan carries > this share of the weight, or
  double beachedWheelFraction = 0.5;   // wheels carry < this share while the frame touches,
  double beachedTimeS = 0.2;           // for longer than this: Beached()
};

struct Pose2 { double x = 0, y = 0, yaw = 0; };
struct ChassisVelocity { double vx = 0, vy = 0, omega = 0; };
struct Pose3 { double x = 0, y = 0, z = 0, roll = 0, pitch = 0, yaw = 0; };

namespace vec {
inline Vec3 Add(Vec3 a, Vec3 b) { return {a[0] + b[0], a[1] + b[1], a[2] + b[2]}; }
inline Vec3 Sub(Vec3 a, Vec3 b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
inline Vec3 Scale(Vec3 a, double s) { return {a[0] * s, a[1] * s, a[2] * s}; }
inline double Dot(Vec3 a, Vec3 b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
inline Vec3 Cross(Vec3 a, Vec3 b) {
  return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
}
inline double Norm(Vec3 a) { return std::sqrt(Dot(a, a)); }
using Mat3 = std::array<double, 9>;  // row-major
inline Vec3 Mul(const Mat3& m, Vec3 v) {
  return {m[0] * v[0] + m[1] * v[1] + m[2] * v[2], m[3] * v[0] + m[4] * v[1] + m[5] * v[2],
          m[6] * v[0] + m[7] * v[1] + m[8] * v[2]};
}
inline Vec3 MulT(const Mat3& m, Vec3 v) {
  return {m[0] * v[0] + m[3] * v[1] + m[6] * v[2], m[1] * v[0] + m[4] * v[1] + m[7] * v[2],
          m[2] * v[0] + m[5] * v[1] + m[8] * v[2]};
}
struct Quat { double w = 1, x = 0, y = 0, z = 0; };
inline Mat3 ToMatrix(const Quat& q) {
  const double w = q.w, x = q.x, y = q.y, z = q.z;
  return {1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
          2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
          2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)};
}
/** q from WPILib-style roll, pitch, yaw (R = Rz(yaw) Ry(pitch) Rx(roll)). */
inline Quat FromEuler(double roll, double pitch, double yaw) {
  const double cr = std::cos(roll / 2), sr = std::sin(roll / 2), cp = std::cos(pitch / 2), sp = std::sin(pitch / 2);
  const double cy = std::cos(yaw / 2), sy = std::sin(yaw / 2);
  return {cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy};
}
/** Integrate a world-frame angular velocity over dt (exact rotation of |w| dt). */
inline Quat Integrate(const Quat& q, Vec3 w, double dt) {
  const double angle = Norm(w) * dt;
  if (angle < 1e-15) return q;
  const double s = std::sin(angle / 2) / Norm(w);
  const Quat d{std::cos(angle / 2), w[0] * s, w[1] * s, w[2] * s};
  Quat r{d.w * q.w - d.x * q.x - d.y * q.y - d.z * q.z, d.w * q.x + d.x * q.w + d.y * q.z - d.z * q.y,
         d.w * q.y - d.x * q.z + d.y * q.w + d.z * q.x, d.w * q.z + d.x * q.y - d.y * q.x + d.z * q.w};
  const double n = std::sqrt(r.w * r.w + r.x * r.x + r.y * r.y + r.z * r.z);
  return {r.w / n, r.x / n, r.y / n, r.z / n};
}
}  // namespace vec

class SwerveSim {
 public:
  struct ModuleState {
    double steerAngle = 0, steerRate = 0;   // rad, rad/s (continuous, robot frame)
    double wheelAngle = 0, wheelSpeed = 0;  // rad, rad/s
    double normalForce = 0;                 // N
    bool inContact = false;
    double slipLongitudinal = 0, slipLateral = 0;  // m/s, contact point vs tread
    bool sliding = false;
    Vec3 contactPoint{0, 0, 0}, force{0, 0, 0};    // world
  };

  explicit SwerveSim(SwerveParams params) : p_(std::move(params)) {
    if (p_.modulePositions.empty()) throw std::invalid_argument("SwerveParams.modulePositions is empty");
    if (p_.driveMotorFoc.stallCurrent <= 0) p_.driveMotorFoc = p_.driveMotor;
    if (p_.steerMotorFoc.stallCurrent <= 0) p_.steerMotorFoc = p_.steerMotor;
    const double lx = p_.bumperLengthX, ly = p_.bumperLengthY, h = std::max(p_.robotHeightM, 2 * p_.cgHeightM);
    if (p_.rollMoiKgM2 <= 0) p_.rollMoiKgM2 = p_.massKg * (ly * ly + h * h) / 12;
    if (p_.pitchMoiKgM2 <= 0) p_.pitchMoiKgM2 = p_.massKg * (lx * lx + h * h) / 12;
    if (p_.frameLengthX <= 0) p_.frameLengthX = std::max(0.1, lx - 0.15);
    if (p_.frameLengthY <= 0) p_.frameLengthY = std::max(0.1, ly - 0.15);
    for (std::size_t i = 0; i < p_.modulePositions.size(); ++i)
      modules_.push_back({TalonFx(p_.driveMotor, p_.driveMotorFoc), TalonFx(p_.steerMotor, p_.steerMotorFoc), {}, {}});
    BuildSamples();
    ChooseSubsteps();
    SetPose(0, 0, 0);
  }

  const SwerveParams& Params() const { return p_; }
  std::size_t ModuleCount() const { return modules_.size(); }
  TalonFx& DriveMotor(std::size_t i) { return modules_.at(i).drive; }
  TalonFx& SteerMotor(std::size_t i) { return modules_.at(i).steer; }
  const TalonFx& DriveMotor(std::size_t i) const { return modules_.at(i).drive; }
  const TalonFx& SteerMotor(std::size_t i) const { return modules_.at(i).steer; }
  /** Internal sub-steps per 1 ms (the sub-step length is fixed; longer Steps use more). */
  int Substeps() const { return substeps_; }

  /**
   * Teleport to (x, y, yaw) at rest, seated on the terrain: roll and pitch come from a plane
   * fitted through the terrain heights under the wheels.
   */
  void SetPose(double x, double y, double yaw, const Terrain* terrain = nullptr) {
    double ground = 0, slopeX = 0, slopeY = 0;  // plane z = ground + slopeX * u + slopeY * v (robot frame)
    if (terrain) {
      // Least squares over the module contact points (u, v robot frame).
      const double c = std::cos(yaw), s = std::sin(yaw);
      double A[3][3] = {}, b[3] = {};
      for (const auto& m : p_.modulePositions) {
        const double u = m[0], v = m[1];
        const double h = terrain->Height(x + c * u - s * v, y + s * u + c * v);
        const double row[3] = {1, u, v};
        for (int i = 0; i < 3; ++i) {
          b[i] += row[i] * h;
          for (int j = 0; j < 3; ++j) A[i][j] += row[i] * row[j];
        }
      }
      double sol[3];
      if (Solve3(A, b, sol)) { ground = sol[0]; slopeX = sol[1]; slopeY = sol[2]; }
      else ground = terrain->Height(x, y);
    }
    // Nose up (slopeX > 0) is negative pitch; left side up (slopeY > 0) is positive roll.
    const double pitch = -std::atan(slopeX), roll = std::atan(slopeY * std::cos(std::atan(slopeX)));
    q_ = vec::FromEuler(roll, pitch, yaw);
    const double sag = p_.massKg * kGravity / (p_.tireNormalStiffness * static_cast<double>(modules_.size()));
    const auto r = vec::ToMatrix(q_);
    pos_ = vec::Add({x, y, ground - sag}, vec::Mul(r, {p_.comX, p_.comY, p_.cgHeightM}));
    vel_ = omega_ = {0, 0, 0};
    for (auto& m : modules_) {
      m.state.wheelSpeed = m.state.steerRate = 0;
      m.anchor = {0, 0, 0};
    }
    unwrappedYaw_ = yaw;
    lastYaw_ = yaw;
    prevVel_ = vel_;
    accel_ = {0, 0, 0};
    beachedTimer_ = 0;
    beached_ = false;
  }
  /** Set the chassis velocity (field vx, vy in m/s, yaw rate), e.g. to start a test moving. */
  void SetVelocity(double vx, double vy, double omega) {
    vel_ = {vx, vy, 0};
    omega_ = {0, 0, omega};
  }
  /** Height of the robot's top above its floor reference (for overhead obstacles). */
  void SetRobotHeight(double meters) { p_.robotHeightM = meters; }
  double RobotHeight() const { return p_.robotHeightM; }

  /** Force (N) at a world point (z = centre-of-mass height), applied during the next Step. */
  void ApplyExternalForce(double fx, double fy, double px, double py) {
    ApplyExternalForce3(fx, fy, 0, px, py, pos_[2]);
  }
  void ApplyExternalForce3(double fx, double fy, double fz, double px, double py, double pz) {
    const Vec3 f{fx, fy, fz};
    extForce_ = vec::Add(extForce_, f);
    extTorque_ = vec::Add(extTorque_, vec::Cross(vec::Sub({px, py, pz}, pos_), f));
  }

  /**
   * Advance dt (typically 1 ms): steps every TalonFx once, then integrates the chassis over
   * Substeps() sub-steps. terrain/obstacles may be null (flat, open floor).
   */
  void Step(double dt, double busVoltage, const Terrain* terrain = nullptr, const Obstacles* obstacles = nullptr) {
    supply_ = 0;
    for (auto& m : modules_) {
      auto& s = m.state;
      const double dRot = (p_.driveGearing * s.wheelAngle + p_.couplingRatio * s.steerAngle) / (2 * kPi);
      const double dRps = (p_.driveGearing * s.wheelSpeed + p_.couplingRatio * s.steerRate) / (2 * kPi);
      m.drive.Step(dt, busVoltage, dRot, dRps);
      m.steer.Step(dt, busVoltage, p_.steerGearing * s.steerAngle / (2 * kPi), p_.steerGearing * s.steerRate / (2 * kPi));
      supply_ += m.drive.SupplyCurrent() + m.steer.SupplyCurrent();
    }
    events_.clear();
    eventKeys_.clear();
    if (obstacles && hints_.size() != obstacles->Prisms().size()) hints_.assign(obstacles->Prisms().size(), Vec2{0, 0});
    // Sub-steps are at most maxSubstep_ long whatever dt is (the devices still update once per
    // call, so call at 1 ms for Phoenix-like closed loops).
    const int n = std::max(1, static_cast<int>(std::ceil(dt / maxSubstep_ - 1e-9)));
    const double h = dt / n;
    for (int k = 0; k < n; ++k) Substep(h, terrain, obstacles);
    extForce_ = extTorque_ = {0, 0, 0};
    // Gyro yaw unwrapping, accelerometer, beaching.
    const double yaw = Pose3().yaw;
    double dy = yaw - lastYaw_;
    dy -= 2 * kPi * std::floor((dy + kPi) / (2 * kPi));
    unwrappedYaw_ += dy;
    lastYaw_ = yaw;
    accel_ = dt > 0 ? vec::Scale(vec::Sub(vel_, prevVel_), 1 / dt) : Vec3{0, 0, 0};
    prevVel_ = vel_;
    const double weight = p_.massKg * kGravity;
    double wheels = 0;
    for (const auto& m : modules_) wheels += m.state.normalForce;
    const bool condition = frameNormal_ > p_.beachedFrameFraction * weight ||
                           (wheels < p_.beachedWheelFraction * weight && frameNormal_ > 0.05 * weight);
    beachedTimer_ = condition ? beachedTimer_ + dt : 0;
    beached_ = beachedTimer_ > p_.beachedTimeS;
    time_ += dt;
  }

  // --- Pose and motion -----------------------------------------------------------------
  /** Robot origin (frame centre at floor level) on the field, and heading. */
  Pose2 Pose() const { const auto p = Pose3(); return {p.x, p.y, p.yaw}; }
  mayhem::sim::Pose3 Pose3() const {
    const auto r = vec::ToMatrix(q_);
    const Vec3 o = vec::Add(pos_, vec::Mul(r, {-p_.comX, -p_.comY, -p_.cgHeightM}));
    mayhem::sim::Pose3 out;
    out.x = o[0]; out.y = o[1]; out.z = o[2];
    out.yaw = std::atan2(r[3], r[0]);
    out.pitch = std::asin(std::clamp(-r[6], -1.0, 1.0));
    out.roll = std::atan2(r[7], r[8]);
    return out;
  }
  /** Field-relative velocity of the robot origin (vx, vy) and yaw rate. */
  ChassisVelocity FieldVelocity() const {
    const auto r = vec::ToMatrix(q_);
    const Vec3 rel = vec::Mul(r, {-p_.comX, -p_.comY, -p_.cgHeightM});
    const Vec3 v = vec::Add(vel_, vec::Cross(omega_, rel));
    return {v[0], v[1], omega_[2]};
  }
  /** Robot-relative (vx forward, vy left, omega). */
  ChassisVelocity RobotVelocity() const {
    const auto f = FieldVelocity();
    const double yaw = Pose3().yaw, c = std::cos(yaw), s = std::sin(yaw);
    return {c * f.vx + s * f.vy, -s * f.vx + c * f.vy, f.omega};
  }
  Vec3 CenterOfMass() const { return pos_; }
  Vec3 LinearVelocity() const { return vel_; }
  Vec3 AngularVelocity() const { return omega_; }  // world frame
  Vec3 Acceleration() const { return accel_; }      // world frame, finite difference per Step
  double Time() const { return time_; }

  // --- Gyro (Pigeon2-like) -----------------------------------------------------------------
  /** Continuous (unwrapped) yaw, like Pigeon2 GetYaw (radians here). */
  double GyroYaw() const { return unwrappedYaw_; }
  double GyroPitch() const { return Pose3().pitch; }
  double GyroRoll() const { return Pose3().roll; }
  /** Body-frame angular rates (roll rate about x, pitch rate about y, yaw rate about z), rad/s. */
  Vec3 GyroRates() const { return vec::MulT(vec::ToMatrix(q_), omega_); }
  /** World-frame yaw rate (Pigeon2 AngularVelocityZWorld), rad/s. */
  double GyroYawRateWorld() const { return omega_[2]; }

  // --- Modules -----------------------------------------------------------------------------
  const ModuleState& Module(std::size_t i) const { return modules_.at(i).state; }
  double ModuleDrivePosition(std::size_t i) const { return modules_.at(i).state.wheelAngle * p_.wheelRadius; }
  double ModuleDriveVelocity(std::size_t i) const { return modules_.at(i).state.wheelSpeed * p_.wheelRadius; }
  double ModuleSteerAngle(std::size_t i) const { return modules_.at(i).state.steerAngle; }
  double ModuleSteerRate(std::size_t i) const { return modules_.at(i).state.steerRate; }
  double ModuleNormalForce(std::size_t i) const { return modules_.at(i).state.normalForce; }
  bool ModuleInContact(std::size_t i) const { return modules_.at(i).state.inContact; }
  /** Magnitude of the tread slip velocity (m/s). */
  double ModuleSlip(std::size_t i) const {
    const auto& s = modules_.at(i).state;
    return std::hypot(s.slipLongitudinal, s.slipLateral);
  }
  /** Set a module's azimuth (rad, robot frame), e.g. to match an absolute encoder offset. */
  void SetModuleAngle(std::size_t i, double radians) { modules_.at(i).state.steerAngle = radians; modules_.at(i).state.steerRate = 0; }

  // --- Contacts, electrical, state flags --------------------------------------------------
  double TotalSupplyCurrent() const { return supply_; }
  const std::vector<ContactEvent>& Contacts() const { return events_; }
  /** Total normal force on the belly pan and bumpers from the terrain (N), last sub-step. */
  double FrameContactForce() const { return frameNormal_; }
  bool FrameInContact() const { return frameNormal_ > 0; }
  /** High-centred: the frame carries the robot instead of the wheels for > beachedTimeS. */
  bool Beached() const { return beached_; }
  bool HighCentered() const { return beached_; }
  /** Angle between the robot's up axis and world up (rad). */
  double TiltAngle() const { return std::acos(std::clamp(vec::ToMatrix(q_)[8], -1.0, 1.0)); }
  /**
   * Signed distance (m) from the centre of mass, projected along gravity, to the edge of the
   * support polygon of wheels in contact (negative: outside, the robot is tipping).
   */
  double SupportMargin() const {
    std::vector<Vec2> pts;
    for (const auto& m : modules_) if (m.state.inContact) pts.push_back({m.state.contactPoint[0], m.state.contactPoint[1]});
    if (pts.size() < 3) return -1;
    const auto hull = Hull(pts);
    if (hull.size() < 3) return -1;
    double margin = 1e300;
    for (std::size_t i = 0; i < hull.size(); ++i) {
      const Vec2 a = hull[i], b = hull[(i + 1) % hull.size()];
      const double len = std::hypot(b[0] - a[0], b[1] - a[1]);
      if (len < 1e-9) continue;
      margin = std::min(margin, geometry::Cross({b[0] - a[0], b[1] - a[1]}, {pos_[0] - a[0], pos_[1] - a[1]}) / len);
    }
    return margin;
  }
  /** Tipped over: tilted more than 60 degrees. */
  bool TippedOver() const { return TiltAngle() > kPi / 3; }

 private:
  struct Mod {
    TalonFx drive, steer;
    ModuleState state;
    Vec3 anchor;  // static-friction bristle displacement (world, in the contact plane)
  };
  SwerveParams p_;
  std::vector<Mod> modules_;
  std::vector<Vec3> frameSamples_, bumperSamples_;  // body coordinates
  int substeps_ = 2;
  double maxSubstep_ = 0.0005;
  Vec3 pos_{0, 0, 0}, vel_{0, 0, 0}, omega_{0, 0, 0}, extForce_{0, 0, 0}, extTorque_{0, 0, 0};
  Vec3 prevVel_{0, 0, 0}, accel_{0, 0, 0};
  vec::Quat q_;
  double supply_ = 0, frameNormal_ = 0, unwrappedYaw_ = 0, lastYaw_ = 0, time_ = 0, beachedTimer_ = 0;
  bool beached_ = false;
  std::vector<ContactEvent> events_;
  std::vector<int> eventKeys_;
  std::vector<Vec2> hints_;

  static bool Solve3(double A[3][3], const double b[3], double x[3]) {
    const double det = A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) - A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
                       A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0]);
    if (std::abs(det) < 1e-12) return false;
    for (int k = 0; k < 3; ++k) {
      double M[3][3];
      for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) M[i][j] = j == k ? b[i] : A[i][j];
      x[k] = (M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) - M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) +
              M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0])) / det;
    }
    return true;
  }

  static std::vector<Vec2> Hull(std::vector<Vec2> pts) {
    std::sort(pts.begin(), pts.end());
    std::vector<Vec2> h;
    for (int pass = 0; pass < 2; ++pass) {
      const std::size_t start = h.size();
      for (const auto& p : pts) {
        while (h.size() >= start + 2 &&
               geometry::Cross({h[h.size() - 1][0] - h[h.size() - 2][0], h[h.size() - 1][1] - h[h.size() - 2][1]},
                               {p[0] - h[h.size() - 2][0], p[1] - h[h.size() - 2][1]}) <= 0)
          h.pop_back();
        h.push_back(p);
      }
      h.pop_back();
      std::reverse(pts.begin(), pts.end());
    }
    return h;
  }

  void BuildSamples() {
    const double zFrame = -p_.cgHeightM + p_.frameClearanceM;
    const int nx = std::max(2, p_.frameSamplesX), ny = std::max(2, p_.frameSamplesY);
    for (int i = 0; i < nx; ++i)
      for (int j = 0; j < ny; ++j)
        frameSamples_.push_back({-p_.comX + p_.frameLengthX * (static_cast<double>(i) / (nx - 1) - 0.5),
                                 -p_.comY + p_.frameLengthY * (static_cast<double>(j) / (ny - 1) - 0.5), zFrame});
    // Bumper perimeter, bottom and top edges, about every 10 cm.
    const double hx = p_.bumperLengthX / 2, hy = p_.bumperLengthY / 2;
    for (double z : {p_.bumperBottomM, p_.bumperTopM}) {
      const double bz = -p_.cgHeightM + z;
      const int sx = std::max(2, static_cast<int>(std::ceil(p_.bumperLengthX / 0.1))) ;
      const int sy = std::max(2, static_cast<int>(std::ceil(p_.bumperLengthY / 0.1)));
      for (int i = 0; i <= sx; ++i) {
        const double x = -hx + p_.bumperLengthX * i / sx;
        bumperSamples_.push_back({x - p_.comX, -hy - p_.comY, bz});
        bumperSamples_.push_back({x - p_.comX, hy - p_.comY, bz});
      }
      for (int j = 1; j < sy; ++j) {
        const double y = -hy + p_.bumperLengthY * j / sy;
        bumperSamples_.push_back({-hx - p_.comX, y - p_.comY, bz});
        bumperSamples_.push_back({hx - p_.comX, y - p_.comY, bz});
      }
    }
  }

  void ChooseSubsteps() {
    // Keep h * (fastest explicit rate) small for: tire bristles + damping on the chassis
    // (translation and yaw), wheel/frame/bumper springs.
    const double n = static_cast<double>(modules_.size());
    double rMax2 = 0;
    for (const auto& m : p_.modulePositions) rMax2 = std::max(rMax2, m[0] * m[0] + m[1] * m[1]);
    const double perWheelMass = p_.massKg / n;
    const double ct = 2 * 0.5 * std::sqrt(p_.tireTangentialStiffness * perWheelMass);
    const double rate = n * ct * std::max(1 / p_.massKg, rMax2 / p_.moiKgM2);
    const double wSpring = std::sqrt(std::max({n * p_.tireNormalStiffness, n * p_.tireTangentialStiffness * std::max(1.0, rMax2 * p_.massKg / p_.moiKgM2),
                                               p_.bumperStiffness, 9 * p_.framePointStiffness}) / p_.massKg);
    const double dt = 0.001;
    const int a = static_cast<int>(std::ceil(dt * rate / 0.25));
    const int b = static_cast<int>(std::ceil(dt * wSpring / 0.15));
    substeps_ = std::clamp(std::max({a, b, 2}), 2, 20);
    maxSubstep_ = dt / substeps_;
  }

  double Mu(double slip) const { return p_.muKinetic + (p_.muStatic - p_.muKinetic) * std::exp(-slip / p_.slipVelocity); }

  void AddEvent(int key, const std::string& name, double impulse, Vec3 at) {
    for (std::size_t i = 0; i < eventKeys_.size(); ++i)
      if (eventKeys_[i] == key) {
        events_[i].impulse += impulse;
        events_[i].x = at[0]; events_[i].y = at[1]; events_[i].z = at[2];
        return;
      }
    eventKeys_.push_back(key);
    events_.push_back({name, impulse, at[0], at[1], at[2]});
  }

  void Substep(double h, const Terrain* terrain, const Obstacles* obstacles) {
    using namespace vec;
    const Mat3 r = ToMatrix(q_);
    Vec3 force = Add({0, 0, -p_.massKg * kGravity}, extForce_);
    Vec3 torque = extTorque_;
    const double n = static_cast<double>(modules_.size());
    const double perWheelMass = p_.massKg / n;
    const double weight = p_.massKg * kGravity;
    const double kn = p_.tireNormalStiffness, cn = 2 * p_.tireNormalDampingRatio * std::sqrt(kn * perWheelMass);
    const double kt = p_.tireTangentialStiffness, ct = 2 * 0.5 * std::sqrt(kt * perWheelMass);
    const double R = p_.wheelRadius;

    for (std::size_t i = 0; i < modules_.size(); ++i) {
      Mod& m = modules_[i];
      ModuleState& s = m.state;
      const auto& mp = p_.modulePositions[i];
      const Vec3 rb{mp[0] - p_.comX, mp[1] - p_.comY, -p_.cgHeightM + R};
      const Vec3 c = Add(pos_, Mul(r, rb));
      SurfaceContact sc;
      if (terrain) sc = terrain->SphereContact(c[0], c[1], c[2], R);
      else { sc.depth = R - c[2]; sc.normal = {0, 0, 1}; }
      const Vec3 nrm = sc.normal;
      const Vec3 cp = Sub(c, Scale(nrm, R));
      const Vec3 rel = Sub(cp, pos_);
      const Vec3 vcp = Add(vel_, Cross(omega_, rel));
      double normal = 0;
      // Spring capped at kMaxWheelLoad x weight so a teleport into the terrain cannot explode.
      if (sc.depth > 0)
        normal = std::clamp(std::min(kn * sc.depth, kMaxWheelLoad * weight) - cn * Dot(vcp, nrm), 0.0, kMaxWheelLoad * weight);
      // Tread directions in the terrain plane.
      const Vec3 heading = Mul(r, {std::cos(s.steerAngle), std::sin(s.steerAngle), 0});
      Vec3 el = Sub(heading, Scale(nrm, Dot(heading, nrm)));
      const double elLen = Norm(el);
      if (elLen < 1e-6) normal = 0;
      el = elLen > 1e-6 ? Scale(el, 1 / elLen) : Vec3{1, 0, 0};
      const Vec3 et = Cross(nrm, el);
      const double vl = Dot(vcp, el), vt = Dot(vcp, et);
      // Drive torque at the wheel.
      const double rotorRps = (p_.driveGearing * s.wheelSpeed + p_.couplingRatio * s.steerRate) / (2 * kPi);
      const double motorTorque = m.drive.TorqueAt(rotorRps);
      const double eta = p_.driveEfficiency, G = p_.driveGearing;
      const double wheelTorque = eta * G * motorTorque;
      const double wheelDamping = eta * G * G * m.drive.RotorDamping();
      const double rolling = -p_.rollingResistance * normal * R * std::tanh(s.wheelSpeed * R / 0.01);
      // Elasto-plastic tread contact.
      double zl = Dot(m.anchor, el), zt = Dot(m.anchor, et);
      double fl = 0, ft = 0;
      const double J = p_.wheelInertia;
      if (normal <= 0) {
        zl = zt = 0;
        s.wheelSpeed += h * (wheelTorque + rolling) / (J + h * wheelDamping);
        s.sliding = false;
        s.slipLongitudinal = vl - R * s.wheelSpeed;
        s.slipLateral = vt;
      } else {
        const double K = kt * h + ct;
        const double sl0 = vl - R * s.wheelSpeed;
        const double fl0 = -(kt * (zl + h * sl0) + ct * sl0);
        const double w1 = s.wheelSpeed + h * (wheelTorque - R * fl0 + rolling) / (J + h * wheelDamping + h * R * R * K);
        const double sl = vl - R * w1;
        fl = -(kt * (zl + h * sl) + ct * sl);
        ft = -(kt * (zt + h * vt) + ct * vt);
        const double slip0 = std::hypot(sl0, vt);
        const double fmax = Mu(slip0) * normal;
        const double fmag = std::hypot(fl, ft);
        if (fmag <= fmax) {
          zl += h * sl;
          zt += h * vt;
          s.wheelSpeed = w1;
          s.sliding = false;
          s.slipLongitudinal = sl;
        } else {
          double dl, dt2;
          if (slip0 > 1e-3) { dl = -sl0 / slip0; dt2 = -vt / slip0; }
          else { dl = fl / fmag; dt2 = ft / fmag; }
          fl = fmax * dl;
          ft = fmax * dt2;
          s.wheelSpeed += h * (wheelTorque - R * fl + rolling) / (J + h * wheelDamping);
          zl = -fl / kt;
          zt = -ft / kt;
          s.sliding = true;
          s.slipLongitudinal = vl - R * s.wheelSpeed;
        }
        s.slipLateral = vt;
      }
      m.anchor = Add(Scale(el, zl), Scale(et, zt));
      const Vec3 fc = Add(Scale(nrm, normal), Add(Scale(el, fl), Scale(et, ft)));
      force = Add(force, fc);
      torque = Add(torque, Cross(rel, fc));
      s.wheelAngle += h * s.wheelSpeed;
      s.normalForce = normal;
      s.inContact = normal > 0;
      s.contactPoint = cp;
      s.force = fc;
      // Azimuth.
      const double steerTorque = p_.steerGearing * m.steer.TorqueAt(p_.steerGearing * s.steerRate / (2 * kPi)) +
                                 p_.couplingRatio * motorTorque;
      const double steerDamping = m.steer.RotorDamping() * p_.steerGearing * p_.steerGearing;
      detail::StepDof(h, p_.steerInertia, steerTorque, steerDamping, 0, p_.muStatic * normal * p_.scrubRadius,
                      -kInf, kInf, 0, s.steerAngle, s.steerRate);
    }

    // Belly pan and bumper edges against the terrain.
    frameNormal_ = 0;
    const double kp = p_.framePointStiffness;
    const double cpd = 2 * 0.5 * std::sqrt(kp * p_.massKg / 4);
    auto samples = [&](const std::vector<Vec3>& pts, double mu) {
      for (const auto& b : pts) {
        const Vec3 w = Add(pos_, Mul(r, b));
        double ground = 0;
        Vec3 nrm{0, 0, 1};
        if (terrain) { ground = terrain->Height(w[0], w[1]); if (ground > w[2]) nrm = terrain->Normal(w[0], w[1]); }
        const double d = ground - w[2];
        if (d <= 0) continue;
        const Vec3 rel = Sub(w, pos_);
        const Vec3 vp = Add(vel_, Cross(omega_, rel));
        const double vn = Dot(vp, nrm);
        // Spring capped per sample; an overlap deeper than a real impact (a teleport into the
        // terrain) pushes out at no more than kDepenetrationSpeed.
        double spring = std::min(kp * d * nrm[2], weight);
        if (d > kDeepOverlap && vn > kDepenetrationSpeed) spring = 0;
        const double fn = spring - cpd * vn;
        if (fn <= 0) continue;
        const Vec3 vtan = Sub(vp, Scale(nrm, vn));
        const double speed = Norm(vtan);
        const Vec3 fc = Add(Scale(nrm, fn), Scale(vtan, -mu * fn / std::sqrt(speed * speed + 1e-4)));
        force = Add(force, fc);
        torque = Add(torque, Cross(rel, fc));
        frameNormal_ += fn;
      }
    };
    samples(frameSamples_, p_.frameFriction);
    samples(bumperSamples_, p_.bumperFriction);
    // Roof (at the current robot height): an overturned robot rests on its top instead of sinking.
    {
      const double hx = p_.bumperLengthX / 2, hy = p_.bumperLengthY / 2, z = -p_.cgHeightM + p_.robotHeightM;
      std::vector<Vec3> roof;
      for (double sx : {-1.0, 0.0, 1.0})
        for (double sy : {-1.0, 0.0, 1.0}) roof.push_back({sx * hx - p_.comX, sy * hy - p_.comY, z});
      samples(roof, p_.frameFriction);
    }

    // Bumpers against obstacles.
    Vec3 depenetrate{0, 0, 0};
    if (obstacles) {
      const Vec3 origin = Add(pos_, Mul(r, {-p_.comX, -p_.comY, -p_.cgHeightM}));
      const double yaw = std::atan2(r[3], r[0]);
      const auto corners = OrientedBox{origin[0], origin[1], yaw, p_.bumperLengthX / 2, p_.bumperLengthY / 2}.Corners();
      const std::vector<Vec2> body(corners.begin(), corners.end());
      const double zLo = origin[2] + p_.bumperBottomM, zHi = origin[2] + p_.robotHeightM;
      const double zc = origin[2] + 0.5 * (p_.bumperBottomM + p_.bumperTopM);
      const double kb = p_.bumperStiffness, cb = 2 * p_.bumperDampingRatio * std::sqrt(kb * p_.massKg);
      // Obstacle forces are horizontal only (never up from an overhead structure's underside).
      // A contact acts at the middle of the height overlap (the bumpers for a wall, the robot's
      // top edge for an overhead arm) with the spring capped at 3x weight. An overlap deeper than
      // a real impact can produce (a teleport into the obstacle) is instead pushed out gently at
      // the robot out kinematically (no force, no torque) at kDepenetrationSpeed, and its
      // velocity into the obstacle is removed, so it always resolves without a launch.
      auto respond = [&](const Penetration& pen, int key, const std::string& name, double overlapLo, double overlapHi) {
        const Vec3 at{pen.point[0], pen.point[1], 0.5 * (overlapLo + overlapHi)};
        if (pen.depth > kDeepOverlap) {
          const double push = std::min(pen.depth - 0.5 * kDeepOverlap, kDepenetrationSpeed * h);
          depenetrate[0] += push * pen.normal[0];
          depenetrate[1] += push * pen.normal[1];
          const double vIn = vel_[0] * pen.normal[0] + vel_[1] * pen.normal[1];
          if (vIn < 0) { vel_[0] -= vIn * pen.normal[0]; vel_[1] -= vIn * pen.normal[1]; }
          AddEvent(key, name, 0, at);
          return;
        }
        const Vec3 rel = Sub(at, pos_);
        const Vec3 vp = Add(vel_, Cross(omega_, rel));
        const double vn = vp[0] * pen.normal[0] + vp[1] * pen.normal[1];
        const double fn = std::min(std::min(kb * pen.depth, 3 * weight) - cb * vn, 3 * weight);
        if (fn <= 0) return;
        const Vec2 t{-pen.normal[1], pen.normal[0]};
        const double vtan = vp[0] * t[0] + vp[1] * t[1];
        const double ftan = -p_.bumperFriction * fn * vtan / std::sqrt(vtan * vtan + 1e-4);
        const Vec3 fc{fn * pen.normal[0] + ftan * t[0], fn * pen.normal[1] + ftan * t[1], 0};
        force = Add(force, fc);
        torque = Add(torque, Cross(rel, fc));
        AddEvent(key, name, fn * h, at);
      };
      for (std::size_t w = 0; w < obstacles->Walls().size(); ++w) {
        const auto pen = obstacles->BodyVsWall(body, w);
        if (pen.hit) respond(pen, -1 - static_cast<int>(w), obstacles->Walls()[w].name, zLo, zc + (zc - zLo));
      }
      for (std::size_t i = 0; i < obstacles->Prisms().size(); ++i) {
        const auto pen = obstacles->BodyVsPrism(body, zLo, zHi, i, hints_[i]);
        if (pen.hit) {
          hints_[i] = pen.normal;
          const auto& prism = obstacles->Prisms()[i];
          // Reaching down to the bumpers: contact at bumper height; overhead: the overlap's middle.
          const double topOfBumper = origin[2] + p_.bumperTopM;
          if (prism.zMin <= topOfBumper) respond(pen, static_cast<int>(i), prism.name, zLo, zc + (zc - zLo));
          else respond(pen, static_cast<int>(i), prism.name, std::max(zLo, prism.zMin), std::min(zHi, prism.zMax));
        } else {
          hints_[i] = {0, 0};
        }
      }
    }

    // Integrate the chassis.
    {
      const double len = std::hypot(depenetrate[0], depenetrate[1]), cap = kDepenetrationSpeed * h;
      if (len > cap) depenetrate = Scale(depenetrate, cap / len);
    }
    vel_ = Add(vel_, Scale(force, h / p_.massKg));
    const Vec3 inv{1 / p_.rollMoiKgM2, 1 / p_.pitchMoiKgM2, 1 / p_.moiKgM2};
    const Vec3 I{p_.rollMoiKgM2, p_.pitchMoiKgM2, p_.moiKgM2};
    const Vec3 wb = MulT(r, omega_);
    const Vec3 Lb{I[0] * wb[0], I[1] * wb[1], I[2] * wb[2]};
    const Vec3 tb = Sub(MulT(r, torque), Cross(wb, Lb));
    const Vec3 wbNew{wb[0] + h * tb[0] * inv[0], wb[1] + h * tb[1] * inv[1], wb[2] + h * tb[2] * inv[2]};
    omega_ = Mul(r, wbNew);
    pos_ = Add(Add(pos_, Scale(vel_, h)), depenetrate);
    q_ = Integrate(q_, omega_, h);
  }

  static constexpr double kInf = std::numeric_limits<double>::infinity();
  static constexpr double kMaxWheelLoad = 8;          // x weight, per wheel spring
  static constexpr double kDeepOverlap = 0.03;        // m: deeper than any real impact
  static constexpr double kDepenetrationSpeed = 0.5;  // m/s: max push-out speed from a deep overlap
};

/**
 * Bumper-to-bumper contact between two robots (penalty spring-damper with Coulomb friction, the
 * same model as obstacles). Call once per 1 ms before both robots' Step; the forces are applied
 * through ApplyExternalForce and both robots record a ContactEvent named `nameOfB` / `nameOfA`
 * only in this return value (true while touching).
 */
inline bool CollideRobots(SwerveSim& a, SwerveSim& b, double stiffness = 1e6, double dampingRatio = 0.4,
                          double friction = 0.4) {
  const auto pa = a.Pose3(), pb = b.Pose3();
  const auto& qa = a.Params();
  const auto& qb = b.Params();
  // Height ranges must overlap (a robot on a platform can sit above another's bumpers).
  if (pa.z + qa.bumperBottomM >= pb.z + b.RobotHeight() || pb.z + qb.bumperBottomM >= pa.z + a.RobotHeight()) return false;
  const auto ca = OrientedBox{pa.x, pa.y, pa.yaw, qa.bumperLengthX / 2, qa.bumperLengthY / 2}.Corners();
  const auto cb = OrientedBox{pb.x, pb.y, pb.yaw, qb.bumperLengthX / 2, qb.bumperLengthY / 2}.Corners();
  const Penetration pen = geometry::Polygons({ca.begin(), ca.end()}, {cb.begin(), cb.end()});
  if (!pen.hit) return false;
  auto pointVelocity = [](const SwerveSim& s, const Pose3& p, Vec2 at) {
    const auto v = s.FieldVelocity();
    return Vec2{v.vx - v.omega * (at[1] - p.y), v.vy + v.omega * (at[0] - p.x)};
  };
  const Vec2 va = pointVelocity(a, pa, pen.point), vb = pointVelocity(b, pb, pen.point);
  const Vec2 rel{va[0] - vb[0], va[1] - vb[1]};
  const double vn = rel[0] * pen.normal[0] + rel[1] * pen.normal[1];
  const double reduced = qa.massKg * qb.massKg / (qa.massKg + qb.massKg);
  // Same robustness as obstacles: spring capped at 3x the lighter robot's weight; an overlap
  // deeper than a real impact (robots teleported into each other) separates at <= 0.5 m/s.
  const double cap = 3 * std::min(qa.massKg, qb.massKg) * kGravity;
  double spring = std::min(stiffness * pen.depth, cap);
  if (pen.depth > 0.03) spring = vn < 0.5 ? cap * std::min(1.0, (0.5 - vn) / 0.5) : 0;
  const double fn = std::min(spring - 2 * dampingRatio * std::sqrt(stiffness * reduced) * vn, cap);
  if (fn <= 0) return true;
  const Vec2 t{-pen.normal[1], pen.normal[0]};
  const double vt = rel[0] * t[0] + rel[1] * t[1];
  const double ft = -friction * fn * vt / std::sqrt(vt * vt + 1e-4);
  const double fx = fn * pen.normal[0] + ft * t[0], fy = fn * pen.normal[1] + ft * t[1];
  a.ApplyExternalForce(fx, fy, pen.point[0], pen.point[1]);
  b.ApplyExternalForce(-fx, -fy, pen.point[0], pen.point[1]);
  return true;
}

}  // namespace mayhem::sim
