#pragma once
// mayhem::sim generic 1-DOF plants: Flywheel, Arm, Elevator (and linear slides), Roller.
// Header-only, C++17, no dependencies.
//
// Each plant is driven by rotor torque from one or more TalonFx and exposes the rotor position
// (rot) and velocity (rot/s) to feed back into them:
//
//   double tau = talon.Step(dt, bus, arm.RotorPosition(), arm.RotorVelocity());
//   arm.Step(dt, tau, talon.RotorDamping());
//
// `gearing` is rotor rotations per mechanism rotation (> 1 is a reduction). The optional third
// argument, the motors' back-EMF damping at the rotor (N*m per rad/s, TalonFx::RotorDamping(),
// summed over motors), is integrated implicitly so a stiff motor on a light mechanism stays
// stable at 1 ms. Friction is implicit Coulomb (true sticking, no creep) and hard stops are
// implicit spring-dampers (or rigid when stiffness <= 0), so nothing explodes numerically.
// Inertias should include the motors' reflected rotor inertia (J_rotor * gearing^2).
#include "mayhem/sim/motor.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace mayhem::sim {

inline constexpr double kGravity = 9.80665;

namespace detail {

/**
 * One implicit step of J dw/dt = tau + (motor damping about the current w) - viscous w
 * - coulomb sign(w), with optional stops [lo, hi]. tau is the total torque evaluated at the
 * current state (motor torque already contains -motorDamping * w).
 */
inline void StepDof(double dt, double inertia, double tau, double motorDamping, double viscous,
                    double coulomb, double lo, double hi, double stiffness, double& pos, double& vel) {
  double p = inertia * vel + dt * (tau + motorDamping * vel);
  double d = inertia + dt * (motorDamping + viscous);
  if (coulomb > 0) {
    if (std::abs(p) <= dt * coulomb) p = 0;
    else p -= std::copysign(dt * coulomb, p);
  }
  double w = p / d;
  const double next = pos + dt * w;
  if (stiffness > 0 && (next > hi || next < lo)) {
    const double limit = next > hi ? hi : lo;
    const double c = 2.0 * 0.7 * std::sqrt(stiffness * inertia);  // damping ratio 0.7
    const double w2 = (p - dt * stiffness * (pos - limit)) / (d + dt * c + dt * dt * stiffness);
    const double force = -stiffness * (pos + dt * w2 - limit) - c * w2;
    // Only push away from the stop, never pull into it.
    if ((limit == hi && force < 0) || (limit == lo && force > 0)) w = w2;
  } else if (stiffness <= 0) {
    if (next > hi) { pos = hi; vel = std::min(0.0, w); return; }
    if (next < lo) { pos = lo; vel = std::max(0.0, w); return; }
  }
  vel = w;
  pos += dt * w;
}

}  // namespace detail

/** Spinning mass, e.g. a shooter wheel. Position in rad, velocity in rad/s (mechanism). */
struct Flywheel {
  double inertia = 0.002;     // kg*m^2 at the flywheel
  double gearing = 1;         // rotor rotations per flywheel rotation
  double viscous = 0;         // N*m per rad/s at the flywheel
  double coulombFriction = 0; // N*m at the flywheel

  void Step(double dt, double rotorTorque, double rotorDamping = 0) {
    detail::StepDof(dt, inertia, gearing * rotorTorque, rotorDamping * gearing * gearing, viscous,
                    coulombFriction, -kInf, kInf, 0, angle_, speed_);
  }
  /** Take energy out (J), e.g. a game piece leaving the wheel. Never reverses the wheel. */
  void RemoveEnergy(double joules) {
    const double e = std::max(0.0, 0.5 * inertia * speed_ * speed_ - joules);
    speed_ = std::copysign(std::sqrt(2 * e / inertia), speed_);
  }
  double Energy() const { return 0.5 * inertia * speed_ * speed_; }
  double Position() const { return angle_; }
  double Velocity() const { return speed_; }
  double RotorPosition() const { return angle_ * gearing / (2 * kPi); }
  double RotorVelocity() const { return speed_ * gearing / (2 * kPi); }
  void SetState(double angleRad, double speedRadPerSec) { angle_ = angleRad; speed_ = speedRadPerSec; }

  // Internal state: public only so the struct stays an aggregate ({inertia, gearing, ...}).
  static constexpr double kInf = std::numeric_limits<double>::infinity();
  double angle_ = 0, speed_ = 0;
};

/**
 * Pivoting arm. Angle in rad from horizontal, positive up, in the robot frame. Gravity acts on
 * angle + pitchOffset (SetPitchOffset: the robot's tilt in the arm's plane, positive raising the
 * arm's side), so a tilted robot changes the gravity torque.
 */
struct Arm {
  double inertia = 0.1;        // kg*m^2 about the pivot
  double gearing = 50;
  double massKg = 2;
  double comLengthM = 0.3;     // pivot to centre of mass
  double minAngle = -kPi / 2, maxAngle = kPi / 2;
  double hardStopStiffness = 0;  // N*m/rad; <= 0 is a rigid stop
  double startAngle = 0;
  double viscous = 0, coulombFriction = 0;

  void Step(double dt, double rotorTorque, double rotorDamping = 0) {
    Init();
    const double gravity = -massKg * kGravity * comLengthM * std::cos(angle_ + pitch_);
    detail::StepDof(dt, inertia, gearing * rotorTorque + gravity + external_, rotorDamping * gearing * gearing,
                    viscous, coulombFriction, minAngle, maxAngle, hardStopStiffness, angle_, speed_);
  }
  void SetPitchOffset(double radians) { pitch_ = radians; }
  /** Extra torque at the pivot (N*m) applied every step until changed, e.g. a held piece. */
  void SetExternalTorque(double torque) { external_ = torque; }
  double Angle() { Init(); return angle_; }
  double Position() { return Angle(); }
  double Velocity() const { return speed_; }
  double RotorPosition() { return Angle() * gearing / (2 * kPi); }
  double RotorVelocity() const { return speed_ * gearing / (2 * kPi); }
  bool AtLowerStop() { return Angle() <= minAngle + 1e-9; }
  bool AtUpperStop() { return Angle() >= maxAngle - 1e-9; }
  void SetState(double angleRad, double speed) { angle_ = angleRad; speed_ = speed; init_ = true; }

  // Internal state: public only so the struct stays an aggregate ({inertia, gearing, ...}).
  void Init() { if (!init_) { angle_ = std::clamp(startAngle, minAngle, maxAngle); init_ = true; } }
  double angle_ = 0, speed_ = 0, pitch_ = 0, external_ = 0;
  bool init_ = false;
};

/**
 * Carriage on a drum or sprocket. Position in m along the travel. angleFromHorizontal (rad):
 * pi/2 is a vertical elevator, 0 a horizontal linear slide (no gravity). SetPitchOffset adds the
 * robot's tilt in the slide's plane.
 */
struct Elevator {
  double massKg = 5;
  double drumRadius = 0.02;  // m (pitch radius of the sprocket or pulley)
  double gearing = 10;       // rotor rotations per drum rotation
  double min = 0, max = 1;   // m
  double angleFromHorizontal = kPi / 2;
  double hardStopStiffness = 0;  // N/m; <= 0 is a rigid stop
  double start = 0;
  double viscous = 0, coulombFriction = 0;  // N per m/s, N

  void Step(double dt, double rotorTorque, double rotorDamping = 0) {
    Init();
    const double k = gearing / drumRadius;  // N per rotor N*m
    const double gravity = -massKg * kGravity * std::sin(angleFromHorizontal + pitch_);
    detail::StepDof(dt, massKg, k * rotorTorque + gravity + external_, rotorDamping * k * k, viscous,
                    coulombFriction, min, max, hardStopStiffness, x_, v_);
  }
  void SetPitchOffset(double radians) { pitch_ = radians; }
  void SetExternalForce(double newtons) { external_ = newtons; }
  double Position() { Init(); return x_; }
  double Velocity() const { return v_; }
  double RotorPosition() { return Position() / (2 * kPi * drumRadius) * gearing; }
  double RotorVelocity() const { return v_ / (2 * kPi * drumRadius) * gearing; }
  bool AtLowerStop() { return Position() <= min + 1e-12; }
  bool AtUpperStop() { return Position() >= max - 1e-12; }
  void SetState(double positionM, double velocity) { x_ = positionM; v_ = velocity; init_ = true; }

  // Internal state: public only so the struct stays an aggregate ({inertia, gearing, ...}).
  void Init() { if (!init_) { x_ = std::clamp(start, min, max); init_ = true; } }
  double x_ = 0, v_ = 0, pitch_ = 0, external_ = 0;
  bool init_ = false;
};

/** Intake/indexer roller. loadTorque (N*m at the roller) resists motion like Coulomb friction. */
struct Roller {
  double inertia = 0.0005;
  double gearing = 3;
  double loadTorque = 0;

  void Step(double dt, double rotorTorque, double rotorDamping = 0) {
    detail::StepDof(dt, inertia, gearing * rotorTorque, rotorDamping * gearing * gearing, 0, loadTorque,
                    -kInf, kInf, 0, angle_, speed_);
  }
  void SetLoadTorque(double torque) { loadTorque = torque; }
  double Position() const { return angle_; }
  double Velocity() const { return speed_; }
  double RotorPosition() const { return angle_ * gearing / (2 * kPi); }
  double RotorVelocity() const { return speed_ * gearing / (2 * kPi); }
  /** Surface speed (m/s) of a roller of the given radius. */
  double SurfaceSpeed(double radius) const { return speed_ * radius; }
  void SetState(double angleRad, double speed) { angle_ = angleRad; speed_ = speed; }

  // Internal state: public only so the struct stays an aggregate ({inertia, gearing, ...}).
  static constexpr double kInf = std::numeric_limits<double>::infinity();
  double angle_ = 0, speed_ = 0;
};

}  // namespace mayhem::sim
