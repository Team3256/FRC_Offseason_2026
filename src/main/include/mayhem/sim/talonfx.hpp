#pragma once
// mayhem::sim TalonFX firmware emulator. Header-only, C++17, no dependencies.
//
// Emulates what a Phoenix 6 TalonFX does with a control request: unit conversion through
// SensorToMechanismRatio * RotorToSensorRatio, slot gains (kP kI kD kS kV kA kG), Motion Magic
// profiles (trapezoid, jerk-limited, Expo, velocity), peak-voltage clamps, the stator and supply
// current limits (with SupplyCurrentLowerLimit / LowerTime), neutral brake/coast and FOC vs
// trapezoidal motor curves. Call Step() at 1 kHz (the device's closed-loop rate).
//
// Frames. Step() takes the rotor position/velocity in the device's positive direction (what
// Phoenix reports as RotorPosition, i.e. after Inverted) and returns the shaft torque in that
// same direction. Wire your plant so that positive device output moves it positive: Phoenix
// users already do this with SimState.Orientation. `inverted` therefore only matters for
// followers, where it decides how the leader's output maps onto this motor (see Follower).
//
// Fidelity notes (Phoenix internals that are not documented and had to be chosen):
// - kD acts on (reference velocity - measured velocity) in position modes and on
//   (reference acceleration - measured acceleration) in velocity modes, so a step in the
//   position target does not kick the output.
// - The kI integrator resets when the request type or slot changes; the I term is clamped
//   to the peak output so it cannot wind up without bound.
// - Motion Magic re-plans from the current profile state every 1 ms (WPILib-equivalent
//   trapezoid and exponential profiles); a profile starts from the measured position and
//   velocity whenever a Motion Magic request is entered from another request type. A jerk
//   limit uses an online jerk-limited approximation (the exact CTRE S-curve is not public).
// - Motion Magic Expo assumes a 12 V maximum input: max velocity = 12 / kV, optionally
//   capped by MotionMagicCruiseVelocity (implemented by scaling that maximum input).
// - The supply limit lowers to SupplyCurrentLowerLimit after the demanded supply current
//   has stayed at or above SupplyCurrentLimit for SupplyCurrentLowerTime, and returns to the
//   regular limit once the demand drops below the lower limit. Regenerated current is not
//   supply-limited.
// - Velocity signals are noise- and latency-free (the device filters its velocity).
// - No voltage ramps, limit switches or deadbands other than TorqueCurrentFOC's.
#include "mayhem/sim/motor.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <type_traits>
#include <variant>

namespace mayhem::sim {

struct SlotGains {
  double kP = 0, kI = 0, kD = 0, kS = 0, kV = 0, kA = 0, kG = 0;
  enum class Gravity { ElevatorStatic, ArmCosine } gravity = Gravity::ElevatorStatic;
  enum class StaticSign { VelocitySign, ClosedLoopSign } staticSign = StaticSign::VelocitySign;
  double gravityArmPositionOffset = 0;  // mechanism rotations
};

// Defaults are Phoenix 6 (2026) factory defaults.
struct TalonFxConfig {
  std::array<SlotGains, 3> slots{};
  bool inverted = false;  // InvertedValue::Clockwise_Positive
  enum class Neutral { Coast, Brake } neutral = Neutral::Coast;
  double sensorToMechanismRatio = 1, rotorToSensorRatio = 1;
  bool continuousWrap = false;
  bool statorLimitEnable = true;
  double statorLimit = 120;
  bool supplyLimitEnable = true;
  double supplyLimit = 70, supplyLowerLimit = 40, supplyLowerTime = 1.0;
  double peakForwardVoltage = 16, peakReverseVoltage = -16;
  double mmCruiseVelocity = 0, mmAcceleration = 0, mmJerk = 0, mmExpoKv = 0.12, mmExpoKa = 0.1;
  // Beyond the required set:
  double peakForwardTorqueCurrent = 800, peakReverseTorqueCurrent = -800;
  double peakForwardDutyCycle = 1, peakReverseDutyCycle = -1;
  bool forwardSoftLimitEnable = false, reverseSoftLimitEnable = false;
  double forwardSoftLimitThreshold = 0, reverseSoftLimitThreshold = 0;  // mechanism rotations
};

// Control requests. Positions and velocities are mechanism units (rotations, rot/s).
// enableFoc picks the FOC motor curve (Phoenix's default is true).
struct NeutralOut {};
struct CoastOut {};
struct StaticBrake {};
struct DutyCycleOut { double output = 0; bool enableFoc = true; };
struct VoltageOut { double volts = 0; bool enableFoc = true; };
struct TorqueCurrentFOC { double amps = 0; double maxAbsDutyCycle = 1; double deadband = 0; };
struct VelocityVoltage { double velocity = 0, acceleration = 0, feedForward = 0; int slot = 0; bool enableFoc = true; };
struct PositionVoltage { double position = 0, velocity = 0, feedForward = 0; int slot = 0; bool enableFoc = true; };
struct MotionMagicVoltage { double position = 0, feedForward = 0; int slot = 0; bool enableFoc = true; };
struct MotionMagicExpoVoltage { double position = 0, feedForward = 0; int slot = 0; bool enableFoc = true; };
struct MotionMagicVelocityVoltage { double velocity = 0, acceleration = 0, feedForward = 0; int slot = 0; bool enableFoc = true; };
// Torque-current (FOC) closed loops: gains and feedForward are in amps.
struct VelocityTorqueCurrentFOC { double velocity = 0, acceleration = 0, feedForward = 0; int slot = 0; };
struct PositionTorqueCurrentFOC { double position = 0, velocity = 0, feedForward = 0; int slot = 0; };
struct MotionMagicTorqueCurrentFOC { double position = 0, feedForward = 0; int slot = 0; };
struct MotionMagicExpoTorqueCurrentFOC { double position = 0, feedForward = 0; int slot = 0; };
struct MotionMagicVelocityTorqueCurrentFOC { double velocity = 0, acceleration = 0, feedForward = 0; int slot = 0; };
// Followers: the caller resolves leaderId to a TalonFx and passes it to SetLeader().
// Follower: this motor's rotor turns the same raw direction as the leader's (opposed: the
// opposite), ignoring this motor's Inverted, like Phoenix's MotorAlignmentValue.
// StrictFollower: applies the leader's output in this motor's own Inverted frame.
struct Follower { int leaderId = 0; bool opposed = false; };
struct StrictFollower { int leaderId = 0; };

using ControlRequest = std::variant<NeutralOut, CoastOut, StaticBrake, DutyCycleOut, VoltageOut,
    TorqueCurrentFOC, VelocityVoltage, PositionVoltage, MotionMagicVoltage, MotionMagicExpoVoltage,
    MotionMagicVelocityVoltage, VelocityTorqueCurrentFOC, PositionTorqueCurrentFOC,
    MotionMagicTorqueCurrentFOC, MotionMagicExpoTorqueCurrentFOC, MotionMagicVelocityTorqueCurrentFOC,
    Follower, StrictFollower>;

// ---------------------------------------------------------------------------------------
// Motion profiles (also usable on their own).
namespace profile {

struct State { double position = 0, velocity = 0, acceleration = 0; };

/** WPILib TrapezoidProfile::Calculate(t, current, goal) (goal velocity may be nonzero). */
inline State Trapezoid(double t, State current, State goal, double maxVel, double maxAccel) {
  const double direction = current.position > goal.position ? -1.0 : 1.0;
  auto direct = [&](State s) { s.position *= direction; s.velocity *= direction; return s; };
  current = direct(current);
  goal = direct(goal);
  if (current.velocity > maxVel) current.velocity = maxVel;
  const double cutoffBegin = current.velocity / maxAccel;
  const double cutoffDistBegin = cutoffBegin * cutoffBegin * maxAccel / 2.0;
  const double cutoffEnd = goal.velocity / maxAccel;
  const double cutoffDistEnd = cutoffEnd * cutoffEnd * maxAccel / 2.0;
  const double fullTrapezoidDist = cutoffDistBegin + (goal.position - current.position) + cutoffDistEnd;
  double accelerationTime = maxVel / maxAccel;
  double fullSpeedDist = fullTrapezoidDist - accelerationTime * accelerationTime * maxAccel;
  if (fullSpeedDist < 0) {
    accelerationTime = std::sqrt(fullTrapezoidDist / maxAccel);
    fullSpeedDist = 0;
  }
  const double endAccel = accelerationTime - cutoffBegin;
  const double endFullSpeed = endAccel + fullSpeedDist / maxVel;
  const double endDecel = endFullSpeed + accelerationTime - cutoffEnd;
  State result{current.position, current.velocity, 0};
  if (t < endAccel) {
    result.velocity += t * maxAccel;
    result.position += (current.velocity + t * maxAccel / 2.0) * t;
    result.acceleration = maxAccel;
  } else if (t < endFullSpeed) {
    result.velocity = maxVel;
    result.position += (current.velocity + endAccel * maxAccel / 2.0) * endAccel + maxVel * (t - endAccel);
  } else if (t <= endDecel) {
    result.velocity = goal.velocity + (endDecel - t) * maxAccel;
    const double timeLeft = endDecel - t;
    result.position = goal.position - (goal.velocity + timeLeft * maxAccel / 2.0) * timeLeft;
    result.acceleration = -maxAccel;
  } else {
    result = goal;
    result.acceleration = 0;
  }
  result = direct(result);
  result.acceleration *= direction;
  return result;
}

/** WPILib ExponentialProfile (dv/dt = A v + B u, |u| <= maxInput); A = -kV/kA, B = 1/kA. */
class Exponential {
 public:
  Exponential(double maxInput, double kV, double kA) : u_(maxInput), A_(-kV / kA), B_(1.0 / kA) {}
  double MaxVelocity() const { return -u_ * B_ / A_; }
  State Calculate(double t, State current, State goal) const {
    const double direction = ShouldFlipInput(current, goal) ? -1.0 : 1.0;
    const double u = direction * u_;
    const State inflection = InflectionPoint(current, goal, u);
    double inflectionTime, totalTime;
    Timing(current, inflection, goal, u, inflectionTime, totalTime);
    State r;
    if (t < 0) return current;
    if (t < inflectionTime) {
      r = {DistanceFromTime(t, u, current), VelocityFromTime(t, u, current), 0};
      r.acceleration = A_ * r.velocity + B_ * u;
    } else if (t < totalTime) {
      r = {DistanceFromTime(t - totalTime, -u, goal), VelocityFromTime(t - totalTime, -u, goal), 0};
      r.acceleration = A_ * r.velocity - B_ * u;
    } else {
      r = goal;
      r.acceleration = 0;
    }
    return r;
  }

 private:
  double u_, A_, B_;
  static constexpr double kEpsilon = 1e-9;
  double DistanceFromTime(double t, double input, State initial) const {
    return initial.position + (-B_ * input * t + (initial.velocity + B_ * input / A_) * (std::exp(A_ * t) - 1)) / A_;
  }
  double VelocityFromTime(double t, double input, State initial) const {
    return (initial.velocity + B_ * input / A_) * std::exp(A_ * t) - B_ * input / A_;
  }
  double TimeFromVelocity(double velocity, double input, double initial) const {
    return std::log((A_ * velocity + B_ * input) / (A_ * initial + B_ * input)) / A_;
  }
  double DistanceFromVelocity(double velocity, double input, State initial) const {
    return initial.position + (velocity - initial.velocity) / A_ -
           B_ * input / (A_ * A_) * std::log((A_ * velocity + B_ * input) / (A_ * initial.velocity + B_ * input));
  }
  bool ShouldFlipInput(State current, State goal) const {
    const double u = u_, xf = goal.position, v0 = current.velocity, vf = goal.velocity;
    const double xForward = DistanceFromVelocity(vf, u, current);
    const double xReverse = DistanceFromVelocity(vf, -u, current);
    if (v0 >= MaxVelocity()) return xf < xReverse;
    if (v0 <= -MaxVelocity()) return xf < xForward;
    const bool a = v0 >= 0, b = vf >= 0, c = xf >= xForward, d = xf >= xReverse;
    return (a && !d) || (b && !c) || (!c && !d);
  }
  double InflectionVelocity(double input, State current, State goal) const {
    const double uDir = input > 0 ? 1.0 : (input < 0 ? -1.0 : 0.0);
    const double positionDelta = goal.position - current.position;
    const double velocityDelta = goal.velocity - current.velocity;
    const double scalar = (A_ * current.velocity + B_ * input) * (A_ * goal.velocity - B_ * input);
    const double power = -A_ / B_ / input * (A_ * positionDelta - velocityDelta);
    const double a = -A_ * A_;
    const double c = B_ * B_ * input * input + scalar * std::exp(power);
    if (-1e-9 < c && c < 0) return 0;
    return uDir * std::sqrt(-c / a);
  }
  State InflectionPoint(State current, State goal, double input) const {
    if (std::abs(current.position - goal.position) < kEpsilon && std::abs(current.velocity - goal.velocity) < kEpsilon)
      return current;
    const double v = InflectionVelocity(input, current, goal);
    return {DistanceFromVelocity(v, -input, goal), v, 0};
  }
  void Timing(State current, State inflection, State goal, double input, double& inflectionT, double& totalT) const {
    const double uDir = input > 0 ? 1.0 : -1.0;
    double forward;
    if (std::abs(uDir * MaxVelocity() - inflection.velocity) < kEpsilon) {
      double solvable = inflection.velocity, tToSolvable, xAtSolvable;
      if (std::abs(current.velocity - inflection.velocity) < kEpsilon) {
        tToSolvable = 0;
        xAtSolvable = current.position;
      } else {
        solvable += std::abs(current.velocity) > MaxVelocity() ? uDir * kEpsilon : -uDir * kEpsilon;
        tToSolvable = TimeFromVelocity(solvable, input, current.velocity);
        xAtSolvable = DistanceFromVelocity(solvable, input, current);
      }
      forward = tToSolvable + uDir * (inflection.position - xAtSolvable) / MaxVelocity();
    } else {
      forward = TimeFromVelocity(inflection.velocity, input, current.velocity);
    }
    const double backward = TimeFromVelocity(inflection.velocity, -input, goal.velocity);
    inflectionT = forward;
    totalT = forward - backward;
  }
};

/** Jerk-limited velocity tracking toward vTarget (acceleration <= maxAccel, |da/dt| <= jerk). */
inline State JerkVelocityStep(double dt, State s, double vTarget, double maxAccel, double jerk) {
  const double dv = vTarget - s.velocity;
  double aDesired = std::min(maxAccel, std::sqrt(2.0 * jerk * std::abs(dv)));
  aDesired = dv >= 0 ? aDesired : -aDesired;
  // Do not overshoot the target velocity while ramping acceleration down.
  const double da = std::clamp(aDesired - s.acceleration, -jerk * dt, jerk * dt);
  s.acceleration += da;
  double vNext = s.velocity + s.acceleration * dt;
  if ((vTarget - s.velocity) * (vTarget - vNext) < 0) {
    vNext = vTarget;
    s.acceleration = 0;
  }
  s.position += 0.5 * (s.velocity + vNext) * dt;
  s.velocity = vNext;
  return s;
}

/** Online jerk-limited (S-curve) position step toward goal (rest), approximating Motion Magic with jerk. */
inline State SCurveStep(double dt, State s, double goal, double maxVel, double maxAccel, double jerk) {
  const double d = goal - s.position;
  const double sign = d >= 0 ? 1.0 : -1.0;
  const double dist = std::abs(d);
  if (dist < 1e-9 && std::abs(s.velocity) < 1e-6) return {goal, 0, 0};
  // Highest speed that can still stop within dist (deceleration starts with a = 0).
  const double dKnee = maxAccel * maxAccel * maxAccel / (jerk * jerk);
  const double vStop = dist >= dKnee
      ? 0.5 * (-maxAccel * maxAccel / jerk + std::sqrt(std::pow(maxAccel, 4) / (jerk * jerk) + 8.0 * maxAccel * dist))
      : std::cbrt(dist * dist * jerk);
  const State next = JerkVelocityStep(dt, s, sign * std::min(maxVel, vStop), maxAccel, jerk);
  if ((goal - next.position) * sign < 0) return {goal, 0, 0};
  return next;
}

}  // namespace profile

// ---------------------------------------------------------------------------------------
class TalonFx {
 public:
  /** One curve used for both FOC and trapezoidal requests. */
  explicit TalonFx(DcMotor motor = DcMotor::KrakenX60(), TalonFxConfig config = {})
      : trap_(motor), foc_(motor), cfg_(config) {}
  /** Separate trapezoidal and FOC curves, chosen per request by enableFoc. */
  TalonFx(DcMotor trapezoidal, DcMotor foc, TalonFxConfig config = {})
      : trap_(trapezoidal), foc_(foc), cfg_(config) {}
  /** Kraken X60 with both published curves. */
  static TalonFx KrakenX60(TalonFxConfig config = {}) { return {DcMotor::KrakenX60(), DcMotor::KrakenX60Foc(), config}; }
  static TalonFx KrakenX44(TalonFxConfig config = {}) { return {DcMotor::KrakenX44(), DcMotor::KrakenX44Foc(), config}; }
  static TalonFx Falcon500(TalonFxConfig config = {}) { return {DcMotor::Falcon500(), DcMotor::Falcon500Foc(), config}; }

  void Configure(const TalonFxConfig& config) { cfg_ = config; }
  const TalonFxConfig& Config() const { return cfg_; }
  const DcMotor& Motor(bool foc) const { return foc ? foc_ : trap_; }
  void SetControl(const ControlRequest& request) { request_ = request; }
  const ControlRequest& Request() const { return request_; }
  /** The leader for Follower / StrictFollower requests (step the leader first each tick). */
  void SetLeader(const TalonFx* leader) { leader_ = leader; }
  /** Like Phoenix SetPosition: the mechanism position (and rotor position) now read `rotations`. */
  /** Takes effect at the rotor reading of the next Step (the device latches its sensor then). */
  void SetMechanismPosition(double rotations) {
    pendingPosition_ = rotations;
    havePending_ = true;
  }

  /**
   * Run one device update: closed loop, profile, current limits. rotorPositionRot and
   * rotorVelocityRps are the rotor's position (rot) and velocity (rot/s) in this motor's
   * positive direction (followers: the leader's mechanism direction, see header). Returns
   * shaft torque (N*m) in the same direction, including back-EMF and internal friction;
   * brake mode shorts the windings.
   */
  double Step(double dt, double busVoltage, double rotorPositionRot, double rotorVelocityRps) {
    sign_ = FollowSign();
    rotorPos_ = sign_ * rotorPositionRot;
    rotorVel_ = sign_ * rotorVelocityRps;
    if (havePending_) {
      offset_ = pendingPosition_ - rotorPos_ / Ratio();
      rotorOffset_ = pendingPosition_ * Ratio() - rotorPos_;
      if (profileActive_) profile_ = {pendingPosition_, Velocity(), 0};
      havePending_ = false;
    }
    bus_ = std::max(0.0, busVoltage);
    const double velocity = Velocity();
    measuredAccel_ = dt > 0 && haveLastVelocity_ ? (velocity - lastVelocity_) / dt : 0;
    lastVelocity_ = velocity;
    haveLastVelocity_ = true;

    const std::size_t index = request_.index();
    int slot = std::visit([](const auto& r) -> int {
      if constexpr (HasSlot<std::decay_t<decltype(r)>>::value) return r.slot; else return -1;
    }, request_);
    if (index != lastIndex_ || slot != lastSlot_) {
      integral_ = 0;
      if (!IsProfiled(request_) || index != lastIndex_) profileActive_ = false;
    }
    lastIndex_ = index;
    lastSlot_ = slot;

    Demand demand = Compute(dt);
    Apply(demand, dt);
    return sign_ * Torque(rotorVel_);
  }

  /**
   * Shaft torque at another rotor velocity with the voltage of the last Step held (and the
   * stator limit re-applied), for plants that sub-step between device updates.
   */
  double TorqueAt(double rotorVelocityRps) const { return sign_ * Torque(sign_ * rotorVelocityRps); }
  /** dTorque/dRotorSpeed magnitude (N*m per rad/s) of the last Step, for implicit plants. */
  double RotorDamping() const {
    if (mode_ == Mode::Coast || mode_ == Mode::Current || limited_) return 0;
    return motor().Damping();
  }

  double Position() const { return havePending_ ? pendingPosition_ : rotorPos_ / Ratio() + offset_; }
  double Velocity() const { return rotorVel_ / Ratio(); }
  double Acceleration() const { return measuredAccel_; }
  double RotorPosition() const { return havePending_ ? pendingPosition_ * Ratio() : rotorPos_ + rotorOffset_; }
  double RotorVelocity() const { return rotorVel_; }
  double MotorVoltage() const { return mode_ == Mode::Coast ? 0 : volts_; }
  double StatorCurrent() const { return stator_; }
  double SupplyCurrent() const { return supply_; }
  double TorqueCurrent() const { return current_; }
  double DutyCycle() const { return bus_ > 0 ? MotorVoltage() / bus_ : 0; }
  double ClosedLoopReference() const { return reference_; }
  double ClosedLoopReferenceSlope() const { return referenceSlope_; }
  double ClosedLoopError() const { return error_; }
  double ClosedLoopOutput() const { return closedLoopOutput_; }
  bool SupplyLimitLowered() const { return lowered_; }
  bool IsNeutral() const { return mode_ == Mode::Coast || mode_ == Mode::Brake; }
  bool FocActive() const { return foc_active_; }

 private:
  enum class Mode { Coast, Brake, Voltage, Current };
  struct Demand { Mode mode = Mode::Coast; double value = 0; bool foc = true; };
  template <class T, class = void> struct HasSlot : std::false_type {};
  template <class T> struct HasSlot<T, std::void_t<decltype(T::slot)>> : std::true_type {};

  DcMotor trap_, foc_;
  TalonFxConfig cfg_;
  ControlRequest request_ = NeutralOut{};
  const TalonFx* leader_ = nullptr;
  double sign_ = 1;
  double rotorPos_ = 0, rotorVel_ = 0, offset_ = 0, rotorOffset_ = 0, bus_ = 12, pendingPosition_ = 0;
  bool havePending_ = false;
  double lastVelocity_ = 0, measuredAccel_ = 0;
  bool haveLastVelocity_ = false;
  std::size_t lastIndex_ = std::numeric_limits<std::size_t>::max();
  int lastSlot_ = -2;
  double integral_ = 0;
  bool profileActive_ = false;
  profile::State profile_{};
  Mode mode_ = Mode::Coast;
  bool foc_active_ = true, limited_ = false, lowered_ = false;
  double volts_ = 0, current_ = 0, stator_ = 0, supply_ = 0, lowerTimer_ = 0;
  double reference_ = 0, referenceSlope_ = 0, error_ = 0, closedLoopOutput_ = 0;

  double Ratio() const { return cfg_.sensorToMechanismRatio * cfg_.rotorToSensorRatio; }
  const DcMotor& motor() const { return foc_active_ ? foc_ : trap_; }
  static bool IsProfiled(const ControlRequest& r) {
    return std::holds_alternative<MotionMagicVoltage>(r) || std::holds_alternative<MotionMagicExpoVoltage>(r) ||
           std::holds_alternative<MotionMagicVelocityVoltage>(r) || std::holds_alternative<MotionMagicTorqueCurrentFOC>(r) ||
           std::holds_alternative<MotionMagicExpoTorqueCurrentFOC>(r) ||
           std::holds_alternative<MotionMagicVelocityTorqueCurrentFOC>(r);
  }
  double FollowSign() const {
    if (!leader_) return 1;
    if (const auto* f = std::get_if<Follower>(&request_)) {
      const double leaderSign = leader_->cfg_.inverted ? -1 : 1, mine = cfg_.inverted ? -1 : 1;
      return leaderSign * (f->opposed ? -1 : 1) * mine;
    }
    return 1;
  }

  // Closed loop: returns output (V or A) for a position or velocity target.
  double ClosedLoop(double dt, const SlotGains& g, bool positionMode, double target, double refVel,
                    double refAccel, double feedForward, double peak) {
    const double position = Position(), velocity = Velocity();
    double error = positionMode ? target - position : target - velocity;
    if (positionMode && cfg_.continuousWrap) error -= std::floor(error + 0.5);
    integral_ += error * dt;
    double iTerm = g.kI * integral_;
    if (g.kI != 0 && std::abs(iTerm) > peak) {
      iTerm = std::copysign(peak, iTerm);
      integral_ = iTerm / g.kI;
    }
    const double dTerm = positionMode ? g.kD * (refVel - velocity) : g.kD * (refAccel - measuredAccel_);
    double staticSign;
    if (g.staticSign == SlotGains::StaticSign::VelocitySign)
      staticSign = refVel > 0 ? 1 : (refVel < 0 ? -1 : 0);
    else
      staticSign = error > 0 ? 1 : (error < 0 ? -1 : 0);
    const double gravity = g.gravity == SlotGains::Gravity::ArmCosine
        ? g.kG * std::cos(2 * kPi * (position + g.gravityArmPositionOffset)) : g.kG;
    reference_ = target;
    referenceSlope_ = positionMode ? refVel : refAccel;
    error_ = error;
    closedLoopOutput_ = g.kP * error + iTerm + dTerm + g.kS * staticSign + g.kV * refVel + g.kA * refAccel + gravity + feedForward;
    return closedLoopOutput_;
  }

  const SlotGains& Slot(int slot) const { return cfg_.slots[static_cast<std::size_t>(std::clamp(slot, 0, 2))]; }

  // Motion Magic position profile; returns the profile state after this tick.
  profile::State PositionProfile(double dt, double target, bool expo) {
    if (!profileActive_) {
      profile_ = {Position(), Velocity(), 0};
      profileActive_ = true;
    }
    if (cfg_.continuousWrap) {  // Plan the short way around.
      double delta = target - profile_.position;
      delta -= std::floor(delta + 0.5);
      target = profile_.position + delta;
    }
    if (expo) {
      if (cfg_.mmExpoKv <= 0 || cfg_.mmExpoKa <= 0) return profile_;
      double maxInput = 12.0;
      if (cfg_.mmCruiseVelocity > 0) maxInput = std::min(maxInput, cfg_.mmCruiseVelocity * cfg_.mmExpoKv);
      profile_ = profile::Exponential(maxInput, cfg_.mmExpoKv, cfg_.mmExpoKa).Calculate(dt, profile_, {target, 0, 0});
    } else if (cfg_.mmCruiseVelocity > 0 && cfg_.mmAcceleration > 0) {
      if (cfg_.mmJerk > 0)
        profile_ = profile::SCurveStep(dt, profile_, target, cfg_.mmCruiseVelocity, cfg_.mmAcceleration, cfg_.mmJerk);
      else
        profile_ = profile::Trapezoid(dt, profile_, {target, 0, 0}, cfg_.mmCruiseVelocity, cfg_.mmAcceleration);
    }
    return profile_;
  }
  profile::State VelocityProfile(double dt, double target, double requestAccel) {
    if (!profileActive_) {
      profile_ = {Position(), Velocity(), 0};
      profileActive_ = true;
    }
    const double accel = requestAccel > 0 ? requestAccel : cfg_.mmAcceleration;
    if (accel <= 0) {
      profile_ = {profile_.position, target, 0};
    } else if (cfg_.mmJerk > 0) {
      profile_ = profile::JerkVelocityStep(dt, profile_, target, accel, cfg_.mmJerk);
    } else {
      const double dv = std::clamp(target - profile_.velocity, -accel * dt, accel * dt);
      profile_.acceleration = dt > 0 ? dv / dt : 0;
      profile_.velocity += dv;
    }
    return profile_;
  }

  Demand Compute(double dt) {
    const double vPeak = std::max(cfg_.peakForwardVoltage, -cfg_.peakReverseVoltage);
    const double aPeak = std::max(cfg_.peakForwardTorqueCurrent, -cfg_.peakReverseTorqueCurrent);
    const Mode neutral = cfg_.neutral == TalonFxConfig::Neutral::Brake ? Mode::Brake : Mode::Coast;
    reference_ = referenceSlope_ = error_ = closedLoopOutput_ = 0;
    return std::visit([&](const auto& r) -> Demand {
      using T = std::decay_t<decltype(r)>;
      if constexpr (std::is_same_v<T, NeutralOut>) return {neutral, 0, foc_active_};
      else if constexpr (std::is_same_v<T, CoastOut>) return {Mode::Coast, 0, foc_active_};
      else if constexpr (std::is_same_v<T, StaticBrake>) return {Mode::Brake, 0, foc_active_};
      else if constexpr (std::is_same_v<T, DutyCycleOut>) {
        const double duty = std::clamp(r.output, cfg_.peakReverseDutyCycle, cfg_.peakForwardDutyCycle);
        return {Mode::Voltage, duty * bus_, r.enableFoc};
      } else if constexpr (std::is_same_v<T, VoltageOut>) return {Mode::Voltage, r.volts, r.enableFoc};
      else if constexpr (std::is_same_v<T, TorqueCurrentFOC>) {
        if (std::abs(r.amps) < r.deadband) return {neutral, 0, true};
        return {Mode::Current, r.amps, true};
      } else if constexpr (std::is_same_v<T, VelocityVoltage>)
        return {Mode::Voltage, ClosedLoop(dt, Slot(r.slot), false, r.velocity, r.velocity, r.acceleration, r.feedForward, vPeak), r.enableFoc};
      else if constexpr (std::is_same_v<T, PositionVoltage>)
        return {Mode::Voltage, ClosedLoop(dt, Slot(r.slot), true, r.position, r.velocity, 0, r.feedForward, vPeak), r.enableFoc};
      else if constexpr (std::is_same_v<T, MotionMagicVoltage> || std::is_same_v<T, MotionMagicExpoVoltage>) {
        const auto s = PositionProfile(dt, r.position, std::is_same_v<T, MotionMagicExpoVoltage>);
        return {Mode::Voltage, ClosedLoop(dt, Slot(r.slot), true, s.position, s.velocity, s.acceleration, r.feedForward, vPeak), r.enableFoc};
      } else if constexpr (std::is_same_v<T, MotionMagicVelocityVoltage>) {
        const auto s = VelocityProfile(dt, r.velocity, r.acceleration);
        return {Mode::Voltage, ClosedLoop(dt, Slot(r.slot), false, s.velocity, s.velocity, s.acceleration, r.feedForward, vPeak), r.enableFoc};
      } else if constexpr (std::is_same_v<T, VelocityTorqueCurrentFOC>)
        return {Mode::Current, ClosedLoop(dt, Slot(r.slot), false, r.velocity, r.velocity, r.acceleration, r.feedForward, aPeak), true};
      else if constexpr (std::is_same_v<T, PositionTorqueCurrentFOC>)
        return {Mode::Current, ClosedLoop(dt, Slot(r.slot), true, r.position, r.velocity, 0, r.feedForward, aPeak), true};
      else if constexpr (std::is_same_v<T, MotionMagicTorqueCurrentFOC> || std::is_same_v<T, MotionMagicExpoTorqueCurrentFOC>) {
        const auto s = PositionProfile(dt, r.position, std::is_same_v<T, MotionMagicExpoTorqueCurrentFOC>);
        return {Mode::Current, ClosedLoop(dt, Slot(r.slot), true, s.position, s.velocity, s.acceleration, r.feedForward, aPeak), true};
      } else if constexpr (std::is_same_v<T, MotionMagicVelocityTorqueCurrentFOC>) {
        const auto s = VelocityProfile(dt, r.velocity, r.acceleration);
        return {Mode::Current, ClosedLoop(dt, Slot(r.slot), false, s.velocity, s.velocity, s.acceleration, r.feedForward, aPeak), true};
      } else {  // Follower, StrictFollower
        if (!leader_ || leader_->IsNeutral()) return {neutral, 0, foc_active_};
        // Follower: this device's voltage = leader's * (leader invert * alignment * own invert).
        // StrictFollower: the leader's output in this device's own frame.
        const double v = leader_->MotorVoltage();
        if constexpr (std::is_same_v<T, Follower>) return {Mode::Voltage, v * sign_, leader_->FocActive()};
        else return {Mode::Voltage, v, leader_->FocActive()};
      }
    }, request_);
  }

  // Shorted-winding (brake) current for a back-EMF current emf/R, within the stator limit.
  double BrakeCurrent(double emfCurrent) const {
    const double i = -emfCurrent;
    return cfg_.statorLimitEnable ? std::clamp(i, -cfg_.statorLimit, cfg_.statorLimit) : i;
  }

  // Voltage after the stator limit at rotor speed w (rad/s), for a commanded voltage v.
  double StatorLimited(double v, double w, bool* limited) const {
    if (!cfg_.statorLimitEnable) return v;
    const DcMotor& m = motor();
    const double emf = w / m.Kv(), band = cfg_.statorLimit * m.R();
    const double lo = emf - band, hi = emf + band;
    if (v > hi || v < lo) {
      if (limited) *limited = true;
      return std::clamp(v, lo, hi);
    }
    return v;
  }

  void Apply(const Demand& d, double dt) {
    foc_active_ = d.foc;
    const DcMotor& m = motor();
    const double w = rotorVel_ * 2 * kPi;
    const double emf = w / m.Kv();
    limited_ = false;
    mode_ = d.mode;
    if (d.mode == Mode::Coast || d.mode == Mode::Brake) {
      current_ = d.mode == Mode::Brake ? BrakeCurrent(emf / m.R()) : 0;
      limited_ = d.mode == Mode::Brake && std::abs(current_ * m.R() + emf) > 1e-12;
      volts_ = d.mode == Mode::Brake ? emf + current_ * m.R() : 0;  // ~0 unless the limit acts
      lowerTimer_ = lowered_ ? lowerTimer_ : 0;
      Report();
      return;
    }
    double v;
    if (d.mode == Mode::Current) {
      double amps = std::clamp(d.value, cfg_.peakReverseTorqueCurrent, cfg_.peakForwardTorqueCurrent);
      if (cfg_.statorLimitEnable) amps = std::clamp(amps, -cfg_.statorLimit, cfg_.statorLimit);
      v = emf + amps * m.R();
    } else {
      v = std::clamp(d.value, cfg_.peakReverseVoltage, cfg_.peakForwardVoltage);
      v = StatorLimited(v, w, &limited_);
    }
    // Soft limits stop output that drives further past the threshold.
    const double pos = Position();
    if (cfg_.forwardSoftLimitEnable && pos >= cfg_.forwardSoftLimitThreshold && v > 0) v = 0;
    if (cfg_.reverseSoftLimitEnable && pos <= cfg_.reverseSoftLimitThreshold && v < 0) v = 0;
    v = std::clamp(v, -bus_, bus_);
    // Supply limit: I_supply = V (V - emf) / (R Vbus).
    if (cfg_.supplyLimitEnable && bus_ > 0) {
      const double demand = v * (v - emf) / (m.R() * bus_);
      const bool lowerable = cfg_.supplyLowerTime > 0 && cfg_.supplyLowerLimit < cfg_.supplyLimit;
      if (lowerable) {
        if (lowered_) {
          if (demand < cfg_.supplyLowerLimit) { lowered_ = false; lowerTimer_ = 0; }
        } else if (demand >= cfg_.supplyLimit) {
          lowerTimer_ += dt;
          if (lowerTimer_ > cfg_.supplyLowerTime) lowered_ = true;
        } else {
          lowerTimer_ = 0;
        }
      } else {
        lowered_ = false;
      }
      const double limit = lowered_ ? cfg_.supplyLowerLimit : cfg_.supplyLimit;
      if (demand > limit) {
        const double c = limit * m.R() * bus_;
        const double root = std::sqrt(emf * emf + 4 * c);
        v = v > 0 ? 0.5 * (emf + root) : 0.5 * (emf - root);
        limited_ = true;
      }
    }
    // The stator limit is enforced last and in both signs: |stator current| <= statorLimit even
    // when the request (or a sagging bus) sits far below the back-EMF (regenerative current).
    // This can need a terminal voltage above the bus when the back-EMF exceeds it; the emulator
    // applies it anyway (Phoenix limits |stator|), so treat bus-overspeed regen as approximate.
    v = StatorLimited(v, w, &limited_);
    if (d.mode == Mode::Current) limited_ = true;  // current loop: no back-EMF damping
    volts_ = v;
    current_ = (v - emf) / m.R();
    Report();
  }

  void Report() {
    const double i = current_;
    stator_ = (i * volts_ > 0 || (volts_ == 0 && i == 0)) ? std::abs(i) : -std::abs(i);
    supply_ = bus_ > 0 ? volts_ * i / bus_ : 0;
  }

  double Torque(double rotorVelRps) const {
    const DcMotor& m = motor();
    const double w = rotorVelRps * 2 * kPi;
    double i;
    if (mode_ == Mode::Coast) i = 0;
    else if (mode_ == Mode::Brake) i = BrakeCurrent(w / m.Kv() / m.R());
    else if (mode_ == Mode::Current) {
      // Current loop holds current until the bus runs out.
      const double v = std::clamp(volts_ + (w - rotorVel_ * 2 * kPi) / m.Kv(), -bus_, bus_);
      i = (v - w / m.Kv()) / m.R();
      if (cfg_.statorLimitEnable) i = std::clamp(i, -cfg_.statorLimit, cfg_.statorLimit);
    } else {
      const double v = StatorLimited(volts_, w, nullptr);
      i = (v - w / m.Kv()) / m.R();
    }
    // Internal friction (Kt * free current), smoothly signed over ~1% of free speed.
    const double friction = m.FrictionTorque() * std::tanh(w / (0.01 * m.freeSpeed));
    return m.Kt() * i - friction;
  }
};

}  // namespace mayhem::sim
