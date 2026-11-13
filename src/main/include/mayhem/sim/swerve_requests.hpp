#pragma once
// mayhem::sim emulation of Phoenix 6 SwerveRequest / SwerveModule semantics. Header-only, C++17.
//
// Robot code (or mayhem/sim/phoenix6.hpp) fills one of the request structs below with the same
// fields as the Phoenix request; SwerveRequestEmulator turns it into each module's drive and
// steer ControlRequest and applies them to a SwerveSim's TalonFx emulators, reproducing:
//
//  Request level (FieldCentric, RobotCentric, ApplyRobotSpeeds, ApplyFieldSpeeds,
//  FieldCentricFacingAngle, RobotCentricFacingAngle, SwerveDriveBrake, PointWheelsAt, Idle):
//   deadband on the translation magnitude and on the rotational rate; ForwardPerspective
//   (OperatorPerspective rotates by the operator forward direction, BlueAlliance does not);
//   field->robot rotation by the pose heading; ChassisSpeeds::Discretize over updatePeriod;
//   center of rotation; inverse kinematics that keep the last heading at zero speed;
//   desaturation to speedAt12Volts; FacingAngle's heading PID (PhoenixPIDController with
//   continuous input on [-pi, pi], MaxAbsRotationalRate clamp).
//  Module level (SwerveModuleImpl::Apply):
//   optimize against the module's current angle (flip > 90 degrees); cosine compensation
//   (speed *= max(0, cos(target - current))); coupling correction (drive rotor velocity +=
//   azimuth velocity * couplingRatio); wheel force feedforward (module-direction force ->
//   motor torque = F * wheelRadius / driveGearing -> torque current = torque / kT -> volts =
//   current * 12 / stallCurrent); drive output VelocityVoltage(rotor rot/s, slot 0) or
//   VelocityTorqueCurrentFOC for Velocity, VoltageOut(speed / speedAt12Volts * 12) for
//   OpenLoopVoltage; steer output MotionMagicExpo or Position (voltage or torque-current).
//
// Guessed (Phoenix's .cpp is not public): the wheel force is projected on the optimized target
// angle; open-loop drive adds the force feedforward volts; a non-zero ModuleRequest
// UpdatePeriod adds a steer velocity feedforward (change of target angle / updatePeriod, wrapped
// so a 180-degree optimization flip adds none) to Position steer requests only.
//
// ConfigureLikePhoenix() applies what Phoenix's SwerveModule constructor writes to the motor
// configs (drive: brake, slip-current stator and torque-current limits, ratio 1; steer:
// FusedCANcoder ratios, continuous wrap, MotionMagicExpo kV = 0.12 * steer ratio and kA =
// 1.2 / steer ratio).
#include "mayhem/sim/swerve.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <variant>
#include <vector>

namespace mayhem::sim::swerve {

enum class DriveRequestType { OpenLoopVoltage, Velocity };
enum class SteerRequestType { MotionMagicExpo, Position };
enum class ClosedLoopOutputType { Voltage, TorqueCurrentFOC };
enum class ForwardPerspective { OperatorPerspective, BlueAlliance };
enum class TargetDirectionPerspective { UseForwardPerspective, BlueAlliance };

struct ChassisSpeeds { double vx = 0, vy = 0, omega = 0; };
struct ModuleTarget { double speed = 0, angle = 0; };  // m/s, rad

struct DrivetrainConstants {
  std::vector<std::array<double, 2>> modulePositions;  // robot frame, m
  double speedAt12Volts = 4.5;  // m/s (kSpeedAt12Volts; also the desaturation limit)
  double driveGearing = 6.75, steerGearing = 150.0 / 7.0, couplingRatio = 0, wheelRadius = 0.0508;
  ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType::Voltage;
  ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType::Voltage;
  bool enableFoc = true;
  DcMotor driveMotor = DcMotor::KrakenX60Foc();  // kT and stall current for force feedforwards
  /** Build from a SwerveSim's params (speedAt12Volts must still be set). */
  static DrivetrainConstants From(const SwerveParams& p, double speedAt12Volts) {
    DrivetrainConstants c;
    c.modulePositions = p.modulePositions;
    c.speedAt12Volts = speedAt12Volts;
    c.driveGearing = p.driveGearing; c.steerGearing = p.steerGearing; c.couplingRatio = p.couplingRatio;
    c.wheelRadius = p.wheelRadius;
    c.driveMotor = p.driveMotorFoc.stallCurrent > 0 ? p.driveMotorFoc : p.driveMotor;
    return c;
  }
};

/** What SwerveDrivetrain passes to SwerveRequest::Apply. */
struct ControlParameters {
  double poseX = 0, poseY = 0, heading = 0;  // current pose (rad)
  double operatorForwardDirection = 0;       // rad (0 blue, pi red)
  double timestamp = 0;                      // s, for the heading PID
  double updatePeriod = 1.0 / 250;           // s (odometry period; 1/250 on CAN FD)
  ChassisSpeeds currentSpeeds{};             // robot-centric
};

// Request structs: same fields and defaults as Phoenix 6 (26.x). Angles in rad, speeds m/s.
struct Idle {};
struct SwerveDriveBrake {
  DriveRequestType driveRequestType = DriveRequestType::OpenLoopVoltage;
  SteerRequestType steerRequestType = SteerRequestType::Position;
};
struct FieldCentric {
  double velocityX = 0, velocityY = 0, rotationalRate = 0, deadband = 0, rotationalDeadband = 0;
  Vec2 centerOfRotation{0, 0};
  DriveRequestType driveRequestType = DriveRequestType::OpenLoopVoltage;
  SteerRequestType steerRequestType = SteerRequestType::Position;
  bool desaturateWheelSpeeds = true;
  ForwardPerspective forwardPerspective = ForwardPerspective::OperatorPerspective;
};
struct RobotCentric {
  double velocityX = 0, velocityY = 0, rotationalRate = 0, deadband = 0, rotationalDeadband = 0;
  Vec2 centerOfRotation{0, 0};
  DriveRequestType driveRequestType = DriveRequestType::OpenLoopVoltage;
  SteerRequestType steerRequestType = SteerRequestType::Position;
  bool desaturateWheelSpeeds = true;
};
struct PointWheelsAt {
  double moduleDirection = 0;
  DriveRequestType driveRequestType = DriveRequestType::OpenLoopVoltage;
  SteerRequestType steerRequestType = SteerRequestType::Position;
};
struct ApplyRobotSpeeds {
  ChassisSpeeds speeds{};
  std::vector<double> wheelForceFeedforwardsX, wheelForceFeedforwardsY;  // N, robot-centric
  Vec2 centerOfRotation{0, 0};
  DriveRequestType driveRequestType = DriveRequestType::OpenLoopVoltage;
  SteerRequestType steerRequestType = SteerRequestType::Position;
  bool desaturateWheelSpeeds = true;
};
struct ApplyFieldSpeeds {
  ChassisSpeeds speeds{};  // field-centric
  std::vector<double> wheelForceFeedforwardsX, wheelForceFeedforwardsY;  // N, field-centric
  Vec2 centerOfRotation{0, 0};
  DriveRequestType driveRequestType = DriveRequestType::OpenLoopVoltage;
  SteerRequestType steerRequestType = SteerRequestType::Position;
  bool desaturateWheelSpeeds = true;
  ForwardPerspective forwardPerspective = ForwardPerspective::BlueAlliance;
};
struct HeadingGains { double kP = 0, kI = 0, kD = 0, iZone = std::numeric_limits<double>::infinity(); };
struct FieldCentricFacingAngle {
  double velocityX = 0, velocityY = 0;
  double targetDirection = 0, targetRateFeedforward = 0;
  double deadband = 0, rotationalDeadband = 0, maxAbsRotationalRate = 0;
  Vec2 centerOfRotation{0, 0};
  DriveRequestType driveRequestType = DriveRequestType::OpenLoopVoltage;
  SteerRequestType steerRequestType = SteerRequestType::Position;
  bool desaturateWheelSpeeds = true;
  ForwardPerspective forwardPerspective = ForwardPerspective::OperatorPerspective;
  TargetDirectionPerspective targetDirectionPerspective = TargetDirectionPerspective::UseForwardPerspective;
  HeadingGains headingController{};
};
struct RobotCentricFacingAngle {
  double velocityX = 0, velocityY = 0;
  double targetDirection = 0, targetRateFeedforward = 0;  // field-centric heading
  double deadband = 0, rotationalDeadband = 0, maxAbsRotationalRate = 0;
  Vec2 centerOfRotation{0, 0};
  DriveRequestType driveRequestType = DriveRequestType::OpenLoopVoltage;
  SteerRequestType steerRequestType = SteerRequestType::Position;
  bool desaturateWheelSpeeds = true;
  ForwardPerspective forwardPerspective = ForwardPerspective::OperatorPerspective;
  HeadingGains headingController{};
};

using SwerveRequest = std::variant<Idle, SwerveDriveBrake, FieldCentric, RobotCentric, PointWheelsAt,
                                   ApplyRobotSpeeds, ApplyFieldSpeeds, FieldCentricFacingAngle,
                                   RobotCentricFacingAngle>;

// ---------------------------------------------------------------------------------------
inline double WrapAngle(double a) {
  a = std::fmod(a + kPi, 2 * kPi);
  if (a < 0) a += 2 * kPi;
  return a - kPi;
}

/** WPILib ChassisSpeeds::Discretize (Pose2d log of the constant-twist motion over dt). */
inline ChassisSpeeds Discretize(ChassisSpeeds s, double dt) {
  if (dt <= 0) return s;
  const double dx = s.vx * dt, dy = s.vy * dt, dtheta = s.omega * dt;
  const double halfDtheta = dtheta / 2, cosMinusOne = std::cos(dtheta) - 1;
  const double halfThetaByTan = std::abs(cosMinusOne) < 1e-9 ? 1.0 - dtheta * dtheta / 12.0
                                                              : -(halfDtheta * std::sin(dtheta)) / cosMinusOne;
  // translation.RotateBy(Rotation2d(halfThetaByTan, -halfDtheta)) * hypot(halfThetaByTan, halfDtheta)
  const double norm = std::hypot(halfThetaByTan, halfDtheta);
  const double c = halfThetaByTan / norm, sn = -halfDtheta / norm;
  const double tx = (dx * c - dy * sn) * norm, ty = (dx * sn + dy * c) * norm;
  return {tx / dt, ty / dt, dtheta / dt};
}

/** WPILib SwerveDriveKinematics::DesaturateWheelSpeeds. */
inline void Desaturate(std::vector<ModuleTarget>& states, double maxSpeed) {
  double real = 0;
  for (const auto& s : states) real = std::max(real, std::abs(s.speed));
  if (real > maxSpeed && real > 0)
    for (auto& s : states) s.speed = s.speed / real * maxSpeed;
}

/** WPILib SwerveModuleState::Optimize. */
inline ModuleTarget Optimize(ModuleTarget s, double currentAngle) {
  if (std::abs(WrapAngle(s.angle - currentAngle)) > kPi / 2) {
    s.speed = -s.speed;
    s.angle = WrapAngle(s.angle + kPi);
  }
  return s;
}

/** PhoenixPIDController (frc::PIDController semantics with caller-supplied timestamps). */
class PhoenixPid {
 public:
  void SetGains(const HeadingGains& g) { g_ = g; }
  void Reset() { total_ = 0; prevError_ = 0; havePrev_ = false; }
  double Calculate(double measurement, double setpoint, double timestamp) {
    const double dt = havePrev_ ? timestamp - lastTime_ : 0;
    lastTime_ = timestamp;
    const double error = WrapAngle(setpoint - measurement);
    if (std::abs(error) > g_.iZone) total_ = 0;
    else if (g_.kI != 0 && dt > 0) total_ = std::clamp(total_ + error * dt, -1.0 / g_.kI, 1.0 / g_.kI);
    const double derivative = havePrev_ && dt > 0 ? (error - prevError_) / dt : 0;
    prevError_ = error;
    havePrev_ = true;
    return g_.kP * error + g_.kI * total_ + g_.kD * derivative;
  }

 private:
  HeadingGains g_{};
  double total_ = 0, prevError_ = 0, lastTime_ = 0;
  bool havePrev_ = false;
};

/** One module's output: the requests to send plus the optimized target. */
struct ModuleCommand {
  ControlRequest drive = NeutralOut{};
  ControlRequest steer = NeutralOut{};
  ModuleTarget target{};  // after optimization (speed m/s, angle rad)
  bool apply = false;     // false for Idle (leave the previous requests)
};

/** Apply Phoenix's SwerveModule motor-config defaults to a SwerveSim's TalonFx emulators. */
inline void ConfigureLikePhoenix(SwerveSim& sim, const DrivetrainConstants& c, const SlotGains& steerGains,
                                 const SlotGains& driveGains, double slipCurrent = 120,
                                 TalonFxConfig driveBase = {}, TalonFxConfig steerBase = {}) {
  for (std::size_t i = 0; i < sim.ModuleCount(); ++i) {
    TalonFxConfig d = driveBase;
    d.neutral = TalonFxConfig::Neutral::Brake;
    d.slots[0] = driveGains;
    d.peakForwardTorqueCurrent = slipCurrent;
    d.peakReverseTorqueCurrent = -slipCurrent;
    d.statorLimit = slipCurrent;
    d.statorLimitEnable = true;
    d.rotorToSensorRatio = 1;
    d.sensorToMechanismRatio = 1;
    sim.DriveMotor(i).Configure(d);
    TalonFxConfig s = steerBase;
    s.slots[0] = steerGains;
    s.rotorToSensorRatio = c.steerGearing;
    s.sensorToMechanismRatio = 1;
    s.mmExpoKv = 0.12 * c.steerGearing;
    s.mmExpoKa = 1.2 / c.steerGearing;
    s.continuousWrap = true;
    sim.SteerMotor(i).Configure(s);
  }
}

class SwerveRequestEmulator {
 public:
  explicit SwerveRequestEmulator(DrivetrainConstants c) : c_(std::move(c)) {
    lastHeadings_.assign(c_.modulePositions.size(), 0.0);
    lastTargets_.assign(c_.modulePositions.size(), std::numeric_limits<double>::quiet_NaN());
  }
  const DrivetrainConstants& Constants() const { return c_; }

  /**
   * Compute every module's requests. currentAngles (rad) and steerVelocities (rad/s) are the
   * modules' azimuth angle and rate (the engine knows them exactly).
   */
  std::vector<ModuleCommand> Compute(const SwerveRequest& request, const ControlParameters& params,
                                     const std::vector<double>& currentAngles, const std::vector<double>& steerVelocities) {
    const std::size_t n = c_.modulePositions.size();
    std::vector<ModuleCommand> out(n);
    if (request.index() != lastRequestIndex_) pid_.Reset();
    lastRequestIndex_ = request.index();
    std::visit([&](const auto& r) {
      using T = std::decay_t<decltype(r)>;
      if constexpr (std::is_same_v<T, Idle>) {
        return;
      } else if constexpr (std::is_same_v<T, SwerveDriveBrake>) {
        for (std::size_t i = 0; i < n; ++i)
          out[i] = Module(i, {0, std::atan2(c_.modulePositions[i][1], c_.modulePositions[i][0])}, 0, 0,
                          r.driveRequestType, r.steerRequestType, params.updatePeriod, currentAngles[i], steerVelocities[i]);
      } else if constexpr (std::is_same_v<T, PointWheelsAt>) {
        for (std::size_t i = 0; i < n; ++i)
          out[i] = Module(i, {0, r.moduleDirection}, 0, 0, r.driveRequestType, r.steerRequestType, params.updatePeriod,
                          currentAngles[i], steerVelocities[i]);
      } else if constexpr (std::is_same_v<T, FieldCentric>) {
        out = DoFieldCentric(r, params, currentAngles, steerVelocities);
      } else if constexpr (std::is_same_v<T, RobotCentric>) {
        double vx = r.velocityX, vy = r.velocityY, w = r.rotationalRate;
        if (std::hypot(vx, vy) < r.deadband) vx = vy = 0;
        if (std::abs(w) < r.rotationalDeadband) w = 0;
        out = Speeds(Discretize({vx, vy, w}, params.updatePeriod), r.centerOfRotation, r.desaturateWheelSpeeds, {}, {},
                     r.driveRequestType, r.steerRequestType, params, currentAngles, steerVelocities);
      } else if constexpr (std::is_same_v<T, ApplyRobotSpeeds>) {
        out = Speeds(Discretize(r.speeds, params.updatePeriod), r.centerOfRotation, r.desaturateWheelSpeeds,
                     r.wheelForceFeedforwardsX, r.wheelForceFeedforwardsY, r.driveRequestType, r.steerRequestType,
                     params, currentAngles, steerVelocities);
      } else if constexpr (std::is_same_v<T, ApplyFieldSpeeds>) {
        // Speeds and (field-centric) force feedforwards: perspective, then field -> robot.
        const bool op = r.forwardPerspective == ForwardPerspective::OperatorPerspective;
        double vx = r.speeds.vx, vy = r.speeds.vy;
        if (op) Rotate(vx, vy, params.operatorForwardDirection);
        Rotate(vx, vy, -params.heading);
        std::vector<double> fx = r.wheelForceFeedforwardsX, fy = r.wheelForceFeedforwardsY;
        for (std::size_t i = 0; i < std::min(fx.size(), fy.size()); ++i) {
          if (op) Rotate(fx[i], fy[i], params.operatorForwardDirection);
          Rotate(fx[i], fy[i], -params.heading);
        }
        out = Speeds(Discretize({vx, vy, r.speeds.omega}, params.updatePeriod), r.centerOfRotation, r.desaturateWheelSpeeds,
                     fx, fy, r.driveRequestType, r.steerRequestType, params, currentAngles, steerVelocities);
      } else if constexpr (std::is_same_v<T, FieldCentricFacingAngle>) {
        double face = r.targetDirection;
        if (r.targetDirectionPerspective == TargetDirectionPerspective::UseForwardPerspective &&
            r.forwardPerspective == ForwardPerspective::OperatorPerspective)
          face += params.operatorForwardDirection;
        pid_.SetGains(r.headingController);
        double w = r.targetRateFeedforward + pid_.Calculate(params.heading, WrapAngle(face), params.timestamp);
        if (r.maxAbsRotationalRate > 0) w = std::clamp(w, -r.maxAbsRotationalRate, r.maxAbsRotationalRate);
        FieldCentric f;
        f.velocityX = r.velocityX; f.velocityY = r.velocityY; f.rotationalRate = w;
        f.deadband = r.deadband; f.rotationalDeadband = r.rotationalDeadband; f.centerOfRotation = r.centerOfRotation;
        f.driveRequestType = r.driveRequestType; f.steerRequestType = r.steerRequestType;
        f.desaturateWheelSpeeds = r.desaturateWheelSpeeds; f.forwardPerspective = r.forwardPerspective;
        out = DoFieldCentric(f, params, currentAngles, steerVelocities);
      } else if constexpr (std::is_same_v<T, RobotCentricFacingAngle>) {
        double face = r.targetDirection;
        if (r.forwardPerspective == ForwardPerspective::OperatorPerspective) face += params.operatorForwardDirection;
        pid_.SetGains(r.headingController);
        double w = r.targetRateFeedforward + pid_.Calculate(params.heading, WrapAngle(face), params.timestamp);
        if (r.maxAbsRotationalRate > 0) w = std::clamp(w, -r.maxAbsRotationalRate, r.maxAbsRotationalRate);
        double vx = r.velocityX, vy = r.velocityY;
        if (std::hypot(vx, vy) < r.deadband) vx = vy = 0;
        if (std::abs(w) < r.rotationalDeadband) w = 0;
        out = Speeds(Discretize({vx, vy, w}, params.updatePeriod), r.centerOfRotation, r.desaturateWheelSpeeds, {}, {},
                     r.driveRequestType, r.steerRequestType, params, currentAngles, steerVelocities);
      }
    }, request);
    targets_.resize(n);
    for (std::size_t i = 0; i < n; ++i) if (out[i].apply) targets_[i] = out[i].target;
    return out;
  }

  /** Compute from the sim's exact module angles and apply the requests to its TalonFxs. */
  std::vector<ModuleCommand> Apply(const SwerveRequest& request, SwerveSim& sim, ControlParameters params) {
    std::vector<double> angles, rates;
    for (std::size_t i = 0; i < sim.ModuleCount(); ++i) {
      angles.push_back(sim.ModuleSteerAngle(i));
      rates.push_back(sim.ModuleSteerRate(i));
    }
    auto out = Compute(request, params, angles, rates);
    for (std::size_t i = 0; i < out.size() && i < sim.ModuleCount(); ++i) {
      if (!out[i].apply) continue;
      sim.DriveMotor(i).SetControl(out[i].drive);
      sim.SteerMotor(i).SetControl(out[i].steer);
    }
    return out;
  }
  /** Convenience: control parameters from the sim's true pose and speeds. */
  static ControlParameters ParamsFrom(const SwerveSim& sim, double timestamp, double operatorForward = 0,
                                      double updatePeriod = 1.0 / 250) {
    ControlParameters p;
    const auto pose = sim.Pose();
    const auto v = sim.RobotVelocity();
    p.poseX = pose.x; p.poseY = pose.y; p.heading = pose.yaw;
    p.operatorForwardDirection = operatorForward;
    p.timestamp = timestamp;
    p.updatePeriod = updatePeriod;
    p.currentSpeeds = {v.vx, v.vy, v.omega};
    return p;
  }
  const std::vector<ModuleTarget>& TargetStates() const { return targets_; }

 private:
  DrivetrainConstants c_;
  std::vector<double> lastHeadings_, lastTargets_;
  std::vector<ModuleTarget> targets_;
  std::size_t lastRequestIndex_ = std::variant_npos;
  PhoenixPid pid_;

  static void Rotate(double& x, double& y, double angle) {
    const double c = std::cos(angle), s = std::sin(angle), nx = c * x - s * y;
    y = s * x + c * y;
    x = nx;
  }

  std::vector<ModuleCommand> DoFieldCentric(const FieldCentric& r, const ControlParameters& params,
                                            const std::vector<double>& angles, const std::vector<double>& rates) {
    double vx = r.velocityX, vy = r.velocityY, w = r.rotationalRate;
    if (r.forwardPerspective == ForwardPerspective::OperatorPerspective) Rotate(vx, vy, params.operatorForwardDirection);
    if (std::hypot(vx, vy) < r.deadband) vx = vy = 0;
    if (std::abs(w) < r.rotationalDeadband) w = 0;
    Rotate(vx, vy, -params.heading);  // ChassisSpeeds::FromFieldRelativeSpeeds
    return Speeds(Discretize({vx, vy, w}, params.updatePeriod), r.centerOfRotation, r.desaturateWheelSpeeds, {}, {},
                  r.driveRequestType, r.steerRequestType, params, angles, rates);
  }

  std::vector<ModuleCommand> Speeds(ChassisSpeeds s, Vec2 center, bool desaturate, const std::vector<double>& fx,
                                    const std::vector<double>& fy, DriveRequestType drive, SteerRequestType steer,
                                    const ControlParameters& params, const std::vector<double>& angles,
                                    const std::vector<double>& rates) {
    const std::size_t n = c_.modulePositions.size();
    std::vector<ModuleTarget> states(n);
    const bool still = s.vx == 0 && s.vy == 0 && s.omega == 0;
    for (std::size_t i = 0; i < n; ++i) {
      if (still) { states[i] = {0, lastHeadings_[i]}; continue; }
      const double x = c_.modulePositions[i][0] - center[0], y = c_.modulePositions[i][1] - center[1];
      const double mx = s.vx - s.omega * y, my = s.vy + s.omega * x;
      states[i] = {std::hypot(mx, my), std::atan2(my, mx)};
      lastHeadings_[i] = states[i].angle;
    }
    if (desaturate && c_.speedAt12Volts > 0) Desaturate(states, c_.speedAt12Volts);
    std::vector<ModuleCommand> out(n);
    for (std::size_t i = 0; i < n; ++i)
      out[i] = Module(i, states[i], i < fx.size() ? fx[i] : 0, i < fy.size() ? fy[i] : 0, drive, steer,
                      params.updatePeriod, angles[i], rates[i]);
    return out;
  }

  ModuleCommand Module(std::size_t i, ModuleTarget state, double forceX, double forceY, DriveRequestType drive,
                       SteerRequestType steer, double updatePeriod, double currentAngle, double steerRate) {
    ModuleCommand cmd;
    cmd.apply = true;
    const ModuleTarget opt = Optimize(state, currentAngle);
    cmd.target = opt;
    const double rotationsPerMeter = c_.driveGearing / (2 * kPi * c_.wheelRadius);
    double velocity = opt.speed * rotationsPerMeter;
    // ApplyVelocityCorrections: cosine compensation, then coupling.
    velocity *= std::max(0.0, std::cos(opt.angle - currentAngle));
    velocity += steerRate / (2 * kPi) * c_.couplingRatio;
    // Wheel force feedforward along the module direction.
    const double force = forceX * std::cos(opt.angle) + forceY * std::sin(opt.angle);
    const double motorTorque = force * c_.wheelRadius / c_.driveGearing;
    const double torqueCurrent = motorTorque / c_.driveMotor.Kt();
    const double ffVolts = torqueCurrent / c_.driveMotor.stallCurrent * 12.0;
    const bool torqueDrive = c_.driveClosedLoopOutput == ClosedLoopOutputType::TorqueCurrentFOC;
    if (drive == DriveRequestType::OpenLoopVoltage) {
      const double speed = velocity / rotationsPerMeter;
      cmd.drive = VoltageOut{speed / c_.speedAt12Volts * 12.0 + ffVolts, c_.enableFoc};
    } else if (torqueDrive) {
      cmd.drive = VelocityTorqueCurrentFOC{velocity, 0, torqueCurrent, 0};
    } else {
      cmd.drive = VelocityVoltage{velocity, 0, ffVolts, 0, c_.enableFoc};
    }
    const double targetRot = opt.angle / (2 * kPi);
    double steerVelocity = 0;
    if (updatePeriod > 0 && !std::isnan(lastTargets_[i])) {
      double delta = opt.angle - lastTargets_[i];
      delta -= kPi * std::floor(delta / kPi + 0.5);  // ignore 180-degree flips
      steerVelocity = delta / updatePeriod / (2 * kPi);
    }
    lastTargets_[i] = opt.angle;
    const bool torqueSteer = c_.steerClosedLoopOutput == ClosedLoopOutputType::TorqueCurrentFOC;
    if (steer == SteerRequestType::MotionMagicExpo) {
      if (torqueSteer) cmd.steer = MotionMagicExpoTorqueCurrentFOC{targetRot, 0, 0};
      else cmd.steer = MotionMagicExpoVoltage{targetRot, 0, 0, c_.enableFoc};
    } else {
      if (torqueSteer) cmd.steer = PositionTorqueCurrentFOC{targetRot, steerVelocity, 0, 0};
      else cmd.steer = PositionVoltage{targetRot, steerVelocity, 0, 0, c_.enableFoc};
    }
    return cmd;
  }
};

}  // namespace mayhem::sim::swerve
