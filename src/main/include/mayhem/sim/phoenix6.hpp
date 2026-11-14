#pragma once
// mayhem::sim Phoenix 6 bridge (C++20, Phoenix 6 v26 + WPILib 2026). Only compiled when you
// include it from a project that has Phoenix 6.
//
// In simulation the robot keeps calling SetControl on its real ctre::phoenix6::hardware::TalonFX
// objects; each loop the sim reads what each motor was told and drives the emulator from it:
//
//   if (auto request = mayhem::sim::FromApplied(*talon.GetAppliedControl())) emulator.SetControl(*request);
//   emulator.Configure(mayhem::sim::FromPhoenix(talonConfig));
//
// Follower / StrictFollower map to Follower{leaderId, opposed} / StrictFollower{leaderId}; resolve
// the leader with TalonFx::SetLeader. Unsupported requests (DutyCycle closed loops, Differential*,
// Dynamic*, music) return std::nullopt.
//
// Swerve: Phoenix's SwerveModuleImpl reads its steer angle from status signals, which lag under
// stepped time, so the module math cannot run in simulation. Convert the drivetrain's current
// SwerveRequest with FromRequest(...) instead and apply it with swerve::SwerveRequestEmulator,
// which reproduces Phoenix's request and module math against the engine's exact module angles.
#include "mayhem/sim/swerve_requests.hpp"
#include "mayhem/sim/talonfx.hpp"

#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/controls/ControlRequest.hpp>
#include <ctre/phoenix6/swerve/SwerveRequest.hpp>

#include <optional>
#include <type_traits>

namespace mayhem::sim {

namespace phoenix_detail {
template <class T>
double Val(const T& v) {
  if constexpr (std::is_arithmetic_v<T>) return static_cast<double>(v);
  else return static_cast<double>(v.value());
}
inline SlotGains Slot(double kP, double kI, double kD, double kS, double kV, double kA, double kG, bool armCosine,
                      bool closedLoopSign, double offset) {
  SlotGains g;
  g.kP = kP; g.kI = kI; g.kD = kD; g.kS = kS; g.kV = kV; g.kA = kA; g.kG = kG;
  g.gravity = armCosine ? SlotGains::Gravity::ArmCosine : SlotGains::Gravity::ElevatorStatic;
  g.staticSign = closedLoopSign ? SlotGains::StaticSign::ClosedLoopSign : SlotGains::StaticSign::VelocitySign;
  g.gravityArmPositionOffset = offset;
  return g;
}
template <class S>
SlotGains FromSlot(const S& s) {
  using namespace ctre::phoenix6::signals;
  return Slot(Val(s.kP), Val(s.kI), Val(s.kD), Val(s.kS), Val(s.kV), Val(s.kA), Val(s.kG),
              s.GravityType.value == GravityTypeValue::Arm_Cosine,
              s.StaticFeedforwardSign.value == StaticFeedforwardSignValue::UseClosedLoopSign,
              Val(s.GravityArmPositionOffset));
}
}  // namespace phoenix_detail

/** Phoenix TalonFXConfiguration -> emulator config (the fields the emulator models). */
inline TalonFxConfig FromPhoenix(const ctre::phoenix6::configs::TalonFXConfiguration& c) {
  using namespace ctre::phoenix6::signals;
  using phoenix_detail::Val;
  TalonFxConfig t;
  t.slots = {phoenix_detail::FromSlot(c.Slot0), phoenix_detail::FromSlot(c.Slot1), phoenix_detail::FromSlot(c.Slot2)};
  t.inverted = c.MotorOutput.Inverted.value == InvertedValue::Clockwise_Positive;
  t.neutral = c.MotorOutput.NeutralMode.value == NeutralModeValue::Brake ? TalonFxConfig::Neutral::Brake
                                                                         : TalonFxConfig::Neutral::Coast;
  t.peakForwardDutyCycle = Val(c.MotorOutput.PeakForwardDutyCycle);
  t.peakReverseDutyCycle = Val(c.MotorOutput.PeakReverseDutyCycle);
  t.sensorToMechanismRatio = Val(c.Feedback.SensorToMechanismRatio);
  t.rotorToSensorRatio = Val(c.Feedback.RotorToSensorRatio);
  t.continuousWrap = c.ClosedLoopGeneral.ContinuousWrap;
  t.statorLimitEnable = c.CurrentLimits.StatorCurrentLimitEnable;
  t.statorLimit = Val(c.CurrentLimits.StatorCurrentLimit);
  t.supplyLimitEnable = c.CurrentLimits.SupplyCurrentLimitEnable;
  t.supplyLimit = Val(c.CurrentLimits.SupplyCurrentLimit);
  t.supplyLowerLimit = Val(c.CurrentLimits.SupplyCurrentLowerLimit);
  t.supplyLowerTime = Val(c.CurrentLimits.SupplyCurrentLowerTime);
  t.peakForwardVoltage = Val(c.Voltage.PeakForwardVoltage);
  t.peakReverseVoltage = Val(c.Voltage.PeakReverseVoltage);
  t.peakForwardTorqueCurrent = Val(c.TorqueCurrent.PeakForwardTorqueCurrent);
  t.peakReverseTorqueCurrent = Val(c.TorqueCurrent.PeakReverseTorqueCurrent);
  t.mmCruiseVelocity = Val(c.MotionMagic.MotionMagicCruiseVelocity);
  t.mmAcceleration = Val(c.MotionMagic.MotionMagicAcceleration);
  t.mmJerk = Val(c.MotionMagic.MotionMagicJerk);
  t.mmExpoKv = Val(c.MotionMagic.MotionMagicExpo_kV);
  t.mmExpoKa = Val(c.MotionMagic.MotionMagicExpo_kA);
  t.forwardSoftLimitEnable = c.SoftwareLimitSwitch.ForwardSoftLimitEnable;
  t.reverseSoftLimitEnable = c.SoftwareLimitSwitch.ReverseSoftLimitEnable;
  t.forwardSoftLimitThreshold = Val(c.SoftwareLimitSwitch.ForwardSoftLimitThreshold);
  t.reverseSoftLimitThreshold = Val(c.SoftwareLimitSwitch.ReverseSoftLimitThreshold);
  return t;
}

/** Slot0Configs (e.g. TunerConstants' steerGains/driveGains) -> SlotGains. */
inline SlotGains FromPhoenix(const ctre::phoenix6::configs::Slot0Configs& s) { return phoenix_detail::FromSlot(s); }

/** The request a TalonFX was last given (TalonFX::GetAppliedControl()) -> emulator request. */
inline std::optional<ControlRequest> FromApplied(const ctre::phoenix6::controls::ControlRequest& request) {
  namespace c = ctre::phoenix6::controls;
  using phoenix_detail::Val;
  const auto* r = &request;
  if (dynamic_cast<const c::NeutralOut*>(r)) return NeutralOut{};
  if (dynamic_cast<const c::CoastOut*>(r)) return CoastOut{};
  if (dynamic_cast<const c::StaticBrake*>(r)) return StaticBrake{};
  if (auto* x = dynamic_cast<const c::DutyCycleOut*>(r)) return DutyCycleOut{Val(x->Output), x->EnableFOC};
  if (auto* x = dynamic_cast<const c::VoltageOut*>(r)) return VoltageOut{Val(x->Output), x->EnableFOC};
  if (auto* x = dynamic_cast<const c::TorqueCurrentFOC*>(r))
    return TorqueCurrentFOC{Val(x->Output), Val(x->MaxAbsDutyCycle), Val(x->Deadband)};
  if (auto* x = dynamic_cast<const c::VelocityVoltage*>(r))
    return VelocityVoltage{Val(x->Velocity), Val(x->Acceleration), Val(x->FeedForward), x->Slot, x->EnableFOC};
  if (auto* x = dynamic_cast<const c::PositionVoltage*>(r))
    return PositionVoltage{Val(x->Position), Val(x->Velocity), Val(x->FeedForward), x->Slot, x->EnableFOC};
  if (auto* x = dynamic_cast<const c::MotionMagicVoltage*>(r))
    return MotionMagicVoltage{Val(x->Position), Val(x->FeedForward), x->Slot, x->EnableFOC};
  if (auto* x = dynamic_cast<const c::MotionMagicExpoVoltage*>(r))
    return MotionMagicExpoVoltage{Val(x->Position), Val(x->FeedForward), x->Slot, x->EnableFOC};
  if (auto* x = dynamic_cast<const c::MotionMagicVelocityVoltage*>(r))
    return MotionMagicVelocityVoltage{Val(x->Velocity), Val(x->Acceleration), Val(x->FeedForward), x->Slot, x->EnableFOC};
  if (auto* x = dynamic_cast<const c::VelocityTorqueCurrentFOC*>(r))
    return VelocityTorqueCurrentFOC{Val(x->Velocity), Val(x->Acceleration), Val(x->FeedForward), x->Slot};
  if (auto* x = dynamic_cast<const c::PositionTorqueCurrentFOC*>(r))
    return PositionTorqueCurrentFOC{Val(x->Position), Val(x->Velocity), Val(x->FeedForward), x->Slot};
  if (auto* x = dynamic_cast<const c::MotionMagicTorqueCurrentFOC*>(r))
    return MotionMagicTorqueCurrentFOC{Val(x->Position), Val(x->FeedForward), x->Slot};
  if (auto* x = dynamic_cast<const c::MotionMagicExpoTorqueCurrentFOC*>(r))
    return MotionMagicExpoTorqueCurrentFOC{Val(x->Position), Val(x->FeedForward), x->Slot};
  if (auto* x = dynamic_cast<const c::MotionMagicVelocityTorqueCurrentFOC*>(r))
    return MotionMagicVelocityTorqueCurrentFOC{Val(x->Velocity), Val(x->Acceleration), Val(x->FeedForward), x->Slot};
  if (auto* x = dynamic_cast<const c::Follower*>(r))
    return Follower{x->LeaderID, x->MotorAlignment.value == ctre::phoenix6::signals::MotorAlignmentValue::Opposed};
  if (auto* x = dynamic_cast<const c::StrictFollower*>(r)) return StrictFollower{x->LeaderID};
  return std::nullopt;
}

// --- Swerve requests -------------------------------------------------------------------------
namespace phoenix_detail {
namespace pr = ctre::phoenix6::swerve::requests;
namespace pi = ctre::phoenix6::swerve::impl;
inline swerve::DriveRequestType Drive(pi::DriveRequestType t) {
  return t == pi::DriveRequestType::Velocity ? swerve::DriveRequestType::Velocity : swerve::DriveRequestType::OpenLoopVoltage;
}
inline swerve::SteerRequestType Steer(pi::SteerRequestType t) {
  return t == pi::SteerRequestType::MotionMagicExpo ? swerve::SteerRequestType::MotionMagicExpo : swerve::SteerRequestType::Position;
}
inline swerve::ForwardPerspective Perspective(pr::ForwardPerspectiveValue v) {
  return v == pr::ForwardPerspectiveValue::OperatorPerspective ? swerve::ForwardPerspective::OperatorPerspective
                                                               : swerve::ForwardPerspective::BlueAlliance;
}
inline Vec2 Center(const frc::Translation2d& t) { return {t.X().value(), t.Y().value()}; }
template <class P>
swerve::HeadingGains Gains(const P& pid) {
  return {pid.GetP(), pid.GetI(), pid.GetD(), pid.GetIZone()};
}
template <class V>
std::vector<double> Forces(const V& v) {
  std::vector<double> out;
  for (const auto& f : v) out.push_back(f.value());
  return out;
}
}  // namespace phoenix_detail

inline swerve::SwerveRequest FromRequest(const ctre::phoenix6::swerve::requests::Idle&) { return swerve::Idle{}; }
inline swerve::SwerveRequest FromRequest(const ctre::phoenix6::swerve::requests::SwerveDriveBrake& r) {
  using namespace phoenix_detail;
  return swerve::SwerveDriveBrake{Drive(r.DriveRequestType), Steer(r.SteerRequestType)};
}
inline swerve::SwerveRequest FromRequest(const ctre::phoenix6::swerve::requests::PointWheelsAt& r) {
  using namespace phoenix_detail;
  return swerve::PointWheelsAt{r.ModuleDirection.Radians().value(), Drive(r.DriveRequestType), Steer(r.SteerRequestType)};
}
inline swerve::SwerveRequest FromRequest(const ctre::phoenix6::swerve::requests::FieldCentric& r) {
  using namespace phoenix_detail;
  swerve::FieldCentric o;
  o.velocityX = r.VelocityX.value(); o.velocityY = r.VelocityY.value(); o.rotationalRate = r.RotationalRate.value();
  o.deadband = r.Deadband.value(); o.rotationalDeadband = r.RotationalDeadband.value();
  o.centerOfRotation = Center(r.CenterOfRotation);
  o.driveRequestType = Drive(r.DriveRequestType); o.steerRequestType = Steer(r.SteerRequestType);
  o.desaturateWheelSpeeds = r.DesaturateWheelSpeeds; o.forwardPerspective = Perspective(r.ForwardPerspective);
  return o;
}
inline swerve::SwerveRequest FromRequest(const ctre::phoenix6::swerve::requests::RobotCentric& r) {
  using namespace phoenix_detail;
  swerve::RobotCentric o;
  o.velocityX = r.VelocityX.value(); o.velocityY = r.VelocityY.value(); o.rotationalRate = r.RotationalRate.value();
  o.deadband = r.Deadband.value(); o.rotationalDeadband = r.RotationalDeadband.value();
  o.centerOfRotation = Center(r.CenterOfRotation);
  o.driveRequestType = Drive(r.DriveRequestType); o.steerRequestType = Steer(r.SteerRequestType);
  o.desaturateWheelSpeeds = r.DesaturateWheelSpeeds;
  return o;
}
inline swerve::SwerveRequest FromRequest(const ctre::phoenix6::swerve::requests::ApplyRobotSpeeds& r) {
  using namespace phoenix_detail;
  swerve::ApplyRobotSpeeds o;
  o.speeds = {r.Speeds.vx.value(), r.Speeds.vy.value(), r.Speeds.omega.value()};
  o.wheelForceFeedforwardsX = Forces(r.WheelForceFeedforwardsX);
  o.wheelForceFeedforwardsY = Forces(r.WheelForceFeedforwardsY);
  o.centerOfRotation = Center(r.CenterOfRotation);
  o.driveRequestType = Drive(r.DriveRequestType); o.steerRequestType = Steer(r.SteerRequestType);
  o.desaturateWheelSpeeds = r.DesaturateWheelSpeeds;
  return o;
}
inline swerve::SwerveRequest FromRequest(const ctre::phoenix6::swerve::requests::ApplyFieldSpeeds& r) {
  using namespace phoenix_detail;
  swerve::ApplyFieldSpeeds o;
  o.speeds = {r.Speeds.vx.value(), r.Speeds.vy.value(), r.Speeds.omega.value()};
  o.wheelForceFeedforwardsX = Forces(r.WheelForceFeedforwardsX);
  o.wheelForceFeedforwardsY = Forces(r.WheelForceFeedforwardsY);
  o.centerOfRotation = Center(r.CenterOfRotation);
  o.driveRequestType = Drive(r.DriveRequestType); o.steerRequestType = Steer(r.SteerRequestType);
  o.desaturateWheelSpeeds = r.DesaturateWheelSpeeds; o.forwardPerspective = Perspective(r.ForwardPerspective);
  return o;
}
inline swerve::SwerveRequest FromRequest(const ctre::phoenix6::swerve::requests::FieldCentricFacingAngle& r) {
  using namespace phoenix_detail;
  swerve::FieldCentricFacingAngle o;
  o.velocityX = r.VelocityX.value(); o.velocityY = r.VelocityY.value();
  o.targetDirection = r.TargetDirection.Radians().value(); o.targetRateFeedforward = r.TargetRateFeedforward.value();
  o.deadband = r.Deadband.value(); o.rotationalDeadband = r.RotationalDeadband.value();
  o.maxAbsRotationalRate = r.MaxAbsRotationalRate.value();
  o.centerOfRotation = Center(r.CenterOfRotation);
  o.driveRequestType = Drive(r.DriveRequestType); o.steerRequestType = Steer(r.SteerRequestType);
  o.desaturateWheelSpeeds = r.DesaturateWheelSpeeds; o.forwardPerspective = Perspective(r.ForwardPerspective);
  o.targetDirectionPerspective = r.TargetDirectionPerspective == pr::TargetDirectionPerspectiveValue::BlueAlliance
                                     ? swerve::TargetDirectionPerspective::BlueAlliance
                                     : swerve::TargetDirectionPerspective::UseForwardPerspective;
  o.headingController = Gains(r.HeadingController);
  return o;
}
inline swerve::SwerveRequest FromRequest(const ctre::phoenix6::swerve::requests::RobotCentricFacingAngle& r) {
  using namespace phoenix_detail;
  swerve::RobotCentricFacingAngle o;
  o.velocityX = r.VelocityX.value(); o.velocityY = r.VelocityY.value();
  o.targetDirection = r.TargetDirection.Radians().value(); o.targetRateFeedforward = r.TargetRateFeedforward.value();
  o.deadband = r.Deadband.value(); o.rotationalDeadband = r.RotationalDeadband.value();
  o.maxAbsRotationalRate = r.MaxAbsRotationalRate.value();
  o.centerOfRotation = Center(r.CenterOfRotation);
  o.driveRequestType = Drive(r.DriveRequestType); o.steerRequestType = Steer(r.SteerRequestType);
  o.desaturateWheelSpeeds = r.DesaturateWheelSpeeds; o.forwardPerspective = Perspective(r.ForwardPerspective);
  o.headingController = Gains(r.HeadingController);
  return o;
}
/** Any SwerveRequest the bridge knows (dynamic_cast); std::nullopt for SysId and custom requests. */
inline std::optional<swerve::SwerveRequest> FromRequest(const ctre::phoenix6::swerve::requests::SwerveRequest& r) {
  namespace pr = ctre::phoenix6::swerve::requests;
  if (auto* x = dynamic_cast<const pr::Idle*>(&r)) return FromRequest(*x);
  if (auto* x = dynamic_cast<const pr::SwerveDriveBrake*>(&r)) return FromRequest(*x);
  if (auto* x = dynamic_cast<const pr::PointWheelsAt*>(&r)) return FromRequest(*x);
  if (auto* x = dynamic_cast<const pr::FieldCentric*>(&r)) return FromRequest(*x);
  if (auto* x = dynamic_cast<const pr::RobotCentric*>(&r)) return FromRequest(*x);
  if (auto* x = dynamic_cast<const pr::ApplyRobotSpeeds*>(&r)) return FromRequest(*x);
  if (auto* x = dynamic_cast<const pr::ApplyFieldSpeeds*>(&r)) return FromRequest(*x);
  if (auto* x = dynamic_cast<const pr::FieldCentricFacingAngle*>(&r)) return FromRequest(*x);
  if (auto* x = dynamic_cast<const pr::RobotCentricFacingAngle*>(&r)) return FromRequest(*x);
  return std::nullopt;
}

}  // namespace mayhem::sim
