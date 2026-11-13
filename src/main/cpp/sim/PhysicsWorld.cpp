// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "sim/PhysicsWorld.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <numbers>
#include <set>
#include <stdexcept>

#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include <frc/simulation/RoboRioSim.h>

#include "mayhem/sim/phoenix6.hpp"
#include "mayhem/sim/swerve_requests.hpp"
#include "mayhem/sim/wpilib_sim.hpp"
#include "sim/Rebuilt2026.h"
#include "sim/SimConstants.h"
#include "subsystems/linearslide/LinearSlideConstants.h"
#include "subsystems/shooter/ShooterConstants.h"
#include "subsystems/shooterpivot/ShooterPivotConstants.h"
#include "subsystems/swerve/generated/TunerConstants.h"

namespace sim {
namespace {
namespace sc = sim_constants;
using mayhem::sim::DcMotor;
using mayhem::sim::TalonFx;
using mayhem::sim::Vec3;
constexpr double kPi = std::numbers::pi;
constexpr double kDt = sc::kPhysicsDt;
constexpr int kTicksPerControl = 4;  // Phoenix's odometry/control thread: 250 Hz
/** Robot::Robot sets the roboRIO brownout threshold to 4.75 V. */
constexpr double kBrownoutVolts = 4.75;
/** A contact step with more impulse than this (N s per 1 ms, i.e. 300 N) counts as a hit. */
constexpr double kHitImpulse = 0.3;

std::unique_ptr<PhysicsWorld> g_world;
Perturbation g_nextPerturbation;
bool g_nextField = true;

/** Slide travel direction in the robot frame: forward and 12 degrees down. */
constexpr double kSlideDirX = 0.9782;
constexpr double kSlideDirZ = -0.2079;

mayhem::sim::SwerveParams MakeSwerveParams(const Perturbation& p) {
  mayhem::sim::SwerveParams s;
  const auto& m = TunerConstants::FrontLeft;
  s.massKg = sc::robot::kMassKg * p.massScale;
  s.moiKgM2 = sc::robot::kMoiKgM2 * p.massScale;
  s.rollMoiKgM2 = sc::robot::kRollMoiKgM2 * p.massScale;
  s.pitchMoiKgM2 = sc::robot::kPitchMoiKgM2 * p.massScale;
  s.cgHeightM = sc::robot::kCgHeightM;
  s.comX = sc::robot::kCgForwardM;
  for (const auto* module : {&TunerConstants::FrontLeft, &TunerConstants::FrontRight,
                             &TunerConstants::BackLeft, &TunerConstants::BackRight}) {
    s.modulePositions.push_back({module->LocationX.value(), module->LocationY.value()});
  }
  s.wheelRadius = m.WheelRadius.value();
  s.driveGearing = m.DriveMotorGearRatio;
  s.steerGearing = m.SteerMotorGearRatio;
  s.couplingRatio = m.CouplingGearRatio;
  s.driveMotor = DcMotor::KrakenX60();
  s.driveMotorFoc = DcMotor::KrakenX60Foc();
  s.steerMotor = DcMotor::KrakenX60();
  s.steerMotorFoc = DcMotor::KrakenX60Foc();
  s.wheelInertia = sc::swerve::kWheelInertia;
  s.steerInertia = sc::swerve::kSteerInertia;
  s.muStatic = sc::swerve::kMuStatic * p.muScale;
  s.muKinetic = sc::swerve::kMuKinetic * p.muScale;
  s.slipVelocity = sc::swerve::kSlipVelocity;
  s.rollingResistance = sc::swerve::kRollingResistance;
  s.driveEfficiency = sc::swerve::kDriveEfficiency;
  s.bumperLengthX = sc::robot::kBumperLengthM;
  s.bumperLengthY = sc::robot::kBumperWidthM;
  s.frameLengthX = 0.686;  // 0101_Drivebase_Tubes outer edges (CAD +/-0.343 m)
  s.frameLengthY = 0.686;
  s.frameClearanceM = sc::robot::kFrameClearanceM;
  s.robotHeightM = sc::robot::kStowedHeightM;
  return s;
}

mayhem::sim::BallParams MakeBallParams() {
  mayhem::sim::BallParams b;
  b.radius = sc::fuel::kRadiusM;
  b.mass = sc::fuel::kMassKg;
  b.restitution = sc::fuel::kRestitution;
  b.rollingFriction = sc::fuel::kRollingFriction;
  b.dragCd = sc::fuel::kDragCoefficient;
  b.magnus = sc::fuel::kMagnusCoefficient;
  return b;
}

/**
 * Height where a ball launched from z0 at speed v and elevation theta (backspin per the shooter
 * constants) crosses targetZ on the way down, as horizontal distance; NaN if it never gets there.
 * Same flight model as mayhem::sim::GamePieces (gravity, quadratic drag, Magnus, spin decay).
 */
double RangeAtHeight(double v, double theta, double z0, double targetZ) {
  const auto b = MakeBallParams();
  const double area = kPi * b.radius * b.radius;
  const double kDrag = 0.5 * b.airDensity * b.dragCd * area / b.mass;
  const double kMagnus = 0.5 * b.airDensity * area * b.radius * b.magnus / b.mass;
  // 2-D (x forward, z up); backspin about -y gives lift: (0,-w,0) x (vx,0,vz) = (-w vz, 0, w vx).
  double x = 0, z = z0, vx = v * std::cos(theta), vz = v * std::sin(theta);
  double w = sc::shooter::kBackspinPerSpeed * v;
  constexpr double dt = 0.002;
  const double decay = std::exp(-b.spinDamping * dt);
  for (int i = 0; i < 2000; i++) {
    const double speed = std::hypot(vx, vz);
    const double ax = -kDrag * speed * vx - kMagnus * w * vz;
    const double az = -9.80665 - kDrag * speed * vz + kMagnus * w * vx;
    const double nz = z + vz * dt + 0.5 * az * dt * dt;
    const double nx = x + vx * dt + 0.5 * ax * dt * dt;
    if (vz < 0 && z >= targetZ && nz < targetZ) {
      return x + (nx - x) * (z - targetZ) / (z - nz);
    }
    x = nx;
    z = nz;
    vx += ax * dt;
    vz += az * dt;
    w *= decay;
    if (z < 0) {
      break;
    }
  }
  return std::nan("");
}

/**
 * The launch angle that falls through targetZ {@code range} away on the high arc (above the
 * maximum-range angle) or the low arc, or NaN if out of reach.
 */
double SolveAngle(double v, double range, double z0, double targetZ, bool highArc) {
  double hi = 0.3, best = -1;
  for (double t = 0.3; t < 1.5; t += 0.01) {
    const double r = RangeAtHeight(v, t, z0, targetZ);
    if (std::isfinite(r) && r > best) {
      best = r;
      hi = t;
    }
  }
  if (best < range) {
    return std::nan("");
  }
  if (highArc) {
    // Range at the target height falls as the angle rises past the maximum-range angle.
    double lo = hi, top = 1.55;
    for (int i = 0; i < 40; i++) {
      const double mid = 0.5 * (lo + top);
      const double r = RangeAtHeight(v, mid, z0, targetZ);
      if (std::isfinite(r) && r > range) {
        lo = mid;
      } else {
        top = mid;
      }
    }
    return 0.5 * (lo + top);
  }
  // On the low arc the range at the target height grows with the angle; the lowest angles never
  // reach the target height at all (NaN).
  double lo = 0.2;
  for (int i = 0; i < 40; i++) {
    const double mid = 0.5 * (lo + hi);
    const double r = RangeAtHeight(v, mid, z0, targetZ);
    if (std::isfinite(r) && r >= range) {
      hi = mid;
    } else {
      lo = mid;
    }
  }
  return 0.5 * (lo + hi);
}
}  // namespace

// ------------------------------------------------------------------------------------------------

struct PhysicsWorld::MotorEntry {
  ctre::phoenix6::hardware::TalonFX* device;
  int id;
  Mechanism mechanism;
  TalonFx emulator;
  bool follower = false;
};

PhysicsWorld& PhysicsWorld::Get() {
  if (!g_world) {
    g_world.reset(new PhysicsWorld());
  }
  return *g_world;
}

bool PhysicsWorld::Exists() { return g_world != nullptr; }

void PhysicsWorld::Destroy() { g_world.reset(); }

void PhysicsWorld::SetNextPerturbation(const Perturbation& p) { g_nextPerturbation = p; }

void PhysicsWorld::SetNextFieldEnabled(bool enabled) { g_nextField = enabled; }

PhysicsWorld::PhysicsWorld()
    : m_perturbation{g_nextPerturbation},
      m_battery{sc::battery::kOpenCircuitVolts + g_nextPerturbation.batteryVoltageOffset,
                sc::battery::kInternalResistanceOhms * g_nextPerturbation.batteryResistanceScale,
                sc::battery::kWiringResistanceOhms} {
  // Mechanism plants. Gearing is rotor rotations per mechanism rotation (the configs'
  // SensorToMechanismRatio), so the emulators' positions are the robot code's mechanism units.
  m_flywheel = {sc::shooter::kFlywheelInertia, 1, sc::shooter::kFlywheelViscous, 0.02};
  m_hood = {sc::hood::kInertia,
            shooter_pivot::sim::kGearing,
            sc::hood::kMassKg,
            sc::hood::kComLengthM,
            sc::hood::kMinAngleRad,
            sc::hood::kMaxAngleRad,
            0,
            0,
            0.01,
            0.05};
  const double drum = sc::intake::kSlideTravelM / (sc::intake::kSlideFullRotations * 2 * kPi);
  m_slide = {sc::intake::kSlideMassKg,
             drum,
             linear_slide::sim::kGearing,
             0,
             sc::intake::kSlideTravelM,
             -sc::intake::kSlideInclineRad,
             0,
             0,
             5.0,
             6.0};
  m_intakeRoller = {sc::intake::kRollerInertia, 2, 0.02};
  m_feeder = {sc::hopper::kFeederInertia, 1, 0.02};
  m_indexer = {sc::hopper::kIndexerInertia, 1, 0.02};

  // Drivetrain, configured the way Phoenix configures the real modules.
  m_swerve = std::make_unique<mayhem::sim::SwerveSim>(MakeSwerveParams(m_perturbation));
  const auto& m = TunerConstants::FrontLeft;
  auto constants = mayhem::sim::swerve::DrivetrainConstants::From(
      MakeSwerveParams(m_perturbation), TunerConstants::kSpeedAt12Volts.value());
  mayhem::sim::swerve::ConfigureLikePhoenix(
      *m_swerve, constants, mayhem::sim::FromPhoenix(m.SteerMotorGains),
      mayhem::sim::FromPhoenix(m.DriveMotorGains), m.SlipCurrent.value(),
      mayhem::sim::FromPhoenix(m.DriveMotorInitialConfigs),
      mayhem::sim::FromPhoenix(m.SteerMotorInitialConfigs));

  // Field and FUEL.
  m_fieldEnabled = g_nextField;
  m_fuel = std::make_unique<mayhem::sim::GamePieces>(MakeBallParams());
  if (m_fieldEnabled) {
    rebuilt::BuildField(m_terrain, m_obstacles);
    for (const auto& p : rebuilt::StagedFuel()) {
      m_fuel->Spawn(p[0], p[1]);
    }
  }
  m_robotIndex = m_fuel->AddRobot(m_swerve.get());
  mayhem::sim::IntakeZone intake;
  intake.name = "intake";
  intake.robot = m_robotIndex;
  intake.xMin = sc::robot::kBumperLengthM / 2 - 0.02;
  intake.xMax = intake.xMin;
  intake.yMin = -sc::intake::kCaptureHalfWidthM;
  intake.yMax = sc::intake::kCaptureHalfWidthM;
  intake.zMin = 0;
  intake.zMax = 0.25;
  intake.canAccept = [this] { return HopperCount() < sc::hopper::kCapacity; };
  intake.onIntake = [this](int) { m_metrics.intaked++; };
  m_intakeIndex = m_fuel->AddIntake(intake);
  Preload(sc::hopper::kPreload);
  for (bool red : {false, true}) {
    const auto c = rebuilt::HubCenter(red);
    mayhem::sim::GoalVolume hub;
    hub.name = red ? "hub_red" : "hub_blue";
    hub.cx = c[0];
    hub.cy = c[1];
    hub.radius = rebuilt::kHubOpeningRadiusM - sc::fuel::kRadiusM;
    hub.zMin = rebuilt::kHubGoalBottomM;
    hub.zMax = rebuilt::kHubTopM;
    hub.onScore = [this, red](int ball, int) {
      const bool weAreRed =
          frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kBlue) ==
          frc::DriverStation::Alliance::kRed;
      (red == weAreRed ? m_metrics.scoredOwnHub : m_metrics.scoredOtherHub)++;
      // Scored FUEL comes back out of the hub into the neutral zone.
      m_respawns.push_back({ball, m_time + 1.0, red});
    };
    m_fuel->AddGoal(hub);
  }

  // Starting pose: open carpet in the blue alliance zone, facing red.
  SetTruePose(frc::Pose2d{3_m, 2_m, frc::Rotation2d{}});
  m_kickerAngle = sc::intake::kKickerStowedRad;
  m_robotHeight = sc::robot::kStowedHeightM;
  Shots();  // calibrate once per process, at construction rather than mid-run
}

void PhysicsWorld::AttachMotor(Mechanism mechanism, ctre::phoenix6::hardware::TalonFX& device,
                               const ctre::phoenix6::configs::TalonFXConfiguration& config,
                               const DcMotor& trapezoidal, const DcMotor& foc) {
  auto motor = std::make_unique<MotorEntry>(
      MotorEntry{&device, device.GetDeviceID(), mechanism,
                 TalonFx{trapezoidal, foc, mayhem::sim::FromPhoenix(config)}});
  m_motors.push_back(std::move(motor));
}

PhysicsWorld::MotorEntry* PhysicsWorld::Find(
    const ctre::phoenix6::hardware::TalonFX& device) const {
  for (const auto& m : m_motors) {
    if (m->device == &device) {
      return m.get();
    }
  }
  return nullptr;
}

const TalonFx& PhysicsWorld::Motor(const ctre::phoenix6::hardware::TalonFX& device) const {
  const auto* m = Find(device);
  if (!m) {
    throw std::logic_error("PhysicsWorld: motor was never attached");
  }
  return m->emulator;
}

void PhysicsWorld::SetMechanismPosition(const ctre::phoenix6::hardware::TalonFX& device,
                                        double rotations) {
  if (auto* m = Find(device)) {
    m->emulator.SetMechanismPosition(rotations);
  }
}

frc::Pose2d PhysicsWorld::TruePose() const { return mayhem::sim::ToPose2d(m_swerve->Pose()); }

frc::Pose3d PhysicsWorld::TruePose3d() const { return mayhem::sim::ToPose3d(m_swerve->Pose3()); }

void PhysicsWorld::SetTruePose(const frc::Pose2d& pose) {
  m_swerve->SetPose(pose.X().value(), pose.Y().value(), pose.Rotation().Radians().value(),
                    &m_terrain);
}

void PhysicsWorld::OnOdometryReset(const frc::Pose2d& pose) {
  if (m_keepPoseOnReset) {
    return;
  }
  SetTruePose(
      frc::Pose2d{pose.X() + units::meter_t{m_perturbation.startX},
                  pose.Y() + units::meter_t{m_perturbation.startY},
                  pose.Rotation() + frc::Rotation2d{units::radian_t{m_perturbation.startYaw}}});
}

int PhysicsWorld::HopperCount() const { return m_fuel->HeldCount(m_robotIndex); }

void PhysicsWorld::Preload(int n) {
  for (int i = 0; i < n; i++) {
    const int ball = m_fuel->Spawn(0, 0);
    m_fuel->SetHeld(ball, m_robotIndex);
  }
}

double PhysicsWorld::SlideExtension() const {
  return const_cast<mayhem::sim::Elevator&>(m_slide).Position();
}

double PhysicsWorld::HoodAngle() const { return const_cast<mayhem::sim::Arm&>(m_hood).Angle(); }

// ------------------------------------------------------------------------------------------------

void PhysicsWorld::Step(double loopSeconds) {
  const int steps = std::max(1, static_cast<int>(std::lround(loopSeconds / kDt)));
  const double start = frc::Timer::GetFPGATimestamp().value();
  ReadControls();
  // A disabled robot's TalonFXs output neutral (brake or coast, as configured), whatever they were
  // last told; the swerve request is re-applied every tick, so it is overridden there too.
  const bool enabled = frc::DriverStation::IsEnabled();
  for (int k = 0; k < steps; k++) {
    // The bus the devices see: the battery's terminal voltage through the wiring and the
    // controllers' bus capacitance (Battery::BusVoltage, low-passed so 300 A of demand and
    // regeneration cannot alternate step to step).
    const double bus = m_battery.BusVoltage();
    // roboRIO brownout (Robot sets the threshold to 4.75 V): motor outputs are disabled until the
    // battery recovers.
    if (bus < kBrownoutVolts) {
      m_brownedOut = true;
      m_metrics.brownedOut = true;
    } else if (bus > kBrownoutVolts + 0.5) {
      m_brownedOut = false;
    }
    if (k % kTicksPerControl == 0 && m_driveHooks) {
      m_driveHooks->ControlTick(start + k * kDt);
    }
    if (m_brownedOut || !enabled) {
      for (std::size_t i = 0; i < m_swerve->ModuleCount(); i++) {
        m_swerve->DriveMotor(i).SetControl(mayhem::sim::NeutralOut{});
        m_swerve->SteerMotor(i).SetControl(mayhem::sim::NeutralOut{});
      }
      for (auto& m : m_motors) {
        m->emulator.SetControl(mayhem::sim::NeutralOut{});
      }
    }
    StepMechanisms(kDt, bus);
    const auto pose = m_swerve->Pose3();
    m_hood.SetPitchOffset(-pose.pitch);
    m_slide.SetPitchOffset(-pose.pitch);
    m_swerve->SetRobotHeight(m_robotHeight);
    m_swerve->Step(kDt, bus, &m_terrain, &m_obstacles);
    StepGame(kDt);
    // Signed: regeneration (a flywheel slowing, a robot braking) charges the battery.
    double current = m_swerve->TotalSupplyCurrent();
    for (const auto& m : m_motors) {
      current += m->emulator.SupplyCurrent();
    }
    m_battery.Step(current, kDt);
    m_metrics.maxTotalCurrent = std::max(m_metrics.maxTotalCurrent, current);
    UpdateMetrics(kDt);
    m_time += kDt;
  }
  frc::sim::RoboRioSim::SetVInVoltage(units::volt_t{m_battery.BusVoltage()});
  Publish();
}

void PhysicsWorld::ReadControls() {
  for (auto& m : m_motors) {
    const auto applied = m->device->GetAppliedControl();
    if (!applied) {
      continue;
    }
    const auto request = mayhem::sim::FromApplied(*applied);
    if (!request) {
      continue;
    }
    int leaderId = -1;
    if (const auto* f = std::get_if<mayhem::sim::Follower>(&*request)) {
      leaderId = f->leaderId;
    } else if (const auto* s = std::get_if<mayhem::sim::StrictFollower>(&*request)) {
      leaderId = s->leaderId;
    }
    m->follower = leaderId >= 0;
    if (m->follower) {
      for (auto& other : m_motors) {
        if (other->id == leaderId && other.get() != m.get()) {
          m->emulator.SetLeader(&other->emulator);
        }
      }
    }
    m->emulator.SetControl(*request);
  }
}

void PhysicsWorld::StepMechanisms(double dt, double bus) {
  auto step = [&](Mechanism which, auto& plant) {
    double torque = 0, damping = 0;
    const double position = plant.RotorPosition(), velocity = plant.RotorVelocity();
    // Leaders first: a follower copies its leader's output from this step.
    for (bool followers : {false, true}) {
      for (auto& m : m_motors) {
        if (m->mechanism == which && m->follower == followers) {
          torque += m->emulator.Step(dt, bus, position, velocity);
          damping += m->emulator.RotorDamping();
        }
      }
    }
    plant.Step(dt, torque, damping);
  };
  step(Mechanism::kShooter, m_flywheel);
  step(Mechanism::kHood, m_hood);
  step(Mechanism::kSlide, m_slide);
  step(Mechanism::kIntakeRollers, m_intakeRoller);
  step(Mechanism::kFeeder, m_feeder);
  step(Mechanism::kIndexer, m_indexer);
}

void PhysicsWorld::StepGame(double dt) {
  const double extension = SlideExtension();
  const double fraction = extension / sc::intake::kSlideTravelM;

  // Kicker bar: folded until the slide first goes out, then down for the rest of the match.
  if (fraction > 0.3 || m_kickerAngle < sc::intake::kKickerStowedRad) {
    m_kickerAngle = std::max(0.0, m_kickerAngle - sc::intake::kKickerDeployRadPerSec * dt);
  }

  // Height of the robot (trench clearance): the hopper on the slide drops as it goes out (12 deg
  // rack); the hood's front rollers rise as it opens.
  const double hopperTop = sc::robot::kHopperTopStowedM + kSlideDirZ * extension;
  const double hoodTop =
      sc::robot::kHoodPivotHeightM + sc::robot::kHoodLengthM * std::sin(HoodAngle()) + 0.04;
  m_robotHeight = std::max({hopperTop, hoodTop, sc::robot::kStowedHeightM});

  // Intake: the main roller rides on the slide; it pulls FUEL in once out and spinning.
  auto& zone = m_fuel->Intake(m_intakeIndex);
  const double rollerX = sc::intake::kRollerStowedX + kSlideDirX * extension;
  zone.xMax = std::max(zone.xMin, rollerX + sc::intake::kRollerRadiusM + 0.04);
  zone.enabled =
      fraction >= sc::intake::kMinExtensionFraction &&
      m_intakeRoller.SurfaceSpeed(sc::intake::kRollerRadiusM) >= sc::intake::kMinRollerSurfaceSpeed;

  // Feeding: indexer and feeder both spinning forward move FUEL from the hopper into the shooter.
  const double feederRps = m_feeder.Velocity() / (2 * kPi);
  const double indexerRps = m_indexer.Velocity() / (2 * kPi);
  const bool feeding = feederRps > 30 && indexerRps > 30 && HopperCount() > 0;
  m_feeder.SetLoadTorque(feeding ? 0.15 : 0.02);
  m_indexer.SetLoadTorque(feeding ? 0.15 : 0.02);
  if (feeding) {
    m_feedProgress += sc::hopper::kFeedRateAt60Rps * std::min(feederRps, indexerRps) / 60 * dt;
    while (m_feedProgress >= 1 && HopperCount() > 0) {
      m_feedProgress -= 1;
      LaunchBall();
    }
  } else {
    m_feedProgress = 0;
  }

  // Scored FUEL leaves the hub on its neutral-zone side.
  for (auto it = m_respawns.begin(); it != m_respawns.end();) {
    if (it->at <= m_time) {
      const auto c = rebuilt::HubCenter(it->red);
      const double side = it->red ? -1 : 1;
      const double spread = static_cast<double>((it->ball * 37) % 11 - 5) * 0.09;
      m_fuel->Place(it->ball, c[0] + side * 0.85, c[1] + spread);
      it = m_respawns.erase(it);
    } else {
      ++it;
    }
  }

  m_fuel->Step(dt, &m_terrain, &m_obstacles);
}

void PhysicsWorld::LaunchBall() {
  const auto& model = Shots();
  const auto pose = m_swerve->Pose3();
  const double c = std::cos(pose.yaw), s = std::sin(pose.yaw);
  // Lanes across the wide shooter, fed in turn.
  const double ex = sc::shooter::kExitX;
  const double ey = sc::shooter::kExitY + sc::shooter::kLaneY[m_lane];
  m_lane = (m_lane + 1) % sc::shooter::kLaneY.size();
  const Vec3 position{pose.x + c * ex - s * ey, pose.y + s * ex + c * ey,
                      pose.z + sc::shooter::kExitZ};
  const double hoodRotations = HoodAngle() / (2 * kPi);
  const double theta = model.angleAtZero - model.anglePerRotation * hoodRotations;
  const double surface = m_flywheel.Velocity() * sc::shooter::kFlywheelRadiusM;
  const double speed =
      std::max(0.5, model.exitSpeedRatio * surface * m_perturbation.shotSpeedScale);
  // Out the back (the shooter side), plus the robot's own velocity at the exit point.
  const double heading = pose.yaw + kPi;
  const auto v = m_swerve->FieldVelocity();
  const double rx = position[0] - pose.x, ry = position[1] - pose.y;
  const Vec3 velocity{speed * std::cos(theta) * std::cos(heading) + v.vx - v.omega * ry,
                      speed * std::cos(theta) * std::sin(heading) + v.vy + v.omega * rx,
                      speed * std::sin(theta)};
  const double w = sc::shooter::kBackspinPerSpeed * speed;
  const Vec3 spin{w * std::sin(heading), -w * std::cos(heading), 0};
  m_fuel->LaunchHeld(m_robotIndex, position, velocity, spin);
  m_flywheel.RemoveEnergy(0.5 * sc::fuel::kMassKg * speed * speed *
                          sc::shooter::kShotEnergyLossFactor);
  m_metrics.shotsFired++;
}

void PhysicsWorld::UpdateMetrics(double dt) {
  std::set<std::string> now;
  for (const auto& e : m_swerve->Contacts()) {
    const std::string name{e.name};
    m_metrics.maxImpulse = std::max(m_metrics.maxImpulse, e.impulse);
    if (e.impulse < kHitImpulse) {
      continue;
    }
    now.insert(name);
    if (m_touching.count(name)) {
      continue;  // the same hit, still in contact
    }
    m_metrics.contacts[name]++;
    {
      const auto p = m_swerve->Pose3();
      const auto v = m_swerve->FieldVelocity();
      char text[200];
      std::snprintf(text, sizeof(text), "%s at t=%.2f s, robot (%.2f, %.2f, %.0f deg) at %.2f m/s",
                    name.c_str(), m_time, p.x, p.y, p.yaw * 180 / kPi, std::hypot(v.vx, v.vy));
      m_metrics.hits.emplace_back(text);
    }
    if (name.rfind(rebuilt::kPerimeterName, 0) == 0) {
      m_metrics.wallHits++;
      m_metrics.worstWallImpulse = std::max(m_metrics.worstWallImpulse, e.impulse);
    } else if (name.rfind(rebuilt::kTrenchArmPrefix, 0) == 0) {
      m_metrics.trenchArmHits++;
    } else {
      m_metrics.obstacleHits++;
    }
  }
  m_touching = std::move(now);
  if (m_swerve->Beached()) {
    if (!m_wasBeached) {
      const auto p = m_swerve->Pose3();
      const auto v = m_swerve->FieldVelocity();
      char text[200];
      std::snprintf(
          text, sizeof(text),
          "beached at t=%.2f s, robot (%.2f, %.2f, %.0f deg) at %.2f m/s, pitch %.1f roll %.1f deg",
          m_time, p.x, p.y, p.yaw * 180 / kPi, std::hypot(v.vx, v.vy), p.pitch * 180 / kPi,
          p.roll * 180 / kPi);
      m_metrics.hits.emplace_back(text);
    }
    m_metrics.beachedSeconds += dt;
  }
  m_wasBeached = m_swerve->Beached();
  m_metrics.maxTiltDeg = std::max(m_metrics.maxTiltDeg, m_swerve->TiltAngle() * 180 / kPi);
  m_metrics.tippedOver |= m_swerve->TippedOver();
  m_metrics.minBatteryVolts = std::min(m_metrics.minBatteryVolts, m_battery.BusVoltage());
}

void PhysicsWorld::Publish() {
  m_poseLog.Set(TruePose3d());
  const double extension = SlideExtension();
  m_componentsLog.Set(std::vector<frc::Pose3d>{
      frc::Pose3d{frc::Translation3d{units::meter_t{kSlideDirX * extension}, 0_m,
                                     units::meter_t{kSlideDirZ * extension}},
                  frc::Rotation3d{}},
      frc::Pose3d{frc::Translation3d{units::meter_t{sc::robot::kHoodPivotX}, 0_m,
                                     units::meter_t{sc::robot::kHoodPivotHeightM}},
                  frc::Rotation3d{0_rad, units::radian_t{-HoodAngle()}, 0_rad}},
      frc::Pose3d{frc::Translation3d{units::meter_t{sc::intake::kKickerPivotX}, 0_m,
                                     units::meter_t{sc::intake::kKickerPivotZ}},
                  frc::Rotation3d{0_rad, units::radian_t{-m_kickerAngle}, 0_rad}},
  });
  m_fuelLog.Set(mayhem::sim::ToTranslations(m_fuel->GroundPositions()));
  m_flyingLog.Set(mayhem::sim::ToTranslations(m_fuel->FlyingPositions()));
  m_hopperLog.Set(HopperCount());
  m_batteryLog.Set(m_battery.BusVoltage());
  m_currentLog.Set(m_battery.TotalCurrent());
  m_heightLog.Set(m_robotHeight);
  m_tiltLog.Set(m_swerve->TiltAngle() * 180 / kPi);
  m_beachedLog.Set(m_swerve->Beached());
  std::vector<double> loads, slips;
  for (std::size_t i = 0; i < m_swerve->ModuleCount(); i++) {
    loads.push_back(m_swerve->ModuleNormalForce(i));
    slips.push_back(m_swerve->ModuleSlip(i));
  }
  m_wheelLoadLog.Set(loads);
  m_wheelSlipLog.Set(slips);
  std::string contacts;
  for (const auto& e : m_swerve->Contacts()) {
    if (e.impulse >= kHitImpulse) {
      contacts += std::string{e.name} + " ";
    }
  }
  m_contactLog.Set(contacts);
  m_lastContacts = contacts;
  m_shotsLog.Set(m_metrics.shotsFired);
  m_scoredLog.Set(m_metrics.scoredOwnHub);
  m_flywheelLog.Set(m_flywheel.Velocity() / (2 * kPi));
}

// ------------------------------------------------------------------------------------------------

const PhysicsWorld::ShotModel& PhysicsWorld::Shots() {
  /*
   * The robot's hub tables (flywheel rot/s and hood rotations by distance) are tuned on the real
   * robot, so they score. Fit the exit speed ratio and the hood-to-launch-angle line that make
   * them score here: for each candidate ratio and either arc, solve each table row's launch angle
   * (ball falls through the hub opening's centre), fit angle = a - b * hood, keep the best fit.
   * (For this robot the high arc fits the tables to ~0.1 m: it lobs into the hub.)
   */
  static const ShotModel model = [] {
    const auto& speeds = shooter::kHubLUT.Points();
    const double z0 = sc::shooter::kExitZ, target = rebuilt::kHubTopM;
    ShotModel best{sc::shooter::kExitSpeedRatio, sc::shooter::kLaunchAngleAtZero,
                   sc::shooter::kLaunchAnglePerHoodRotation, 1e9};
    for (bool highArc : {true, false}) {
      for (double ratio = 0.25; ratio <= 0.95; ratio += 0.01) {
        std::vector<double> hood, angle, speed, range;
        bool ok = true;
        for (const auto& [distance, rps] : speeds) {
          const double v = ratio * 2 * kPi * rps * sc::shooter::kFlywheelRadiusM;
          const double r = distance + sc::shooter::kExitX;  // exit is that much closer to the hub
          const double t = SolveAngle(v, r, z0, target, highArc);
          if (!std::isfinite(t)) {
            ok = false;
            break;
          }
          hood.push_back(shooter_pivot::kHubLUT.Get(distance));
          angle.push_back(t);
          speed.push_back(v);
          range.push_back(r);
        }
        if (!ok) {
          continue;
        }
        // Least squares angle = a + c * hood (c = -anglePerRotation).
        const double n = static_cast<double>(hood.size());
        double sh = 0, sa = 0, shh = 0, sha = 0;
        for (std::size_t i = 0; i < hood.size(); i++) {
          sh += hood[i];
          sa += angle[i];
          shh += hood[i] * hood[i];
          sha += hood[i] * angle[i];
        }
        const double c = (n * sha - sh * sa) / (n * shh - sh * sh);
        const double a = (sa - c * sh) / n;
        double miss = 0;
        for (std::size_t i = 0; i < hood.size(); i++) {
          const double r = RangeAtHeight(speed[i], a + c * hood[i], z0, target);
          const double e = std::isfinite(r) ? r - range[i] : 3.0;
          miss += e * e;
        }
        miss = std::sqrt(miss / n);
        if (miss < best.rmsMissM) {
          best = {ratio, a, -c, miss};
        }
      }
    }
    std::printf(
        "[sim] shot model fitted to the hub tables: exit speed %.2f x flywheel surface, launch "
        "%.1f deg - %.1f deg per hood rotation, rms miss %.3f m\n",
        best.exitSpeedRatio, best.angleAtZero * 180 / kPi, best.anglePerRotation * 180 / kPi,
        best.rmsMissM);
    return best;
  }();
  return model;
}

}  // namespace sim
