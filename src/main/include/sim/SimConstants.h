// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <array>
#include <numbers>

/**
 * Physical parameters of the simulated robot and of the 2026 REBUILT game pieces. Everything the
 * physics engine (mayhem::sim, vendored from Mayhem) needs that is not already in the robot's own
 * constants: the motor configs, gear ratios, gains and current limits always come from the real
 * Constants / TunerConstants files, so tuning those in simulation tunes the robot.
 *
 * Values marked "measure" are estimates from the CAD or from the old sim; replace them with
 * measurements to make the sim match the robot. PhysicsWorld::Params() holds a copy of these that
 * a test may perturb (Monte Carlo).
 */
namespace sim_constants {

inline constexpr double kLbToKg = 0.45359237;
inline constexpr double kInToM = 0.0254;

/** Fixed physics step. The robot loop (20 ms) runs 20 of these. */
inline constexpr double kPhysicsDt = 0.001;

namespace robot {
/**
 * Mass properties. Onshape (00.Main, parts with materials, battery 12.89 lb included): 55.1 kg,
 * centre of mass 0.207 m up and 0.027 m behind the frame centre, yaw inertia 6.07 kg m^2 about it.
 * The robot weighs 150 lb with bumpers and battery, so the remaining 12.9 kg is bumpers (~15 lb,
 * a ring at the perimeter, centre 0.09 m up) plus parts without a material in CAD (spread like
 * the rest of the robot).
 */
inline constexpr double kCadMassKg = 55.10;
inline constexpr double kBumperMassKg = 15 * kLbToKg;
inline constexpr double kMassKg = 150 * kLbToKg;
inline constexpr double kUnmodelledMassKg = kMassKg - kCadMassKg - kBumperMassKg;
inline constexpr double kCgHeightM =
    ((kCadMassKg + kUnmodelledMassKg) * 0.207 + kBumperMassKg * 0.09) / kMassKg;
/** Centre of mass ahead of the frame centre (negative: behind). */
inline constexpr double kCgForwardM = -0.027 * (kCadMassKg + kUnmodelledMassKg) / kMassKg;
/** CAD yaw inertia scaled for the unmodelled parts, plus the bumper ring (m a^2 / 6). */
inline constexpr double kMoiKgM2 =
    6.07 * (kCadMassKg + kUnmodelledMassKg) / kCadMassKg + kBumperMassKg * 0.813 * 0.813 / 6;
/** Roll/pitch inertias (CAD 4.25 / 4.49 kg m^2 about the CoM), scaled the same way. */
inline constexpr double kRollMoiKgM2 =
    4.25 * (kCadMassKg + kUnmodelledMassKg) / kCadMassKg + kBumperMassKg * 0.813 * 0.813 / 12;
inline constexpr double kPitchMoiKgM2 =
    4.49 * (kCadMassKg + kUnmodelledMassKg) / kCadMassKg + kBumperMassKg * 0.813 * 0.813 / 12;
/** Bellypan bottom above the carpet (CAD 0102_Drivebase_Bellypan bottom at z = 0.041). */
inline constexpr double kFrameClearanceM = 0.041;
/** Bumper-to-bumper size (robot.mayhem.json). */
inline constexpr double kBumperLengthM = 0.813;
inline constexpr double kBumperWidthM = 0.813;
/** Top of the robot with the hood stowed (CAD: shooter assembly top at 0.547 m). */
inline constexpr double kStowedHeightM = 0.55;
/**
 * Top of the hopper with the slide in. The real hopper was cut down after the CAD (whose
 * polycarbonate reaches 0.61 m) so it sits just under the trench's 22.25 in (0.565 m) opening
 * (team-confirmed); tools/sim/trim_hopper.py trims the AdvantageScope model to the same height.
 * It drops 0.064 m as the slide goes out (12 deg rack).
 */
inline constexpr double kHopperTopStowedM = 0.555;
/**
 * Hood ("pivot" frame: main shooter roller at the back, two mini rollers at the front). It pivots
 * about the main roller axle (team-confirmed); the front mini rollers swing. Robot frame.
 */
inline constexpr double kHoodPivotX = -0.283;
inline constexpr double kHoodPivotHeightM = 0.494;
/** Pivot to the front mini rollers (CAD), which set the robot's height when the hood is up. */
inline constexpr double kHoodLengthM = 0.205;
}  // namespace robot

namespace swerve {
/** Tread friction on carpet. 1.1 is robot_limits.py's traction estimate. measure */
inline constexpr double kMuStatic = 1.15;
inline constexpr double kMuKinetic = 1.0;
/** Slip speed at which the tread reaches full friction (shapes the stick-slip curve). */
inline constexpr double kSlipVelocity = 0.05;
/** Rolling resistance coefficient (carpet, 4 in tread). measure */
inline constexpr double kRollingResistance = 0.012;
inline constexpr double kDriveEfficiency = 0.95;
/** Wheel + gears + reflected Kraken rotor inertia, at the wheel (kg m^2). */
inline constexpr double kWheelInertia = 0.0035;
/** Steering inertia about the steer axis (TunerConstants kSteerInertia). */
inline constexpr double kSteerInertia = 0.01;
}  // namespace swerve

namespace shooter {
/**
 * Flywheel inertia at the motor shaft (gearing 1). Onshape BOM: one 4 in brass flywheel 1.154 lb
 * (0.5 m r^2 = 7.6e-4), aluminium main roller 0412 3.225 lb and its PLA hub 1.961 lb (~1.3e-3 as a
 * 2 in tube), plus four Kraken rotors (~1e-4). measure if the gearing between them is not 1:1
 */
inline constexpr double kFlywheelInertia = 0.0022;
inline constexpr double kFlywheelRadiusM = 2 * kInToM;
inline constexpr double kFlywheelViscous = 2e-5;  // N m per rad/s
/** Launch point in the robot frame: over the main roller, out the back (the shooter side). */
inline constexpr double kExitX = -0.33;
inline constexpr double kExitY = 0.0;
inline constexpr double kExitZ = 0.55;
/**
 * The shooter is as wide as the robot (main roller 0.55 m, eight compliant wheels across it), so
 * FUEL leaves side by side, not in a single stream: lanes at these robot-frame y offsets, fed in
 * turn. measure (how many balls the indexer presents across the width)
 */
inline constexpr std::array<double, 3> kLaneY = {-0.18, 0.0, 0.18};
/**
 * Exit speed = kExitSpeedRatio * flywheel surface speed, and launch elevation [rad] =
 * kLaunchAngleAtZero - kLaunchAnglePerHoodRotation * hood position [mechanism rotations]. These are
 * calibrated at startup against what was measured on the real robot: the hub lookup tables (they
 * score) and ShotCalculator's time-of-flight table (PhysicsWorld::Shots); the values here are only
 * the starting guess for that fit.
 */
inline constexpr double kExitSpeedRatio = 0.45;
inline constexpr double kLaunchAngleAtZero = 1.25;
inline constexpr double kLaunchAnglePerHoodRotation = 6.0;
/** Backspin [rad/s] per m/s of exit speed (Magnus). measure */
inline constexpr double kBackspinPerSpeed = 8.0;
/** Fraction of a ball's kinetic energy the flywheel loses per shot beyond the ball's own energy. */
inline constexpr double kShotEnergyLossFactor = 1.6;
}  // namespace shooter

namespace intake {
/**
 * The slide: two rack plates with the hopper and main roller, driven out by pinions. From the CAD
 * (intake-only export fully out vs the stowed robot export): 0.309 m along a rack inclined 12 deg
 * down toward the front, reached at LinearSlide kIntakeFinalPosition (0.86 mechanism rotations).
 */
inline constexpr double kSlideTravelM = 0.309;
inline constexpr double kSlideInclineRad = 12.0 * std::numbers::pi / 180;
inline constexpr double kSlideFullRotations = 0.86;
/** Main intake roller centre when stowed (robot frame; CAD y -0.292, z 0.2395). */
inline constexpr double kRollerStowedX = 0.292;
inline constexpr double kRollerStowedZ = 0.2395;
/**
 * Kicker bar: frame-mounted arms pivoting at (x 0.1525, z 0.0825), bars 0.351 m out. Folded up
 * against the stowed intake until the slide first deploys, then down (0) for the rest of the
 * match. The folded angle is an estimate. measure
 */
inline constexpr double kKickerPivotX = 0.1525;
inline constexpr double kKickerPivotZ = 0.0825;
/** Onshape revolute limit (Kicker Bar Arm, 0 to -72 deg about CAD +x). */
inline constexpr double kKickerStowedRad = 72.0 * std::numbers::pi / 180;
inline constexpr double kKickerDeployRadPerSec = 6.0;
inline constexpr double kSlideMassKg = 3.0;
inline constexpr double kSlideDrumRadiusM = 0.5 * kInToM;
/** Capture box in front of the bumper (robot frame) when the slide is out. */
inline constexpr double kCaptureDepthM = 0.12;
inline constexpr double kCaptureHalfWidthM = 0.30;
/** Minimum roller surface speed to pull a ball in (m/s), and the roller radius. */
inline constexpr double kMinRollerSurfaceSpeed = 1.5;
inline constexpr double kRollerRadiusM = 1 * kInToM;
inline constexpr double kRollerInertia = 0.0008;
/** Fraction of slide travel needed before balls can be picked up. */
inline constexpr double kMinExtensionFraction = 0.7;
}  // namespace intake

namespace hood {
/**
 * The pivot frame about the main roller axle (Onshape BOM): the main roller and its hub (5.2 lb)
 * sit on the axle; the two mini rollers (2.6 lb) and the plates (0.6 lb) swing 0.1-0.2 m out.
 * Gravity acts as ~1.5 kg at 0.17 m; inertia ~ 1.2 kg x 0.205^2 + plates.
 */
inline constexpr double kMassKg = 1.5;
inline constexpr double kComLengthM = 0.17;
inline constexpr double kInertia = 0.06;
inline constexpr double kMinAngleRad = 0;
inline constexpr double kMaxAngleRad = 42.152 * std::numbers::pi / 180;
}  // namespace hood

namespace hopper {
/** Balls the hopper holds (CAD volume estimate). measure */
inline constexpr int kCapacity = 40;
/** Balls preloaded at the start of auto (game manual: up to 8). */
inline constexpr int kPreload = 8;
/** Balls fed into the shooter per second (all lanes) with feeder + indexer at 60 rot/s. measure */
inline constexpr double kFeedRateAt60Rps = 12.0;
inline constexpr double kFeederInertia = 0.0015;
inline constexpr double kIndexerInertia = 0.0015;
}  // namespace hopper

namespace fuel {
/** FUEL: 5.91 in high-density foam ball, ~0.5 lb (game manual). */
inline constexpr double kRadiusM = 5.91 / 2 * kInToM;
inline constexpr double kMassKg = 0.215;
inline constexpr double kRestitution = 0.55;
inline constexpr double kRollingFriction = 0.06;
inline constexpr double kDragCoefficient = 0.50;
inline constexpr double kMagnusCoefficient = 0.25;
}  // namespace fuel

namespace battery {
inline constexpr double kOpenCircuitVolts = 12.6;
inline constexpr double kInternalResistanceOhms = 0.015;
inline constexpr double kWiringResistanceOhms = 0.008;
}  // namespace battery

namespace vision {
/** Standard deviation of the simulated camera's pose noise (m), as VisionIOSim used. */
inline constexpr double kLinearNoiseStdDev = 0.02;
}  // namespace vision

}  // namespace sim_constants
