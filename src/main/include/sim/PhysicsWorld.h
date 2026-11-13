// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Translation3d.h>
#include <ctre/phoenix6/TalonFX.hpp>

#include "logging/Logger.h"
#include "mayhem/sim/battery.hpp"
#include "mayhem/sim/field.hpp"
#include "mayhem/sim/game_pieces.hpp"
#include "mayhem/sim/mechanisms.hpp"
#include "mayhem/sim/swerve.hpp"
#include "mayhem/sim/talonfx.hpp"

namespace sim {

/** The simulated mechanisms, one plant each. */
enum class Mechanism { kShooter, kHood, kSlide, kIntakeRollers, kFeeder, kIndexer };

/**
 * The drivetrain's side of the simulation. Phoenix applies the drivetrain's request and samples
 * odometry on its 250 Hz odometry thread; the world calls this every 4 ms of simulated time
 * instead.
 */
class DriveHooks {
 public:
  virtual ~DriveHooks() = default;
  /** Apply the current swerve request to the simulated modules and update odometry. */
  virtual void ControlTick(double fpgaSeconds) = 0;
};

/**
 * Perturbations of the nominal robot, for Monte Carlo runs. Applied when the world is created
 * (PhysicsWorld::SetNextPerturbation before the robot is built) or to the start pose when
 * autonomous resets odometry.
 */
struct Perturbation {
  double massScale = 1, muScale = 1, batteryResistanceScale = 1, batteryVoltageOffset = 0;
  /** Added to the start pose the robot is placed at (field frame, m and rad). */
  double startX = 0, startY = 0, startYaw = 0;
  /** Std dev of the simulated camera's pose noise (m). */
  double visionNoise = 0.02;
  /** Multiplies the exit speed of every shot (worn wheels, ball variation). */
  double shotSpeedScale = 1;
};

/** What happened during a run, for tests and Monte Carlo reports. */
struct Metrics {
  int shotsFired = 0, intaked = 0, scoredOwnHub = 0, scoredOtherHub = 0;
  int wallHits = 0, trenchArmHits = 0, obstacleHits = 0;
  double maxImpulse = 0, worstWallImpulse = 0;
  double beachedSeconds = 0, maxTiltDeg = 0, minBatteryVolts = 99, maxTotalCurrent = 0;
  bool tippedOver = false, brownedOut = false;
  /** Obstacle name -> number of hits (contacts with more than ~300 N). */
  std::map<std::string, int> contacts;
  /** Each hit: "<obstacle> at t=<s> robot (x, y, heading) speed <m/s>", in order. */
  std::vector<std::string> hits;
};

/**
 * The simulated robot and field: mayhem::sim (vendored from Mayhem) running the drivetrain,
 * every mechanism, the battery, the REBUILT field and all FUEL, on simulated time. The robot code
 * keeps commanding its real Phoenix devices; each loop the world reads every motor's applied
 * control (TalonFX::GetAppliedControl) into Mayhem's TalonFX emulator, configured from the same
 * TalonFXConfiguration, and steps 20 x 1 ms. Nothing depends on the wall clock, so a test can run
 * the robot loop as fast as the code allows.
 *
 * Exists only in simulation: created by the first Get(), destroyed with the robot
 * (RobotContainer's destructor), so each test robot starts from a fresh field.
 */
class PhysicsWorld {
 public:
  static PhysicsWorld& Get();
  static bool Exists();
  static void Destroy();
  /** Perturbation for the next world created (Monte Carlo). */
  static void SetNextPerturbation(const Perturbation& p);
  /**
   * Whether the next world has the REBUILT field (walls, hubs, bumps, trenches, staged FUEL) or is
   * an empty carpet (default: the field; tests of robot logic that place the robot anywhere use
   * the empty carpet).
   */
  static void SetNextFieldEnabled(bool enabled);
  bool FieldEnabled() const { return m_fieldEnabled; }

  /** Simulate a motor of {@code mechanism}, configured like the real one. */
  void AttachMotor(Mechanism mechanism, ctre::phoenix6::hardware::TalonFX& device,
                   const ctre::phoenix6::configs::TalonFXConfiguration& config,
                   const mayhem::sim::DcMotor& trapezoidal, const mayhem::sim::DcMotor& foc);
  /** The emulator standing in for {@code device} (sensors for the IO's UpdateInputs). */
  const mayhem::sim::TalonFx& Motor(const ctre::phoenix6::hardware::TalonFX& device) const;
  /** TalonFX::SetPosition (a config call, not a control request). */
  void SetMechanismPosition(const ctre::phoenix6::hardware::TalonFX& device, double rotations);

  mayhem::sim::SwerveSim& Swerve() { return *m_swerve; }
  void SetDriveHooks(DriveHooks* hooks) { m_driveHooks = hooks; }

  /** Runs one robot loop of physics (20 x 1 ms). Called from Robot::SimulationPeriodic. */
  void Step(double loopSeconds);

  /** The robot's true pose (the estimate is the drivetrain's). */
  frc::Pose2d TruePose() const;
  frc::Pose3d TruePose3d() const;
  /** Places the robot (zero velocity). */
  void SetTruePose(const frc::Pose2d& pose);
  /**
   * Drivetrain::ResetPose calls this: the robot is moved to the reset pose (plus the
   * perturbation's start offset), as if it had been placed there, unless KeepPoseOnReset is set.
   */
  void OnOdometryReset(const frc::Pose2d& pose);
  void KeepPoseOnReset(bool keep) { m_keepPoseOnReset = keep; }

  int HopperCount() const;
  /** Puts {@code n} FUEL in the hopper (the preload). */
  void Preload(int n);
  double BatteryVolts() const { return m_battery.BusVoltage(); }
  double RobotHeight() const { return m_robotHeight; }
  double SlideExtension() const;
  double HoodAngle() const;
  double KickerAngle() const { return m_kickerAngle; }
  const Metrics& GetMetrics() const { return m_metrics; }
  const Perturbation& GetPerturbation() const { return m_perturbation; }
  double SimSeconds() const { return m_time; }
  mayhem::sim::GamePieces& Fuel() { return *m_fuel; }

  /** Launch model fitted to the robot's hub lookup tables (see Calibrate). */
  struct ShotModel {
    double exitSpeedRatio, angleAtZero, anglePerRotation;
    double rmsMissM;
  };
  static const ShotModel& Shots();

 private:
  PhysicsWorld();
  struct MotorEntry;

  void ReadControls();
  void StepMechanisms(double dt, double bus);
  void StepGame(double dt);
  void LaunchBall();
  void UpdateMetrics(double dt);
  void Publish();
  MotorEntry* Find(const ctre::phoenix6::hardware::TalonFX& device) const;

  Perturbation m_perturbation;
  std::vector<std::unique_ptr<MotorEntry>> m_motors;
  mayhem::sim::Flywheel m_flywheel;
  mayhem::sim::Arm m_hood;
  mayhem::sim::Elevator m_slide;
  mayhem::sim::Roller m_intakeRoller, m_feeder, m_indexer;
  std::unique_ptr<mayhem::sim::SwerveSim> m_swerve;
  mayhem::sim::Battery m_battery;
  mayhem::sim::Terrain m_terrain;
  mayhem::sim::Obstacles m_obstacles;
  std::unique_ptr<mayhem::sim::GamePieces> m_fuel;
  int m_robotIndex = 0, m_intakeIndex = 0;
  DriveHooks* m_driveHooks = nullptr;
  bool m_keepPoseOnReset = false;
  bool m_fieldEnabled = true;
  bool m_brownedOut = false;
  bool m_wasBeached = false;
  std::size_t m_lane = 0;

  double m_time = 0, m_tickAccum = 0, m_feedProgress = 0, m_kickerAngle = 0, m_robotHeight = 0;
  double m_beachedTimer = 0;
  struct PendingRespawn {
    int ball;
    double at;
    bool red;
  };
  std::vector<PendingRespawn> m_respawns;
  Metrics m_metrics;
  std::set<std::string> m_touching;
  std::string m_lastContacts;

  logging::Output<frc::Pose3d> m_poseLog{"Sim/RobotPose"};
  logging::Output<std::vector<frc::Pose3d>> m_componentsLog{"Sim/Components"};
  logging::Output<std::vector<frc::Translation3d>> m_fuelLog{"Sim/Fuel"};
  logging::Output<std::vector<frc::Translation3d>> m_flyingLog{"Sim/FlyingFuel"};
  logging::Output<double> m_hopperLog{"Sim/HopperCount"};
  logging::Output<double> m_batteryLog{"Sim/BatteryVolts"};
  logging::Output<double> m_currentLog{"Sim/TotalCurrentAmps"};
  logging::Output<double> m_heightLog{"Sim/RobotHeightMeters"};
  logging::Output<double> m_tiltLog{"Sim/TiltDegrees"};
  logging::Output<bool> m_beachedLog{"Sim/Beached"};
  logging::Output<std::vector<double>> m_wheelLoadLog{"Sim/WheelNormalForces"};
  logging::Output<std::vector<double>> m_wheelSlipLog{"Sim/WheelSlip"};
  logging::Output<std::string> m_contactLog{"Sim/Contacts"};
  logging::Output<double> m_shotsLog{"Sim/ShotsFired"};
  logging::Output<double> m_scoredLog{"Sim/ScoredOwnHub"};
  logging::Output<double> m_flywheelLog{"Sim/FlywheelRps"};
};

}  // namespace sim
