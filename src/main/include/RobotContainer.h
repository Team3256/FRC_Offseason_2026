// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <memory>
#include <optional>
#include <string>

#include <frc/geometry/Transform2d.h>
#include <frc/smartdashboard/Field2d.h>
#include <ctre/phoenix6/swerve/SwerveRequest.hpp>

#include "Constants.h"
#include "Telemetry.h"
#include "commands/AutoChooser.h"
#include "commands/MayhemAutos.h"
#include "subsystems/Superstructure.h"
#include "subsystems/feeder/Feeder.h"
#include "subsystems/indexer/Indexer.h"
#include "subsystems/intakerollers/IntakeRollers.h"
#include "subsystems/linearslide/LinearSlide.h"
#include "subsystems/shooter/Shooter.h"
#include "subsystems/shooterpivot/ShooterPivot.h"
#include "subsystems/sotm/ShotCalculator.h"
#include "subsystems/swerve/CommandSwerveDrivetrain.h"
#include "subsystems/vision/Vision.h"
#include "utils/MappedXboxController.h"

class RobotContainer {
 public:
  RobotContainer();
  ~RobotContainer();

  /** Call at the very start of each robot loop (before the scheduler runs). */
  void BeginLoop();
  /** Call after the scheduler has run. */
  void Periodic();

  // Accessors for tests and simulation tooling.
  Superstructure& GetSuperstructure() { return m_superstructure; }
  subsystems::CommandSwerveDrivetrain& GetDrivetrain() { return m_drivetrain; }
  ShotCalculator& GetShotCalculator() { return m_shotCalculator; }
  Shooter& GetShooter() { return m_shooter; }
  ShooterPivot& GetShooterPivot() { return m_shooterPivot; }
  LinearSlide& GetLinearSlide() { return m_linearSlide; }
  IntakeRollers& GetIntakeRollers() { return m_intakeRollers; }
  Feeder& GetFeeder() { return m_feeder; }
  Indexer& GetIndexer() { return m_indexer; }
  Vision& GetVision() { return m_vision; }
  AutoChooser& GetAutoChooser() { return m_mayhemAutoChooser; }
  frc::Field2d& GetField() { return m_field2d; }
  /** Start pose of the selected auto (alliance-correct) while disabled, for lining up. */
  const std::optional<frc::Pose2d>& GetAutoStartPose() const { return m_autoStartPose; }
  /** True while the robot is within tolerance of the selected auto's start pose. */
  bool IsAtAutoStartPose() const { return m_autoAtStart; }

  /** Robot within this distance and heading of the auto's start pose counts as lined up. */
  static constexpr double kAutoStartToleranceMeters = 0.15;
  static constexpr double kAutoStartToleranceDegrees = 5.0;

 private:
  void ConfigureMayhemAutoChooser();
  void ConfigureOperatorBinds();
  void ConfigureSwerve();
  /** Draws the selected auto on the field while disabled and reports how far off its start the
   * robot is. */
  void UpdateAutoPreview();
  double VxSupplier();
  double VySupplier();
  /** Driver request with trench assist, scaled to the given max speeds. */
  ctre::phoenix6::swerve::requests::FieldCentric& DriveRequest(double maxSpeed, double maxAngular);

  // One chooser for all Mayhem autonomous routines.
  AutoChooser m_mayhemAutoChooser;

 public:
  MappedXboxController m_driverController{constants::controller::kDriverControllerPort, "Driver"};
  MappedXboxController m_operatorController{constants::controller::kOperatorControllerPort,
                                            "Operator"};

 private:
  Telemetry m_logger;

  IntakeRollers m_intakeRollers;
  Shooter m_shooter;
  ShooterPivot m_shooterPivot;
  LinearSlide m_linearSlide;
  Feeder m_feeder;
  Indexer m_indexer;

  subsystems::CommandSwerveDrivetrain m_drivetrain;

  static inline const frc::Transform2d kRobotToShooterTransform{0_m, 0_m, frc::Rotation2d{}};

  Vision m_vision;
  ShotCalculator m_shotCalculator;

  frc::Field2d m_field2d;
  frc::FieldObject2d* m_autoPathObject = m_field2d.GetObject("AutoPath");
  frc::FieldObject2d* m_autoStartObject = m_field2d.GetObject("AutoStart");
  frc::FieldObject2d* m_shotTargetObject = m_field2d.GetObject("ShotTarget");
  frc::FieldObject2d* m_lookaheadObject = m_field2d.GetObject("ShotLookahead");
  bool m_autoPreviewShown = false;
  std::string m_autoPreviewName;
  bool m_autoPreviewRed = false;
  std::optional<frc::Pose2d> m_autoStartPose;
  bool m_autoAtStart = false;
  logging::Output<double> m_autoStartDistanceLog{"Auto/StartPoseErrorMeters"};
  logging::Output<double> m_autoStartHeadingLog{"Auto/StartPoseErrorDegrees"};
  logging::Output<bool> m_autoAtStartLog{"Auto/AtStartPose"};

  Superstructure m_superstructure;

  ctre::phoenix6::swerve::requests::FieldCentric m_drive;
  ctre::phoenix6::swerve::requests::FieldCentricFacingAngle m_azimuth;
};
