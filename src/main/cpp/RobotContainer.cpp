// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "RobotContainer.h"

#include <cmath>
#include <numbers>
#include <vector>

#include <frc/DriverStation.h>
#include <frc/RobotBase.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "sim/PhysicsWorld.h"
#include "sim/SimConstants.h"
#include "sim/SimMechs.h"
#include "subsystems/linearslide/LinearSlideIOSim.h"
#include "subsystems/linearslide/LinearSlideIOTalonFX.h"
#include "subsystems/rollers/RollerPairIOSim.h"
#include "subsystems/shooter/ShooterIOSim.h"
#include "subsystems/shooter/ShooterIOTalonFX.h"
#include "subsystems/shooterpivot/ShooterPivotIOSim.h"
#include "subsystems/shooterpivot/ShooterPivotIOTalonFX.h"
#include "subsystems/swerve/SwerveConstants.h"
#include "subsystems/vision/VisionConstants.h"
#include "subsystems/vision/VisionIOSim.h"
#include "utils/Util.h"
#ifndef ROBOT_DESKTOP_BUILD
#include "subsystems/vision/VisionIOPhotonVision.h"
#endif

using namespace swerve_constants;

namespace {
bool IsSim() { return frc::RobotBase::IsSimulation(); }

template <typename Sim, typename Real, typename Base>
std::unique_ptr<Base> Choose() {
  if (IsSim()) {
    return std::make_unique<Sim>();
  }
  return std::make_unique<Real>();
}

std::vector<std::unique_ptr<VisionIO>> MakeVisionIO(subsystems::Drivetrain& drivetrain) {
  std::vector<std::unique_ptr<VisionIO>> io;
  // Only the back camera is mounted (front cameras were commented out in the Java code too).
  if (IsSim()) {
    if (constants::feature_flags::kSimulateVision) {
      // The camera sees the robot's true pose (plus noise), not the pose estimate.
      io.push_back(std::make_unique<VisionIOSim>(
          vision::kRobotToBackCam, [] { return sim::PhysicsWorld::Get().TruePose(); },
          sim::PhysicsWorld::Get().GetPerturbation().visionNoise));
    } else {
      io.push_back(std::make_unique<VisionIO>());
    }
  } else {
#ifndef ROBOT_DESKTOP_BUILD
    io.push_back(std::make_unique<VisionIOPhotonVision>(vision::kBackCam, vision::kRobotToBackCam));
#else
    io.push_back(std::make_unique<VisionIO>());
#endif
  }
  return io;
}

/** -(sign(v) * v^2): the driver's squared stick response, inverted like the Java code. */
double SquareInput(double v) { return -(util::Signum(v) * std::pow(v, 2)); }
}  // namespace

RobotContainer::RobotContainer()
    : m_logger{TunerConstants::kSpeedAt12Volts.value()},
      m_intakeRollers{true, MakeIntakeRollersIO(IsSim())},
      m_shooter{true, Choose<ShooterIOSim, ShooterIOTalonFX, ShooterIO>()},
      m_shooterPivot{true, Choose<ShooterPivotIOSim, ShooterPivotIOTalonFX, ShooterPivotIO>()},
      m_linearSlide{true, Choose<LinearSlideIOSim, LinearSlideIOTalonFX, LinearSlideIO>()},
      m_feeder{true, MakeFeederIO(IsSim())},
      m_indexer{true, MakeIndexerIO(IsSim())},
      m_drivetrain{TunerConstants::CreateDrivetrain()},
      m_vision{[this](const frc::Pose2d& pose, units::second_t timestamp,
                      const std::array<double, 3>& stdDevs) {
                 m_drivetrain.AddVisionMeasurement(pose, timestamp, stdDevs);
               },
               MakeVisionIO(m_drivetrain)},
      m_shotCalculator{[this] { return m_drivetrain.State().Pose; },
                       [this] { return m_drivetrain.GetFieldRelativeSpeeds(); },
                       kRobotToShooterTransform},
      m_superstructure{m_indexer,        m_shooterPivot,
                       m_shooter,        m_intakeRollers,
                       m_linearSlide,    m_feeder,
                       m_shotCalculator, m_shotCalculator.GetRobotPoseSupplier()} {
  ConfigureMayhemAutoChooser();
  ConfigureSwerve();
  // Buttons' starting state, so a button already down at startup is not a press (like a WPILib
  // Trigger, which samples its condition when it is bound).
  SampleDriverButtons();
  frc::SmartDashboard::PutData("Field Visualize", &m_field2d);

  if (IsSim()) {
    SimMechs::GetInstance().PublishToNT();
  }
}

void RobotContainer::ConfigureMayhemAutoChooser() {
  // The project's routine order is the chooser order; new/renamed routines appear
  // automatically after regeneration, without a second hand-maintained table.
  for (const auto& definition : mayhem_generated::routines) {
    const auto* def = &definition;
    m_mayhemAutoChooser.AddRoutine(std::string{def->name}, [this, def] {
      return suite_autos::Routine(*def, m_drivetrain, m_superstructure, m_shotCalculator);
    });
  }

  frc::SmartDashboard::PutData("Mayhem Auto Chooser", &m_mayhemAutoChooser);
}

double RobotContainer::VxSupplier() {
  return SquareInput(m_driverController.GetLeftY()) * MaxSpeed;
}

double RobotContainer::VySupplier() {
  return SquareInput(m_driverController.GetLeftX()) * MaxSpeed;
}

ctre::phoenix6::swerve::requests::FieldCentric& RobotContainer::DriveRequest(double maxSpeed,
                                                                             double maxAngular) {
  auto& hid = m_driverController;
  const frc::ChassisSpeeds assisted = m_drivetrain.ApplyTrenchAssist(
      frc::ChassisSpeeds{units::meters_per_second_t{SquareInput(hid.GetLeftY()) * maxSpeed},
                         units::meters_per_second_t{SquareInput(hid.GetLeftX()) * maxSpeed},
                         units::radians_per_second_t{-hid.GetRightX() * maxAngular}});
  return m_drive.WithVelocityX(assisted.vx)
      .WithVelocityY(assisted.vy)
      .WithRotationalRate(assisted.omega);
}

void RobotContainer::ConfigureSwerve() {
  // (WithRotationalRate, not a rotational deadband, as in the Java code; it is overwritten every
  // loop anyway)
  m_drive.WithDeadband(units::meters_per_second_t{deadbandMultiplier * MaxSpeed})
      .WithRotationalRate(units::radians_per_second_t{deadbandMultiplier * MaxAngularRate});
  m_azimuth.WithDeadband(units::meters_per_second_t{deadbandMultiplier * MaxSpeed});
  m_azimuth.HeadingController.EnableContinuousInput(-std::numbers::pi, std::numbers::pi);
  m_azimuth.HeadingController.SetPID(azimuth_targets::aziKP, azimuth_targets::aziKi,
                                     azimuth_targets::aziKD);

  m_drivetrain.RegisterTelemetry([this](const subsystems::Drivetrain::SwerveDriveState& state) {
    m_logger.Telemeterize(state);
  });
}

void RobotContainer::SampleDriverButtons() {
  m_aButton.Update(m_driverController.GetAButton());
  m_bButton.Update(m_driverController.GetBButton());
  m_xButton.Update(m_driverController.GetXButton());
  m_yButton.Update(m_driverController.GetYButton());
  m_leftTrigger.Update(m_driverController.GetLeftTriggerAxis() > 0.5);
  m_leftBumper.Update(m_driverController.GetLeftBumperButton());
  m_rightBumper.Update(m_driverController.GetRightBumperButton());
  m_povUp.Update(m_driverController.GetPOV() == 0);
  m_povDown.Update(m_driverController.GetPOV() == 180);
  m_povRight.Update(m_driverController.GetPOV() == 90);
}

void RobotContainer::UpdateDriverButtons() {
  SampleDriverButtons();

  // Presses while disabled, or during an auto, do nothing (and are not remembered for later).
  if (!frc::DriverStation::IsEnabled() || IsAutoRunning()) {
    m_superstructure.SetOperatorInputs({});
  } else {
    m_superstructure.SetOperatorInputs({.shootHeld = m_aButton.Get(),
                                        .intakePressed = m_xButton.Rose(),
                                        .jitterHeld = m_yButton.Get(),
                                        .deployPressed = m_bButton.Rose(),
                                        .outtakeHeld = m_leftTrigger.Get()});
  }
  if (!frc::DriverStation::IsEnabled()) {
    m_driveMode = DriveMode::kNormal;
    return;
  }

  if (m_povUp.Rose()) {
    m_superstructure.AddShootMultiplier(.02);
  }
  if (m_povDown.Rose()) {
    m_superstructure.AddShootMultiplier(-0.02);
  }

  // A routine owns the drivetrain for all of autonomous: driving buttons wait for it to end.
  if (IsAutoRunning()) {
    m_driveMode = DriveMode::kNormal;
    return;
  }
  // The most recent press wins; releasing the button that set the mode goes back to normal.
  if ((m_driveMode == DriveMode::kSlow && m_leftBumper.Fell()) ||
      (m_driveMode == DriveMode::kSuperSlow && m_rightBumper.Fell()) ||
      (m_driveMode == DriveMode::kAim && m_aButton.Fell())) {
    m_driveMode = DriveMode::kNormal;
  }
  if (m_leftBumper.Rose()) {
    m_driveMode = DriveMode::kSlow;
  }
  if (m_rightBumper.Rose()) {
    m_driveMode = DriveMode::kSuperSlow;
  }
  if (m_povRight.Rose()) {
    m_drivetrain.SeedFieldCentric();
    m_driveMode = DriveMode::kNormal;
  }
  // Held A turns to the shot (aim assist) while the driver translates.
  if (m_aButton.Rose()) {
    m_driveMode = DriveMode::kAim;
  }
}

void RobotContainer::UpdateAutonomous() {
  const bool autoEnabled = frc::DriverStation::IsAutonomousEnabled();
  if (autoEnabled && !m_autoEnabled) {
    m_runningAuto = m_mayhemAutoChooser.SelectedRoutine();
    if (m_runningAuto != nullptr) {
      m_runningAuto->Start();
    }
  } else if (!autoEnabled && m_runningAuto != nullptr) {
    m_runningAuto->Stop();
    m_runningAuto = nullptr;
  }
  m_autoEnabled = autoEnabled;
  if (m_runningAuto != nullptr) {
    m_runningAuto->Periodic();
  }
}

void RobotContainer::UpdateDrive() {
  if (!frc::DriverStation::IsEnabled() || IsAutoRunning()) {
    return;
  }
  switch (m_driveMode) {
    case DriveMode::kNormal:
      m_drivetrain.SetControl(DriveRequest(MaxSpeed, MaxAngularRate));
      break;
    case DriveMode::kSlow:
      m_drivetrain.SetControl(DriveRequest(SlowMaxSpeed, SlowMaxAngular));
      break;
    case DriveMode::kSuperSlow:
      m_drivetrain.SetControl(DriveRequest(SuperSlowMaxSpeed, SlowMaxAngular));
      break;
    case DriveMode::kAim:
      m_drivetrain.RotateToLookahead(m_shotCalculator, VxSupplier(), VySupplier());
      break;
  }
}

RobotContainer::~RobotContainer() {
  // Before any member goes away: the drivetrain holds references to m_drive and the auto
  // factory's request (see Drivetrain::Shutdown).
  m_drivetrain.Shutdown();
  if (IsSim()) {
    // A fresh field and robot for the next robot program (tests build several).
    sim::PhysicsWorld::Destroy();
  }
}

void RobotContainer::BeginLoop() { m_drivetrain.InvalidateStateCache(); }

void RobotContainer::Periodic() {
  // Inputs first: sensors, then vision into the pose estimate.
  m_intakeRollers.Periodic();
  m_shooter.Periodic();
  m_shooterPivot.Periodic();
  m_linearSlide.Periodic();
  m_feeder.Periodic();
  m_indexer.Periodic();
  m_drivetrain.Periodic();
  m_vision.Periodic();

  // Requests, then control.
  UpdateDriverButtons();
  UpdateAutonomous();
  UpdateDrive();
  m_driveModeLog.Set(IsAutoRunning()                        ? "Auto"
                     : m_driveMode == DriveMode::kSlow      ? "Slow"
                     : m_driveMode == DriveMode::kSuperSlow ? "SuperSlow"
                     : m_driveMode == DriveMode::kAim       ? "Aim"
                                                            : "Normal");
  m_superstructure.Periodic();

  m_field2d.SetRobotPose(m_drivetrain.State().Pose);
  m_shotCalculator.Periodic();
  m_shotTargetObject->SetPose(m_superstructure.GetTarget());
  m_lookaheadObject->SetPose(m_shotCalculator.GetLookaheadPose());
  UpdateAutoPreview();
}

void RobotContainer::UpdateAutoPreview() {
  if (!frc::DriverStation::IsDisabled()) {
    if (m_autoPreviewShown) {
      m_autoPathObject->SetPoses(std::span<const frc::Pose2d>{});
      m_autoStartObject->SetPoses(std::span<const frc::Pose2d>{});
      m_autoPreviewShown = false;
      m_autoStartPose.reset();
    }
    return;
  }

  // Rebuild the drawing only when the selection or the alliance changes.
  const std::string& selected = m_mayhemAutoChooser.Selected();
  const bool red = suite_autos::IsRed();
  if (!m_autoPreviewShown || selected != m_autoPreviewName || red != m_autoPreviewRed) {
    m_autoPreviewShown = true;
    m_autoPreviewName = selected;
    m_autoPreviewRed = red;
    m_autoStartPose.reset();
    std::vector<frc::Pose2d> path;
    if (const auto* def = suite_autos::Find(selected)) {
      m_autoStartPose = suite_autos::ToWpi(suite_autos::Resolve(def->startPose, red));
      path = suite_autos::Preview(*def, red);
    }
    m_autoPathObject->SetPoses(path);
    if (m_autoStartPose) {
      m_autoStartObject->SetPose(*m_autoStartPose);
    } else {
      m_autoStartObject->SetPoses(std::span<const frc::Pose2d>{});
    }
  }

  if (m_autoStartPose) {
    const frc::Pose2d& pose = m_drivetrain.State().Pose;
    const double distance = pose.Translation().Distance(m_autoStartPose->Translation()).value();
    const double heading =
        std::abs((pose.Rotation() - m_autoStartPose->Rotation()).Degrees().value());
    m_autoStartDistanceLog.Set(distance);
    m_autoStartHeadingLog.Set(heading);
    m_autoAtStart = distance <= kAutoStartToleranceMeters && heading <= kAutoStartToleranceDegrees;
    m_autoAtStartLog.Set(m_autoAtStart);
  } else {
    m_autoAtStart = false;
  }
}
