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
#include <frc2/command/button/RobotModeTriggers.h>

#include "mayhem/CtreSwerve.h"
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
using SS = Superstructure::StructureState;

namespace {
bool IsSim() { return frc::RobotBase::IsSimulation(); }

template <typename Sim, typename Real, typename Base>
std::unique_ptr<Base> Choose() {
  if (IsSim()) {
    return std::make_unique<Sim>();
  }
  return std::make_unique<Real>();
}

std::vector<std::unique_ptr<VisionIO>> MakeVisionIO(
    subsystems::CommandSwerveDrivetrain& drivetrain) {
  std::vector<std::unique_ptr<VisionIO>> io;
  // Only the back camera is mounted (front cameras were commented out in the Java code too).
  if (IsSim()) {
    if (constants::feature_flags::kSimulateVision) {
      io.push_back(std::make_unique<VisionIOSim>(
          vision::kRobotToBackCam, [&drivetrain] { return drivetrain.State().Pose; }));
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
  mayhem::RecoveryConfig recoveryConfig;
  recoveryConfig.persistenceLoops = 10;

  // Mayhem factory shared by both Mayhem auto classes.
  m_mayhemAutoFactory = mayhem::CtreSwerveAutoFactory(m_drivetrain);
  m_mayhemAutoFactory->WithTelemetry(true).WithRecoveryConfig(recoveryConfig);

  m_sweepDepotMayhemAutos = std::make_unique<SweepDepotMayhemAutos>(
      *m_mayhemAutoFactory, m_superstructure, m_drivetrain, m_shotCalculator);
  m_mayhemAutos = std::make_unique<SweepMayhemAutos>(*m_mayhemAutoFactory, m_superstructure,
                                                     m_drivetrain, m_shotCalculator);

  ConfigureMayhemAutoChooser();
  ConfigureSwerve();
  ConfigureOperatorBinds();
  frc::SmartDashboard::PutData("Field Visualize", &m_field2d);

  if (IsSim()) {
    SimMechs::GetInstance().PublishToNT();
  }
}

void RobotContainer::ConfigureMayhemAutoChooser() {
  // Existing Depot routine: both Depot path segments.
  m_mayhemAutoChooser.AddRoutine("Depot Trench Sweep Depot",
                                 [this] { return m_sweepDepotMayhemAutos->TopTrenchSweepDepot(); });
  // Bottom trench options.
  m_mayhemAutoChooser.AddRoutine("Outpost Trench Sweep",
                                 [this] { return m_mayhemAutos->BottomTrenchPart1Only(); });
  m_mayhemAutoChooser.AddRoutine("Outpost Trench Sweep x2",
                                 [this] { return m_mayhemAutos->BottomTrenchPart1And2(); });
  // Top trench options.
  m_mayhemAutoChooser.AddRoutine("Depot Trench Sweep",
                                 [this] { return m_mayhemAutos->TopTrenchPart1Only(); });
  m_mayhemAutoChooser.AddRoutine("Depot Trench Sweep x2",
                                 [this] { return m_mayhemAutos->TopTrenchPart1And2(); });

  frc::SmartDashboard::PutData("Mayhem Auto Chooser", &m_mayhemAutoChooser);

  // Schedule the selected Mayhem routine during autonomous.
  frc2::RobotModeTriggers::Autonomous().WhileTrue(m_mayhemAutoChooser.SelectedCommandScheduler());
}

void RobotContainer::ConfigureOperatorBinds() {
  m_driverController.A().OnTrue(m_superstructure.SetState(SS::SHOOT));
  m_driverController.X().OnTrue(m_superstructure.SetState(SS::INTAKE));
  m_driverController.Y().OnTrue(m_superstructure.SetState(SS::JITTER));
  m_driverController.B().OnTrue(m_superstructure.SetState(SS::HOME));
  m_driverController.LeftTrigger().OnTrue(m_superstructure.SetState(SS::OUTTAKE));
}

double RobotContainer::VxSupplier() {
  return SquareInput(m_driverController.GetHID().GetLeftY()) * MaxSpeed;
}

double RobotContainer::VySupplier() {
  return SquareInput(m_driverController.GetHID().GetLeftX()) * MaxSpeed;
}

ctre::phoenix6::swerve::requests::FieldCentric& RobotContainer::DriveRequest(double maxSpeed,
                                                                             double maxAngular) {
  auto& hid = m_driverController.GetHID();
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

  // Default drive with trench assist.
  m_drivetrain.SetDefaultCommand(
      m_drivetrain.ApplyRequest([this]() -> ctre::phoenix6::swerve::requests::FieldCentric& {
        return DriveRequest(MaxSpeed, MaxAngularRate);
      }));

  // Slow drive with trench assist.
  m_driverController.LeftBumper().WhileTrue(
      m_drivetrain.ApplyRequest([this]() -> ctre::phoenix6::swerve::requests::FieldCentric& {
        return DriveRequest(SlowMaxSpeed, SlowMaxAngular);
      }));

  // Super slow drive with trench assist.
  m_driverController.RightBumper().WhileTrue(
      m_drivetrain.ApplyRequest([this]() -> ctre::phoenix6::swerve::requests::FieldCentric& {
        return DriveRequest(SuperSlowMaxSpeed, SlowMaxAngular);
      }));

  m_driverController.POVUp().OnTrue(m_superstructure.AddShootMultiplier(.02));
  m_driverController.POVDown().OnTrue(m_superstructure.AddShootMultiplier(-0.02));

  m_driverController.POVRight().OnTrue(
      m_drivetrain.RunOnce([this] { m_drivetrain.SeedFieldCentric(); }));

  m_drivetrain.RegisterTelemetry(
      [this](const subsystems::CommandSwerveDrivetrain::SwerveDriveState& state) {
        m_logger.Telemeterize(state);
      });

  m_driverController.A().WhileTrue(m_drivetrain.RotateToLookahead(
      m_shotCalculator, [this] { return VxSupplier(); }, [this] { return VySupplier(); }));
}

RobotContainer::~RobotContainer() {
  // Before any member goes away: the drivetrain holds references to m_drive and the auto
  // factory's request (see CommandSwerveDrivetrain::Shutdown).
  m_drivetrain.Shutdown();
}

void RobotContainer::BeginLoop() { m_drivetrain.InvalidateStateCache(); }

void RobotContainer::Periodic() {
  m_field2d.SetRobotPose(m_drivetrain.State().Pose);
  m_superstructure.Periodic();
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
  const bool red = m_mayhemAutoFactory->IsRed();
  if (!m_autoPreviewShown || selected != m_autoPreviewName || red != m_autoPreviewRed) {
    m_autoPreviewShown = true;
    m_autoPreviewName = selected;
    m_autoPreviewRed = red;
    m_autoStartPose.reset();
    std::vector<frc::Pose2d> path;
    if (auto routine = m_mayhemAutoChooser.SelectedRoutine()) {
      constexpr double kSampleSpacing = 0.1;  // seconds of path between drawn poses
      for (const auto& trajectory : routine->Trajectories()) {
        const auto traj = trajectory->GetResolvedTrajectory();
        if (!traj) {
          continue;
        }
        if (!m_autoStartPose) {
          m_autoStartPose = traj->InitialPose();
        }
        for (double t = 0; t < traj->TotalTime(); t += kSampleSpacing) {
          path.push_back(traj->SampleAt(t).GetPose());
        }
        path.push_back(traj->FinalPose());
      }
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
    m_autoAtStartLog.Set(distance <= kAutoStartToleranceMeters &&
                         heading <= kAutoStartToleranceDegrees);
  }
}
