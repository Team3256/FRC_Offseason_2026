#include "commands/MayhemAutos.h"
#include <frc/Alert.h>
#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include <algorithm>
#include <ctre/phoenix6/swerve/SwerveRequest.hpp>
#include <numbers>
#include "FieldConstants.h"
#include "commands/MayhemObserver.h"
#include "mayhem/alliance.hpp"
#include "subsystems/Superstructure.h"
#include "subsystems/sotm/ShotCalculator.h"
#include "subsystems/swerve/Drivetrain.h"

namespace suite_autos {
bool IsRed() { return frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed; }
frc::Pose2d ToWpi(mayhem::Pose p) {
  return {units::meter_t{p.x}, units::meter_t{p.y}, frc::Rotation2d{units::radian_t{p.heading}}};
}
mayhem::FieldTransform Transform(bool red) {
  // REBUILT is rotationally symmetric: red is the blue routine rotated 180 degrees.
  return red ? mayhem::FieldTransform::Flip : mayhem::FieldTransform::None;
}
mayhem::Pose Resolve(mayhem::Pose p, bool red) {
  return mayhem::TransformPose(p, Transform(red), field_constants::kFieldLength,
                               field_constants::kFieldWidth);
}
const mayhem::RoutineInfo* Find(std::string_view name) {
  return mayhem::FindRoutine(mayhem_generated::routines, name);
}
std::vector<frc::Pose2d> Preview(const mayhem::RoutineInfo& def, bool red) {
  // The path Mayhem's editor drives (blends and corners included), about every 0.1 m.
  std::vector<frc::Pose2d> path;
  for (const auto& p : mayhem::PreviewPath(def, Transform(red), field_constants::kFieldLength,
                                           field_constants::kFieldWidth))
    path.push_back(ToWpi(p));
  return path;
}
namespace {
class SuiteRoutine final : public AutoRoutine {
 public:
  SuiteRoutine(const mayhem::RoutineInfo& def, subsystems::Drivetrain& drive, Superstructure& ss,
               ShotCalculator& shots)
      : def_(def), drive_(drive), ss_(ss), shots_(shots) {
    // Enter/guard/transition and goal errors go to the log and NT under machine "Autonomous". The
    // clock also times Tick().
    runner_.Attach(observer_, mayhem_generated::autonomous::manifest,
                   [] { return frc::Timer::GetFPGATimestamp().value(); });
    runner_.Bind("superstructure.intake", mayhem::Instant([this] { ss_.StartIntaking(); }))
        .Bind("superstructure.stow", mayhem::Instant([this] { ss_.StowIntake(); }))
        // Same as the original AlignAndShoot: turn to the lookahead shot and request SHOOT once
        // aimed (Periodic). The step's 3.5 s window (onTimeout "succeed") ends it; a missed shot
        // never aborts the routine.
        .Bind("drivetrain.alignAndShoot", {[this] {
                                             aligning_ = true;
                                             shotRequested_ = false;
                                           },
                                           [] { return mayhem::ActionStatus::Running; },
                                           [this](bool) {
                                             aligning_ = false;
                                             ss_.SetAutoShooting(false);
                                           }})
        .BindSignal("superstructure.ready",
                    [this] { return mayhem::SignalValue{ss_.IsReadyToShoot()}; });
    // Built while disabled, when the routine is selected: an action, event or signal in the project
    // that nothing here handles is an error before the match, not a failed step during it.
    std::string unbound = runner_.UnboundReport();
    std::replace(unbound.begin(), unbound.end(), '\n', ';');
    unboundAlert_.SetText("Auto \"" + std::string{def_.name} +
                          "\" has unhandled Mayhem IDs: " + unbound);
    unboundAlert_.Set(!unbound.empty());
  }
  std::string Unbound() const { return runner_.UnboundReport(); }
  void Start() override {
    // Re-derived from the blue routine on every enable; measured odometry is never transformed.
    routine_.Apply(Transform(IsRed()));
    drive_.ResetPose(ToWpi(routine_.StartPose()));
    failureHandled_ = false;
    runner_.Start(Velocity());
    statusLog_.Set("Running");
    nameLog_.Set(def_.name);
  }
  void Periodic() override {
    // dt from the runner's clock: zero in the loop that started it (the last command again), and a
    // slow loop (first enabled loop, log flush) is clamped instead of failing.
    const auto out = runner_.Tick(Pose(), Velocity());
    if (runner_.Status() == mayhem::RunStatus::Running && aligning_) {
      // Same as the original AlignAndShoot: the drivetrain's heading controller turns to the
      // lookahead shot, and SHOOT is requested once within 3 degrees.
      drive_.RotateToLookahead(shots_, 0.0, 0.0);
      if (!shotRequested_ &&
          ShotCalculator::IsAimed(drive_.State().Pose.Rotation(), shots_.GetDriveAngle(),
                                  3.0 * std::numbers::pi / 180.0)) {
        ss_.SetAutoShooting(true);
        shotRequested_ = true;
      }
      aimedLog_.Set(shotRequested_);
    } else {
      Output(out);
    }
    const auto status = runner_.Status();
    if (status == mayhem::RunStatus::Failed && !failureHandled_) {
      failureHandled_ = true;
      statusLog_.Set("Failed");
      ss_.StopAll();
    } else if (status == mayhem::RunStatus::Completed)
      statusLog_.Set("Completed");
    if (const auto* goal = runner_.ActiveGoal())
      targetLog_.Set(ToWpi(goal->pose));
    overrunLog_.Set(static_cast<double>(runner_.Overruns()));
    outputLog_.Set(frc::ChassisSpeeds{units::meters_per_second_t{out.vx},
                                      units::meters_per_second_t{out.vy},
                                      units::radians_per_second_t{out.omega}});
  }
  // Runs (holding the drivetrain stopped after completion or failure) until autonomous ends.
  void Stop() override {
    const bool running = runner_.Status() == mayhem::RunStatus::Running;
    runner_.Cancel();
    aligning_ = false;
    Output({});
    if (running)
      statusLog_.Set("Cancelled");
    ss_.StopAll();
  }

 private:
  mayhem::Pose Pose() const {
    const auto p = drive_.State().Pose;
    return {p.X().value(), p.Y().value(), p.Rotation().Radians().value()};
  }
  mayhem::Velocity Velocity() const {
    const auto v = drive_.GetFieldRelativeSpeeds();
    return {v.vx.value(), v.vy.value(), v.omega.value()};
  }
  void Output(mayhem::Velocity v) {
    drive_.ApplyMayhemSpeeds({units::meters_per_second_t{v.vx}, units::meters_per_second_t{v.vy},
                              units::radians_per_second_t{v.omega}});
  }
  const mayhem::RoutineInfo& def_;
  subsystems::Drivetrain& drive_;
  Superstructure& ss_;
  ShotCalculator& shots_;
  mayhem::AllianceRoutine routine_{def_, field_constants::kFieldLength,
                                   field_constants::kFieldWidth};
  MayhemObserver observer_{"Autonomous"};  // before the runner, which reports to it
  mayhem::AutoRunner runner_{routine_.View()};
  bool aligning_ = false, shotRequested_ = false, failureHandled_ = false;
  frc::Alert unboundAlert_{"", frc::Alert::AlertType::kError};
  logging::Output<std::string> statusLog_{"Mayhem/Auto/Status"}, nameLog_{"Mayhem/Auto/Name"};
  logging::Output<double> overrunLog_{"Mayhem/Auto/LoopOverruns"};
  logging::Output<bool> aimedLog_{"Mayhem/Auto/ShotAimAcquired"};
  logging::Output<frc::Pose2d> targetLog_{"Mayhem/Auto/Target"};
  logging::Output<frc::ChassisSpeeds> outputLog_{"Mayhem/Auto/FieldSpeeds"};
};
}  // namespace
std::unique_ptr<AutoRoutine> Routine(const mayhem::RoutineInfo& def, subsystems::Drivetrain& drive,
                                     Superstructure& ss, ShotCalculator& shots) {
  return std::make_unique<SuiteRoutine>(def, drive, ss, shots);
}
std::string UnboundReport(const mayhem::RoutineInfo& def, subsystems::Drivetrain& drive,
                          Superstructure& ss, ShotCalculator& shots) {
  return SuiteRoutine{def, drive, ss, shots}.Unbound();
}
}  // namespace suite_autos
