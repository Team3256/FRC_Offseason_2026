#include "commands/MayhemAutos.h"
#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include <algorithm>
#include <cmath>
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
  std::vector<frc::Pose2d> path{ToWpi(Resolve(def.startPose, red))};
  for (std::size_t i = 0; i < def.routine.nodeCount; ++i) {
    const auto& n = def.routine.nodes[i];
    if (n.kind != mayhem::NodeKind::Pose)
      continue;
    const auto p = ToWpi(Resolve(n.goal.pose, red));
    const auto from = path.back();
    const int count = std::max(
        1, static_cast<int>(std::ceil(from.Translation().Distance(p.Translation()).value() / .1)));
    for (int j = 1; j <= count; ++j) {
      const double f = double(j) / count;
      path.emplace_back(from.Translation() + (p.Translation() - from.Translation()) * f,
                        p.Rotation());
    }
  }
  return path;
}
namespace {
class SuiteRoutine final : public AutoRoutine, private mayhem::RobotHooks {
 public:
  SuiteRoutine(const mayhem::RoutineInfo& def, subsystems::Drivetrain& drive, Superstructure& ss,
               ShotCalculator& shots)
      : def_(def), drive_(drive), ss_(ss), shots_(shots) {
    // Enter/guard/transition and goal errors go to the log and NT under machine "Autonomous".
    runner_.Attach(observer_, mayhem_generated::autonomous::manifest,
                   [] { return frc::Timer::GetFPGATimestamp().value(); });
  }
  void Start() override {
    // Re-derived from the blue routine on every enable; measured odometry is never transformed.
    routine_.Apply(Transform(IsRed()));
    drive_.ResetPose(ToWpi(routine_.StartPose()));
    last_ = frc::Timer::GetFPGATimestamp().value();
    failureHandled_ = false;
    runner_.Start(Velocity());
    statusLog_.Set("Running");
    nameLog_.Set(def_.name);
  }
  void Periodic() override {
    const double now = frc::Timer::GetFPGATimestamp().value(), dt = now - last_;
    last_ = now;
    // Periodic() runs in the same loop as Start(). A zero dt is not a fault.
    if (dt == 0)
      return;
    dtLog_.Set(dt);
    // The runner clamps a slow loop (first enabled loop, log flush) instead of failing.
    const auto out = runner_.Tick(Pose(), Velocity(), dt);
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
  mayhem::SignalValue Signal(std::string_view key) const override {
    if (key == "superstructure.ready")
      return ss_.IsReadyToShoot();
    return {};
  }
  void BeginAction(std::string_view action) override {
    if (action == "drivetrain.alignAndShoot") {
      aligning_ = true;
      shotRequested_ = false;
      actionStart_ = frc::Timer::GetFPGATimestamp().value();
      return;
    }
    if (action == "superstructure.intake")
      ss_.StartIntaking();
    else if (action == "superstructure.stow")
      ss_.StowIntake();
  }
  mayhem::ActionStatus TickAction(std::string_view action) override {
    if (action == "drivetrain.alignAndShoot") {
      // The original 3.5 s aim-and-shoot window; the routine always continues afterwards.
      return frc::Timer::GetFPGATimestamp().value() - actionStart_ >= kShootWindowSeconds
                 ? mayhem::ActionStatus::Succeeded
                 : mayhem::ActionStatus::Running;
    }
    if (action == "superstructure.intake" || action == "superstructure.stow")
      return mayhem::ActionStatus::Succeeded;
    return mayhem::ActionStatus::Failed;
  }
  void EndAction(std::string_view action, bool /*interrupted*/) override {
    if (action == "drivetrain.alignAndShoot") {
      aligning_ = false;
      ss_.SetAutoShooting(false);
    }
  }
  void PathEvent(std::string_view action) override { BeginAction(action); }
  const mayhem::RoutineInfo& def_;
  subsystems::Drivetrain& drive_;
  Superstructure& ss_;
  ShotCalculator& shots_;
  mayhem::AllianceRoutine routine_{def_, field_constants::kFieldLength,
                                   field_constants::kFieldWidth};
  MayhemObserver observer_{"Autonomous"};  // before the runner, which reports to it
  mayhem::AutoRunner runner_{routine_.View(), static_cast<mayhem::RobotHooks&>(*this)};
  double last_ = 0, actionStart_ = 0;
  static constexpr double kShootWindowSeconds = 3.5;
  bool aligning_ = false, shotRequested_ = false, failureHandled_ = false;
  logging::Output<std::string> statusLog_{"Mayhem/Auto/Status"}, nameLog_{"Mayhem/Auto/Name"};
  logging::Output<double> dtLog_{"Mayhem/Auto/DeltaSeconds"};
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
}  // namespace suite_autos
