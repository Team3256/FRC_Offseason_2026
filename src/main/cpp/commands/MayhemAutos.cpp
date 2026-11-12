#include "commands/MayhemAutos.h"
#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <algorithm>
#include <cmath>
#include <ctre/phoenix6/swerve/SwerveRequest.hpp>
#include <numbers>
#include "FieldConstants.h"
#include "commands/MayhemObserver.h"
#include "subsystems/Superstructure.h"
#include "subsystems/sotm/ShotCalculator.h"
#include "subsystems/swerve/CommandSwerveDrivetrain.h"

namespace suite_autos {
bool IsRed() { return frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed; }
frc::Pose2d ToWpi(mayhem::Pose p) {
  return {units::meter_t{p.x}, units::meter_t{p.y}, frc::Rotation2d{units::radian_t{p.heading}}};
}
mayhem::Pose Resolve(mayhem::Pose p, bool red) {
  return red ? mayhem::FlipPose(p, field_constants::kFieldLength, field_constants::kFieldWidth) : p;
}
const robot_mayhem::AutoDefinition* Find(std::string_view name) {
  for (const auto& def : robot_mayhem::autos)
    if (def.name == name)
      return &def;
  return nullptr;
}
std::vector<frc::Pose2d> Preview(const robot_mayhem::AutoDefinition& def, bool red) {
  std::vector<frc::Pose2d> path{ToWpi(Resolve(def.start, red))};
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
class SuiteCommand final : public frc2::CommandHelper<frc2::Command, SuiteCommand>,
                           private mayhem::RobotHooks {
 public:
  SuiteCommand(const robot_mayhem::AutoDefinition& def, subsystems::CommandSwerveDrivetrain& drive,
               Superstructure& ss, ShotCalculator& shots)
      : def_(def),
        drive_(drive),
        ss_(ss),
        shots_(shots),
        nodes_(def.routine.nodes, def.routine.nodes + def.routine.nodeCount),
        markers_(def.routine.nodeCount) {
    AddRequirements({&drive_});
    SetName(std::string{def.name});
    observer_.Describe(robot_mayhem::AutonomousManifest);
    for (std::size_t i = 0; i < def.routine.transitionCount; ++i)
      observer_.RegisterGuard(std::string{def.routine.transitions[i].id}, "condition");
  }
  void Initialize() override {
    const bool red = IsRed();
    // Re-resolve from immutable BLUE definitions on each enable, including aim points
    // and rotation markers. Never transform measured blue-origin odometry.
    for (std::size_t i = 0; i < nodes_.size(); ++i) {
      nodes_[i] = def_.routine.nodes[i];
      auto& g = nodes_[i].goal;
      g.pose = Resolve(g.pose, red);
      if (red) {
        g.aimX = field_constants::kFieldLength - g.aimX;
        g.aimY = field_constants::kFieldWidth - g.aimY;
      }
      markers_[i].clear();
      for (std::size_t j = 0; j < g.markerCount; ++j) {
        auto marker = g.markers[j];
        if (red && marker.kind == mayhem::MarkerKind::Rotation)
          marker.heading = mayhem::Wrap(marker.heading + std::numbers::pi);
        markers_[i].push_back(marker);
      }
      g.markers = markers_[i].data();
    }
    auto routine = def_.routine;
    routine.nodes = nodes_.data();
    runner_ =
        std::make_unique<mayhem::AutoRunner>(routine, static_cast<mayhem::RobotHooks&>(*this));
    drive_.ResetPose(ToWpi(Resolve(def_.start, red)));
    last_ = frc::Timer::GetFPGATimestamp().value();
    failureHandled_ = false;
    runner_->Start(Velocity());
    statusLog_.Set("Running");
    nameLog_.Set(def_.name);
  }
  void Execute() override {
    if (!runner_)
      return;
    if (ss_.GetState() == Superstructure::StructureState::CANCEL_ALL) {
      runner_->Cancel();
      statusLog_.Set("Failed");
      Output({});
      return;
    }
    const double now = frc::Timer::GetFPGATimestamp().value(), dt = now - last_;
    last_ = now;
    // WPILib can execute in the same tick as Initialize. A zero dt is not a fault.
    if (dt == 0)
      return;
    dtLog_.Set(dt);
    const auto p = Pose();
    // A single slow loop (first enabled loop, log flush) must not fail the whole routine.
    const auto out = runner_->Tick(p, Velocity(), std::min(dt, kMaxDt));
    if (runner_->Status() == mayhem::RunStatus::Running && aligning_) {
      // Same as the original AlignAndShoot: the drivetrain's heading controller turns to the
      // lookahead shot, and SHOOT is requested once within 3 degrees.
      align_.get()->Execute();
      if (!shotRequested_ &&
          ShotCalculator::IsAimed(drive_.State().Pose.Rotation(), shots_.GetDriveAngle(),
                                  3.0 * std::numbers::pi / 180.0)) {
        ss_.RequestState(Superstructure::StructureState::SHOOT,
                         Superstructure::RequestSource::Autonomous);
        shotRequested_ = true;
      }
      aimedLog_.Set(shotRequested_);
    } else {
      Output(out);
    }
    const auto status = runner_->Status();
    if (status == mayhem::RunStatus::Failed && !failureHandled_) {
      failureHandled_ = true;
      statusLog_.Set("Failed");
      ss_.RequestState(Superstructure::StructureState::CANCEL_ALL,
                       Superstructure::RequestSource::Supervisor);
    } else if (status == mayhem::RunStatus::Completed)
      statusLog_.Set("Completed");
    for (const auto& node : nodes_)
      if (node.id == runner_->ActiveState() && node.kind == mayhem::NodeKind::Pose) {
        const auto& g = node.goal;
        double angle = g.pose.heading;
        if (g.headingMode == mayhem::HeadingMode::PointAt)
          angle = std::atan2(g.aimY - p.y, g.aimX - p.x);
        observer_.GoalProgress(node.id, std::hypot(g.pose.x - p.x, g.pose.y - p.y),
                               std::abs(mayhem::Wrap(angle - p.heading)), false, now);
        targetLog_.Set(ToWpi(g.pose));
        break;
      }
    outputLog_.Set(frc::ChassisSpeeds{units::meters_per_second_t{out.vx},
                                      units::meters_per_second_t{out.vy},
                                      units::radians_per_second_t{out.omega}});
  }
  // Keep the drivetrain stopped after completion/failure until auto is cancelled.
  // Otherwise the driver's default drive command could take control during auto.
  bool IsFinished() override { return false; }
  void End(bool interrupted) override {
    if (runner_)
      runner_->Cancel();
    if (aligning_)
      align_.get()->End(true);
    aligning_ = false;
    Output({});
    if (interrupted)
      statusLog_.Set("Cancelled");
    ss_.RequestState(Superstructure::StructureState::CANCEL_ALL,
                     Superstructure::RequestSource::Supervisor);
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
      align_.get()->Initialize();
      aligning_ = true;
      shotRequested_ = false;
      actionStart_ = frc::Timer::GetFPGATimestamp().value();
      return;
    }
    constexpr std::string_view prefix = "superstructure.setState.";
    if (action.starts_with(prefix))
      for (int i = 0; i < Superstructure::kNumStates; ++i) {
        const auto s = static_cast<Superstructure::StructureState>(i);
        if (action.substr(prefix.size()) == Superstructure::ToString(s)) {
          ss_.RequestState(s, Superstructure::RequestSource::Autonomous);
          return;
        }
      }
  }
  mayhem::ActionStatus TickAction(std::string_view action) override {
    if (action == "drivetrain.alignAndShoot") {
      // The original 3.5 s aim-and-shoot window; the routine always continues afterwards.
      return frc::Timer::GetFPGATimestamp().value() - actionStart_ >= kShootWindowSeconds
                 ? mayhem::ActionStatus::Succeeded
                 : mayhem::ActionStatus::Running;
    }
    constexpr std::string_view prefix = "superstructure.setState.";
    if (action.starts_with(prefix))
      for (int i = 0; i < Superstructure::kNumStates; ++i)
        if (action.substr(prefix.size()) ==
            Superstructure::ToString(static_cast<Superstructure::StructureState>(i)))
          return mayhem::ActionStatus::Succeeded;
    return mayhem::ActionStatus::Failed;
  }
  void EndAction(std::string_view action, bool interrupted) override {
    if (action == "drivetrain.alignAndShoot" && aligning_) {
      align_.get()->End(interrupted);
      aligning_ = false;
    }
  }
  void PathEvent(std::string_view action) override { BeginAction(action); }
  void StateEntered(std::string_view id) override {
    observer_.Enter("Autonomous", id, frc::Timer::GetFPGATimestamp().value());
  }
  void GuardEvaluated(std::string_view edge, bool accepted) override {
    observer_.Guard("Autonomous", edge, "condition", accepted,
                    frc::Timer::GetFPGATimestamp().value());
  }
  void Transitioned(std::string_view edge, std::string_view from, std::string_view to) override {
    const double now = frc::Timer::GetFPGATimestamp().value();
    observer_.Transition("Autonomous", edge, from, to, now);
    for (const auto& n : nodes_)
      if (n.id == from && n.kind == mayhem::NodeKind::Pose)
        observer_.GoalProgress(from, std::hypot(n.goal.pose.x - Pose().x, n.goal.pose.y - Pose().y),
                               std::abs(mayhem::Wrap(n.goal.pose.heading - Pose().heading)), true,
                               now);
  }
  const robot_mayhem::AutoDefinition& def_;
  subsystems::CommandSwerveDrivetrain& drive_;
  Superstructure& ss_;
  ShotCalculator& shots_;
  std::vector<mayhem::AutoNode> nodes_;
  std::vector<std::vector<mayhem::PathMarker>> markers_;
  std::unique_ptr<mayhem::AutoRunner> runner_;
  MayhemObserver observer_{"Autonomous"};
  double last_ = 0, actionStart_ = 0;
  static constexpr double kMaxDt = 0.1;
  static constexpr double kShootWindowSeconds = 3.5;
  frc2::CommandPtr align_ =
      drive_.RotateToLookahead(shots_, [] { return 0.0; }, [] { return 0.0; });
  bool aligning_ = false, shotRequested_ = false, failureHandled_ = false;
  logging::Output<std::string> statusLog_{"Mayhem/Auto/Status"}, nameLog_{"Mayhem/Auto/Name"};
  logging::Output<double> dtLog_{"Mayhem/Auto/DeltaSeconds"};
  logging::Output<bool> aimedLog_{"Mayhem/Auto/ShotAimAcquired"};
  logging::Output<frc::Pose2d> targetLog_{"Mayhem/Auto/Target"};
  logging::Output<frc::ChassisSpeeds> outputLog_{"Mayhem/Auto/FieldSpeeds"};
};
}  // namespace
frc2::CommandPtr Command(const robot_mayhem::AutoDefinition& def,
                         subsystems::CommandSwerveDrivetrain& drive, Superstructure& ss,
                         ShotCalculator& shots) {
  return frc2::CommandPtr{std::make_unique<SuiteCommand>(def, drive, ss, shots)};
}
}  // namespace suite_autos
