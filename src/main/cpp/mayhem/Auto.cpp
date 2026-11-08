// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "mayhem/Auto.h"

#include <algorithm>
#include <atomic>
#include <stdexcept>

#include <frc/DriverStation.h>
#include <frc/Timer.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/Commands.h>
#include <frc2/command/ProxyCommand.h>
#include <networktables/NetworkTableInstance.h>
#include <wpi/sendable/SendableBuilder.h>
#include <wpi/sendable/SendableRegistry.h>

#include "utils/DSReport.h"

namespace mayhem {

namespace {
double Now() { return frc::Timer::GetFPGATimestamp().value(); }
}  // namespace

// ------------------------------------------------------------------------------------ telemetry
MayhemTelemetry::MayhemTelemetry(std::string_view root) {
  auto t = nt::NetworkTableInstance::GetDefault().GetTable(root);
  m_reference = t->GetStructTopic<frc::Pose2d>("reference").Publish();
  m_path = t->GetStructArrayTopic<frc::Pose2d>("trajectory").Publish();
  m_bridge = t->GetStructArrayTopic<frc::Pose2d>("bridge").Publish();
  m_state = t->GetStringTopic("state").Publish();
  m_name = t->GetStringTopic("name").Publish();
  m_time = t->GetDoubleTopic("time").Publish();
  m_rate = t->GetDoubleTopic("clockRate").Publish();
  m_error = t->GetDoubleTopic("positionError").Publish();
  m_planMs = t->GetDoubleTopic("lastPlanMs").Publish();
  m_bridges = t->GetDoubleTopic("bridgesPlanned").Publish();
  m_terrain = t->GetBooleanTopic("onRoughTerrain").Publish();
  m_unbeaching = t->GetBooleanTopic("unbeaching").Publish();
  m_tilt = t->GetDoubleTopic("tiltDegrees").Publish();
  m_unbeaches = t->GetDoubleTopic("unbeachesStarted").Publish();
  m_unbeachTarget = t->GetStructArrayTopic<frc::Pose2d>("unbeachTarget").Publish();
}

void MayhemTelemetry::StartTrajectory(const MayhemTrajectory& traj) {
  const auto& s = traj.Samples();
  const size_t step = std::max<size_t>(1, s.size() / 150);
  std::vector<frc::Pose2d> poses;
  poses.reserve((s.size() + step - 1) / step);
  for (size_t i = 0; i < s.size(); i += step) {
    poses.push_back(s[i].GetPose());
  }
  m_path.Set(poses);
  m_name.Set(traj.Name());
}

void MayhemTelemetry::Update(const TrajectoryRunner& r) {
  m_reference.Set(r.Reference().GetPose());
  m_state.Set(TrajectoryRunner::StateName(r.GetState()));
  m_time.Set(r.TrajectoryTime());
  m_rate.Set(r.ClockRate());
  m_error.Set(r.PositionError());
  m_planMs.Set(r.LastPlanSeconds() * 1000);
  m_bridges.Set(r.BridgesPlanned());
  m_terrain.Set(r.OnRoughTerrain());
  m_unbeaching.Set(r.IsUnbeaching());
  m_tilt.Set(r.TiltDegrees());
  m_unbeaches.Set(r.UnbeachCount());
  if (auto t = r.UnbeachTarget()) {
    m_unbeachTarget.Set(std::vector<frc::Pose2d>{frc::Pose2d{*t, frc::Rotation2d{}}});
  } else {
    m_unbeachTarget.Set(std::span<const frc::Pose2d>{});
  }
  if (r.GetBridge()) {
    m_bridge.Set(r.GetBridge()->Poses(30));
  } else {
    m_bridge.Set(std::span<const frc::Pose2d>{});
  }
}

void MayhemTelemetry::Stop() {
  m_state.Set("IDLE");
  m_terrain.Set(false);
  m_unbeaching.Set(false);
  m_unbeachTarget.Set(std::span<const frc::Pose2d>{});
  m_bridge.Set(std::span<const frc::Pose2d>{});
}

// -------------------------------------------------------------------------------------- factory
AutoFactory::AutoFactory(std::function<frc::Pose2d()> poseSupplier,
                         std::function<void(const frc::Pose2d&)> resetOdometry,
                         std::function<frc::ChassisSpeeds()> fieldSpeedsSupplier,
                         std::function<void(const DriveCommand&)> controller,
                         bool useAllianceFlipping, frc2::Subsystem* driveSubsystem)
    : m_pose{std::move(poseSupplier)},
      m_resetOdometry{std::move(resetOdometry)},
      m_fieldSpeeds{std::move(fieldSpeedsSupplier)},
      m_output{std::move(controller)},
      m_driveSubsystem{driveSubsystem} {
  if (useAllianceFlipping) {
    m_useRedAlliance = [] {
      return frc::DriverStation::GetAlliance().value_or(frc::DriverStation::Alliance::kBlue) ==
             frc::DriverStation::Alliance::kRed;
    };
  } else {
    m_useRedAlliance = [] { return false; };
  }
}

std::shared_ptr<AutoRoutine> AutoFactory::NewRoutine(std::string name) {
  return std::make_shared<AutoRoutine>(*this, std::move(name));
}

AutoFactory& AutoFactory::Bind(std::string name, std::function<frc2::CommandPtr()> cmdFactory) {
  m_bindings[std::move(name)] = std::move(cmdFactory);
  return *this;
}
AutoFactory& AutoFactory::WithFollowerConfig(FollowerConfig cfg) {
  m_followerConfig = cfg;
  return *this;
}
AutoFactory& AutoFactory::WithRecoveryConfig(RecoveryConfig cfg) {
  m_recoveryConfig = cfg;
  return *this;
}
AutoFactory& AutoFactory::WithAccelerometer(std::function<double()> accelG) {
  m_accelG = std::move(accelG);
  return *this;
}
AutoFactory& AutoFactory::WithTiltSensor(std::function<double()> pitchDegrees,
                                         std::function<double()> rollDegrees) {
  m_pitchDegrees = std::move(pitchDegrees);
  m_rollDegrees = std::move(rollDegrees);
  return *this;
}
AutoFactory& AutoFactory::WithUnbeach(std::optional<UnbeachConfig> cfg) {
  m_unbeachConfig = cfg;
  return *this;
}
AutoFactory& AutoFactory::WithVisionBoost(std::function<void(bool)> visionBoost) {
  m_visionBoost = std::move(visionBoost);
  return *this;
}
AutoFactory& AutoFactory::WithAllianceFlip(std::function<bool()> useRedAlliance) {
  m_useRedAlliance = std::move(useRedAlliance);
  return *this;
}
AutoFactory& AutoFactory::WithTelemetry(bool enabled) {
  m_telemetry = enabled ? std::make_unique<MayhemTelemetry>("/Mayhem") : nullptr;
  return *this;
}

TrajectoryPtr AutoFactory::LoadTrajectory(const std::string& name, int splitIndex) {
  TrajectoryPtr t = trajectory_loader::Load(name);
  return splitIndex < 0 ? t : t->Segment(splitIndex);
}

frc2::Command* AutoFactory::BoundCommand(const std::string& name) {
  auto it = m_bindings.find(name);
  if (it == m_bindings.end()) {
    return nullptr;
  }
  m_boundCommands.push_back(it->second().Unwrap());
  return m_boundCommands.back().get();
}

// -------------------------------------------------------------------------------------- routine
AutoRoutine::AutoRoutine(AutoFactory& factory, std::string name)
    : m_factory{factory}, m_name{std::move(name)} {}

AutoRoutine::~AutoRoutine() {
  for (auto& t : m_trajectories) {
    t->Reset();
  }
  // Drop the bindings (and the commands they own) before the trajectories they point at.
  m_loop.Clear();
}

AutoTrajectory& AutoRoutine::Trajectory(const std::string& trajectoryName, int splitIndex) {
  TrajectoryPtr loaded;
  try {
    loaded = AutoFactory::LoadTrajectory(trajectoryName, splitIndex);
  } catch (const std::exception& e) {
    const std::string label =
        splitIndex < 0 ? trajectoryName : trajectoryName + "[" + std::to_string(splitIndex) + "]";
    ds_report::Warning("MayhemLib: could not load trajectory '" + label + "': " + e.what());
    m_trajectories.push_back(std::make_unique<AutoTrajectory>(m_factory, *this, label));
    return *m_trajectories.back();
  }
  return Trajectory(std::move(loaded));
}

AutoTrajectory& AutoRoutine::Trajectory(TrajectoryPtr trajectory) {
  m_trajectories.push_back(
      std::make_unique<AutoTrajectory>(m_factory, *this, std::move(trajectory)));
  return *m_trajectories.back();
}

frc2::Trigger AutoRoutine::Active() {
  return Observe([this] { return m_isActive; });
}

frc2::Trigger AutoRoutine::Observe(std::function<bool()> condition) {
  return frc2::Trigger(&m_loop, std::move(condition));
}

frc2::Trigger AutoRoutine::Idle() {
  return Observe([this] {
    return std::none_of(m_trajectories.begin(), m_trajectories.end(),
                        [](const auto& t) { return t->IsActive(); });
  });
}

void AutoRoutine::Poll() {
  if (m_isKilled) {
    m_isActive = false;
    return;
  }
  m_isActive = true;
  m_pollCount++;
  m_loop.Poll();
}

void AutoRoutine::Reset() {
  for (auto& t : m_trajectories) {
    t->Reset();
  }
  if (m_isActive) {
    // Poll once while inactive so triggers see the falling edge; otherwise Active().OnTrue
    // would not fire again the next time this routine runs.
    m_isActive = false;
    m_loop.Poll();
  }
}

void AutoRoutine::Kill() {
  m_isKilled = true;
  Reset();
}

frc2::CommandPtr AutoRoutine::Cmd() {
  return Cmd([] { return false; });
}

frc2::CommandPtr AutoRoutine::Cmd(std::function<bool()> finishCondition) {
  // The command holds the routine alive.
  std::shared_ptr<AutoRoutine> self = shared_from_this();
  return frc2::cmd::Run([self] { self->Poll(); })
      .Until([self, finishCondition = std::move(finishCondition)] {
        return self->m_isKilled || finishCondition();
      })
      .FinallyDo([self] { self->Reset(); })
      .WithName(m_name);
}

// ----------------------------------------------------------------------------------- trajectory
class AutoTrajectory::FollowCommand : public frc2::CommandHelper<frc2::Command, FollowCommand> {
 public:
  explicit FollowCommand(AutoTrajectory* t) : m_t{t} {}

  void Initialize() override {
    AutoTrajectory& t = *m_t;
    AutoFactory& f = t.m_f;
    t.m_running = t.Resolved();
    TrajectoryRunner::EventListener listener;
    listener.onEvent = [tp = m_t](const TrajectoryEvent& e) { tp->OnEvent(e); };
    listener.onZoneEnd = [tp = m_t](const TrajectoryEvent& e) { tp->OnZoneEnd(e); };
    t.m_runner = std::make_unique<TrajectoryRunner>(t.m_running, f.m_followerConfig,
                                                    f.m_recoveryConfig, std::move(listener));
    t.m_runner->WithUnbeach(f.m_unbeachConfig);
    t.m_activeCmd = this;
    t.m_isActive = true;
    t.m_time = 0;
    t.m_routine.OnTrajectoryStart();
    t.m_runner->Start(Now());
    if (f.m_telemetry) {
      f.m_telemetry->StartTrajectory(*t.m_running);
    }
  }

  void Execute() override {
    AutoTrajectory& t = *m_t;
    AutoFactory& f = t.m_f;
    const double now = Now();
    const DriveCommand dc = t.m_runner->Update(now, f.m_pose(), f.m_fieldSpeeds(), f.m_accelG(),
                                               f.m_pitchDegrees(), f.m_rollDegrees());
    f.m_output(dc);
    const double prev = t.m_time;
    t.m_time = t.m_runner->TrajectoryTime();
    for (auto& [time, pulse] : t.m_timePulses) {
      if (prev < time && t.m_time >= time) {
        t.Fire(*pulse);
      }
    }
    t.m_recovering = t.m_runner->IsRecovering();
    t.m_unbeaching = t.m_runner->IsUnbeaching();
    t.m_onTerrain = t.m_runner->OnRoughTerrain();
    t.m_intakeOut = t.m_runner->IntakeExtended();
    f.m_visionBoost(t.m_runner->VisionBoostActive());
    if (f.m_telemetry) {
      f.m_telemetry->Update(*t.m_runner);
    }
  }

  bool IsFinished() override { return m_t->m_runner != nullptr && m_t->m_runner->IsFinished(); }

  void End(bool interrupted) override {
    AutoTrajectory& t = *m_t;
    AutoFactory& f = t.m_f;
    if (t.m_runner) {
      t.m_runner->Stop();
      const bool endsMoving = t.m_runner->Trajectory().FinalSample().Speed() > 0.05;
      if (interrupted || !endsMoving) {
        f.m_output(DriveCommand::Stop(f.m_followerConfig.modules));
      }
    }
    for (auto& [name, z] : t.m_runningZones) {
      frc2::CommandScheduler::GetInstance().Cancel(z);
    }
    t.m_runningZones.clear();
    t.m_zoneActive.clear();
    t.m_isActive = false;
    t.m_recovering = false;
    t.m_unbeaching = false;
    t.m_onTerrain = false;
    t.m_intakeOut = false;
    t.m_activeCmd = nullptr;
    f.m_visionBoost(false);
    if (!interrupted) {
      t.m_hasFinished = true;
      t.m_finishedAt = Now();
      t.m_startsAtFinish = t.m_routine.TrajectoryStarts();
      t.Fire(t.m_donePulse);
    }
    if (f.m_telemetry) {
      f.m_telemetry->Stop();
    }
  }

 private:
  AutoTrajectory* m_t;
};

AutoTrajectory::AutoTrajectory(AutoFactory& f, AutoRoutine& routine, TrajectoryPtr base)
    : m_f{f}, m_routine{routine}, m_base{std::move(base)}, m_name{m_base->Name()} {}

AutoTrajectory::AutoTrajectory(AutoFactory& f, AutoRoutine& routine, std::string missingName)
    : m_f{f}, m_routine{routine}, m_name{std::move(missingName)} {}

AutoTrajectory::~AutoTrajectory() = default;

std::optional<frc::Pose2d> AutoTrajectory::GetInitialPose() const {
  if (!m_base) {
    return std::nullopt;
  }
  return Resolved()->InitialPose();
}

std::optional<frc::Pose2d> AutoTrajectory::GetFinalPose() const {
  if (!m_base) {
    return std::nullopt;
  }
  return Resolved()->FinalPose();
}

frc2::CommandPtr AutoTrajectory::MissingCmd(const std::string& what) {
  return frc2::cmd::RunOnce([what, name = m_name] {
           ds_report::Warning("MayhemLib: skipping " + what + " of trajectory '" + name +
                              "', it failed to load");
         })
      .WithName("Mayhem missing " + m_name);
}

TrajectoryPtr AutoTrajectory::Resolved() const { return m_f.IsRed() ? m_base->Flipped() : m_base; }

frc2::CommandPtr AutoTrajectory::ResetOdometry() {
  if (!m_base) {
    return MissingCmd("odometry reset");
  }
  return frc2::cmd::RunOnce([this] { m_f.m_resetOdometry(Resolved()->InitialPose()); })
      .WithName("Mayhem reset " + m_name);
}

frc2::CommandPtr AutoTrajectory::Cmd() {
  if (!m_base) {
    return MissingCmd("follow");
  }
  FollowCommand c{this};
  c.AddRequirements(m_f.m_driveSubsystem);
  c.SetName("Mayhem " + m_name);
  return std::move(c).ToPtr();
}

void AutoTrajectory::OnEvent(const TrajectoryEvent& e) {
  Fire(MarkerPulse(e.name));
  if (e.IsZone()) {
    m_zoneActive[e.name] = true;
  }
  if (frc2::Command* cmd = m_f.BoundCommand(e.command)) {
    frc2::CommandScheduler::GetInstance().Schedule(cmd);
    if (e.IsZone()) {
      m_runningZones[e.name] = cmd;
    }
  }
}

void AutoTrajectory::OnZoneEnd(const TrajectoryEvent& e) {
  m_zoneActive.erase(e.name);
  auto it = m_runningZones.find(e.name);
  if (it != m_runningZones.end()) {
    frc2::CommandScheduler::GetInstance().Cancel(it->second);
    m_runningZones.erase(it);
  }
}

AutoTrajectory::Pulse& AutoTrajectory::MarkerPulse(const std::string& name) {
  auto& p = m_markerPulses[name];
  if (!p) {
    p = std::make_unique<Pulse>();
  }
  return *p;
}

frc2::Trigger AutoTrajectory::Active() {
  return m_routine.Observe([this] { return m_isActive; });
}

frc2::Trigger AutoTrajectory::Inactive() {
  return m_routine.Observe([this] { return !m_isActive; });
}

frc2::Trigger AutoTrajectory::Done(int cyclesToDelay) {
  return m_routine.Observe([this, cyclesToDelay] { return High(m_donePulse, cyclesToDelay); });
}

frc2::Trigger AutoTrajectory::DoneFor(double seconds) {
  return m_routine.Observe(
      [this, seconds] { return m_hasFinished && !m_isActive && Now() - m_finishedAt <= seconds; });
}

frc2::Trigger AutoTrajectory::RecentlyDone() {
  return m_routine.Observe(
      [this] { return m_hasFinished && m_routine.TrajectoryStarts() == m_startsAtFinish; });
}

frc2::Trigger AutoTrajectory::AtTime(double timeSinceStart) {
  auto& p = m_timePulses[timeSinceStart];
  if (!p) {
    p = std::make_unique<Pulse>();
  }
  Pulse* pulse = p.get();
  return m_routine.Observe([this, pulse] { return High(*pulse, 0); });
}

frc2::Trigger AutoTrajectory::AtTime(const std::string& eventName) {
  Pulse* pulse = &MarkerPulse(eventName);
  return m_routine.Observe([this, pulse, eventName] {
    if (High(*pulse, 0)) {
      return true;
    }
    auto it = m_zoneActive.find(eventName);
    return it != m_zoneActive.end() && it->second;
  });
}

frc2::Trigger AutoTrajectory::Recovering() {
  return m_routine.Observe([this] { return m_recovering; });
}
frc2::Trigger AutoTrajectory::Unbeaching() {
  return m_routine.Observe([this] { return m_unbeaching; });
}
frc2::Trigger AutoTrajectory::OnRoughTerrain() {
  return m_routine.Observe([this] { return m_onTerrain; });
}
frc2::Trigger AutoTrajectory::IntakeExtended() {
  return m_routine.Observe([this] { return m_intakeOut; });
}

void AutoTrajectory::Reset() {
  if (m_activeCmd != nullptr) {
    frc2::CommandScheduler::GetInstance().Cancel(m_activeCmd);
  }
  m_hasFinished = false;
  m_startsAtFinish = -1;
  m_finishedAt = -std::numeric_limits<double>::infinity();
}

// -------------------------------------------------------------------------------------- chooser
namespace {
std::atomic<int> g_instances{0};
}

AutoChooser::AutoChooser() : m_instance{g_instances.fetch_add(1)} {
  m_generators[kNoneName] = [] { return frc2::cmd::None().WithName(kNoneName); };
  m_order.push_back(kNoneName);
  m_built.emplace(frc2::cmd::None().WithName(kNoneName));
  wpi::SendableRegistry::Add(this, "AutoChooser", m_instance);
}

AutoChooser& AutoChooser::AddRoutine(const std::string& name,
                                     std::function<std::shared_ptr<AutoRoutine>()> generator) {
  return AddCmd(name, [generator = std::move(generator)] { return generator()->Cmd(); });
}

AutoChooser& AutoChooser::AddCmd(const std::string& name,
                                 std::function<frc2::CommandPtr()> generator) {
  if (!m_generators.contains(name)) {
    m_order.push_back(name);
  }
  m_generators[name] = std::move(generator);
  return *this;
}

void AutoChooser::Select(const std::string& name) {
  if (!m_generators.contains(name)) {
    ds_report::Warning("[MayhemLib] AutoChooser has no option named '" + name + "'");
    return;
  }
  m_selected = name;
  if (frc::DriverStation::IsDisabled()) {
    Build();
  }
}

frc2::Command* AutoChooser::SelectedCommand() {
  Build();
  return m_built->get();
}

frc2::CommandPtr AutoChooser::SelectedCommandScheduler() {
  return frc2::cmd::Defer([this] { return frc2::ProxyCommand(SelectedCommand()).ToPtr(); }, {})
      .WithName("AutoChooser");
}

void AutoChooser::Build() {
  if (m_selected != m_builtFor) {
    m_built.emplace(m_generators[m_selected]());
    m_builtFor = m_selected;
  }
}

void AutoChooser::InitSendable(wpi::SendableBuilder& builder) {
  builder.SetSmartDashboardType("String Chooser");
  builder.PublishConstInteger(".instance", m_instance);
  builder.AddStringProperty("default", [] { return std::string{kNoneName}; }, nullptr);
  builder.AddStringArrayProperty("options", [this] { return m_order; }, nullptr);
  builder.AddStringProperty("active", [this] { return m_selected; }, nullptr);
  builder.AddStringProperty("selected", nullptr,
                            [this](std::string_view val) { Select(std::string{val}); });
}

}  // namespace mayhem
