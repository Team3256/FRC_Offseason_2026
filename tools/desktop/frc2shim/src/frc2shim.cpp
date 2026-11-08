// Desktop-only shim of WPILib's frc2 command framework. See tools/desktop/frc2shim/README.md.
// Behaviour mirrors allwpilib's wpilibNewCommands (2026) for everything the robot code uses.

#include <algorithm>
#include <stdexcept>
#include <unordered_map>

#include <frc/RobotBase.h>
#include <frc/RobotState.h>
#include <frc/event/EventLoop.h>
#include <wpi/sendable/SendableBuilder.h>
#include <wpi/sendable/SendableRegistry.h>

#include "frc2/command/Command.h"
#include "frc2/command/CommandClasses.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/CommandScheduler.h"
#include "frc2/command/Commands.h"
#include "frc2/command/SubsystemBase.h"
#include "frc2/command/button/CommandGenericHID.h"
#include "frc2/command/button/Trigger.h"

namespace frc2 {

// ------------------------------------------------------------------------------------- scheduler
struct CommandScheduler::Impl {
  frc::EventLoop defaultButtonLoop;
  frc::EventLoop* activeButtonLoop = &defaultButtonLoop;
  // insertion ordered
  std::vector<Subsystem*> subsystems;
  std::unordered_map<const Subsystem*, std::unique_ptr<Command>> defaults;
  std::vector<Command*> scheduled;
  std::unordered_map<const Subsystem*, Command*> requirements;
  bool disabled = false;
  bool inRunLoop = false;
  std::vector<Command*> toSchedule;
  std::vector<Command*> toCancel;
  std::vector<Action> initActions, executeActions, interruptActions, finishActions;

  bool IsScheduled(const Command* c) const {
    return std::find(scheduled.begin(), scheduled.end(), c) != scheduled.end();
  }
  void Remove(Command* c) {
    scheduled.erase(std::remove(scheduled.begin(), scheduled.end(), c), scheduled.end());
    for (auto* r : c->GetRequirements()) {
      auto it = requirements.find(r);
      if (it != requirements.end() && it->second == c) {
        requirements.erase(it);
      }
    }
  }
};

CommandScheduler::CommandScheduler() : m_impl(new Impl) {
  // Like WPILib: registering here also guarantees the registry outlives the scheduler.
  wpi::SendableRegistry::Add(this, "Scheduler");
}
CommandScheduler::~CommandScheduler() = default;

CommandScheduler& CommandScheduler::GetInstance() {
  static CommandScheduler scheduler;
  return scheduler;
}

frc::EventLoop* CommandScheduler::GetActiveButtonLoop() const { return m_impl->activeButtonLoop; }
void CommandScheduler::SetActiveButtonLoop(frc::EventLoop* loop) {
  m_impl->activeButtonLoop = loop;
}
frc::EventLoop* CommandScheduler::GetDefaultButtonLoop() const {
  return &m_impl->defaultButtonLoop;
}

void CommandScheduler::Schedule(const CommandPtr& command) { Schedule(command.get()); }
void CommandScheduler::Schedule(CommandPtr&& command) {
  // The real scheduler keeps ownership; emulate by leaking into a holder.
  static std::vector<std::unique_ptr<Command>> owned;
  owned.emplace_back(std::move(command).Unwrap());
  Schedule(owned.back().get());
}
void CommandScheduler::Schedule(std::span<Command* const> commands) {
  for (auto* c : commands)
    Schedule(c);
}
void CommandScheduler::Schedule(std::initializer_list<Command*> commands) {
  for (auto* c : commands)
    Schedule(c);
}

void CommandScheduler::Schedule(Command* command) {
  if (command == nullptr) {
    return;
  }
  if (m_impl->inRunLoop) {
    m_impl->toSchedule.push_back(command);
    return;
  }
  if (command->IsComposed()) {
    throw std::runtime_error(
        "Commands that have been composed may not be added to another "
        "composition or scheduled individually!");
  }
  if (m_impl->disabled || m_impl->IsScheduled(command) ||
      (frc::RobotState::IsDisabled() && !command->RunsWhenDisabled())) {
    return;
  }
  const auto& reqs = command->GetRequirements();
  bool isDisjoint = true;
  bool allInterruptible = true;
  for (auto* r : reqs) {
    auto it = m_impl->requirements.find(r);
    if (it != m_impl->requirements.end()) {
      isDisjoint = false;
      allInterruptible &=
          it->second->GetInterruptionBehavior() == Command::InterruptionBehavior::kCancelSelf;
    }
  }
  if (!isDisjoint && !allInterruptible) {
    return;
  }
  if (!isDisjoint) {
    for (auto* r : reqs) {
      auto it = m_impl->requirements.find(r);
      if (it != m_impl->requirements.end()) {
        Cancel(it->second);
      }
    }
  }
  m_impl->scheduled.push_back(command);
  for (auto* r : reqs) {
    m_impl->requirements[r] = command;
  }
  command->Initialize();
  for (auto& a : m_impl->initActions)
    a(*command);
}

void CommandScheduler::Run() {
  if (m_impl->disabled) {
    return;
  }
  for (auto* s : m_impl->subsystems) {
    s->Periodic();
    if (frc::RobotBase::IsSimulation()) {
      s->SimulationPeriodic();
    }
  }
  frc::EventLoop* loop = m_impl->activeButtonLoop;
  loop->Poll();

  m_impl->inRunLoop = true;
  bool isDisabled = frc::RobotState::IsDisabled();
  // iterate over a copy; commands may be removed while iterating
  std::vector<Command*> current = m_impl->scheduled;
  for (Command* command : current) {
    if (!m_impl->IsScheduled(command)) {
      continue;
    }
    if (isDisabled && !command->RunsWhenDisabled()) {
      m_impl->inRunLoop = false;
      Cancel(command);
      m_impl->inRunLoop = true;
      continue;
    }
    command->Execute();
    for (auto& a : m_impl->executeActions)
      a(*command);
    if (command->IsFinished()) {
      m_impl->Remove(command);
      command->End(false);
      for (auto& a : m_impl->finishActions)
        a(*command);
    }
  }
  m_impl->inRunLoop = false;

  auto toSchedule = std::move(m_impl->toSchedule);
  m_impl->toSchedule.clear();
  for (auto* c : toSchedule)
    Schedule(c);
  auto toCancel = std::move(m_impl->toCancel);
  m_impl->toCancel.clear();
  for (auto* c : toCancel)
    Cancel(c);

  for (auto* s : m_impl->subsystems) {
    auto it = m_impl->defaults.find(s);
    if (it != m_impl->defaults.end() && !m_impl->requirements.contains(s)) {
      Schedule(it->second.get());
    }
  }
}

void CommandScheduler::RegisterSubsystem(Subsystem* subsystem) {
  if (std::find(m_impl->subsystems.begin(), m_impl->subsystems.end(), subsystem) ==
      m_impl->subsystems.end()) {
    m_impl->subsystems.push_back(subsystem);
  }
}
void CommandScheduler::UnregisterSubsystem(Subsystem* subsystem) {
  auto& v = m_impl->subsystems;
  v.erase(std::remove(v.begin(), v.end(), subsystem), v.end());
  m_impl->defaults.erase(subsystem);
}
void CommandScheduler::UnregisterAllSubsystems() {
  m_impl->subsystems.clear();
  m_impl->defaults.clear();
}

void CommandScheduler::SetDefaultCommand(Subsystem* subsystem, CommandPtr&& defaultCommand) {
  if (!defaultCommand.get()->HasRequirement(subsystem)) {
    throw std::runtime_error("Default commands must require their subsystem!");
  }
  auto owned = std::move(defaultCommand).Unwrap();
  owned->SetComposed(false);
  m_impl->defaults[subsystem] = std::move(owned);
}
void CommandScheduler::RemoveDefaultCommand(Subsystem* subsystem) {
  m_impl->defaults.erase(subsystem);
}
Command* CommandScheduler::GetDefaultCommand(const Subsystem* subsystem) const {
  auto it = m_impl->defaults.find(subsystem);
  return it == m_impl->defaults.end() ? nullptr : it->second.get();
}

void CommandScheduler::Cancel(Command* command) {
  if (command == nullptr) {
    return;
  }
  if (m_impl->inRunLoop) {
    m_impl->toCancel.push_back(command);
    return;
  }
  if (!m_impl->IsScheduled(command)) {
    return;
  }
  m_impl->Remove(command);
  command->End(true);
  for (auto& a : m_impl->interruptActions)
    a(*command);
}
void CommandScheduler::Cancel(const CommandPtr& command) { Cancel(command.get()); }
void CommandScheduler::Cancel(std::span<Command* const> commands) {
  for (auto* c : commands)
    Cancel(c);
}
void CommandScheduler::CancelAll() {
  auto copy = m_impl->scheduled;
  for (auto* c : copy)
    Cancel(c);
}

bool CommandScheduler::IsScheduled(const Command* command) const {
  return m_impl->IsScheduled(command);
}
bool CommandScheduler::IsScheduled(const CommandPtr& command) const {
  return IsScheduled(command.get());
}
Command* CommandScheduler::Requiring(const Subsystem* subsystem) const {
  auto it = m_impl->requirements.find(subsystem);
  return it == m_impl->requirements.end() ? nullptr : it->second;
}
void CommandScheduler::Disable() { m_impl->disabled = true; }
void CommandScheduler::Enable() { m_impl->disabled = false; }
void CommandScheduler::OnCommandInitialize(Action a) {
  m_impl->initActions.push_back(std::move(a));
}
void CommandScheduler::OnCommandExecute(Action a) {
  m_impl->executeActions.push_back(std::move(a));
}
void CommandScheduler::OnCommandInterrupt(Action a) {
  m_impl->interruptActions.push_back(std::move(a));
}
void CommandScheduler::OnCommandFinish(Action a) { m_impl->finishActions.push_back(std::move(a)); }
void CommandScheduler::RequireUngrouped(const Command* command) {
  if (command->IsComposed()) {
    throw std::runtime_error(
        "Commands that have been composed may not be added to another "
        "composition or scheduled individually!");
  }
}
void CommandScheduler::RequireUngroupedAndUnscheduled(const Command* command) {
  RequireUngrouped(command);
  if (IsScheduled(command)) {
    throw std::runtime_error(
        "Commands that have been scheduled individually may not be added to "
        "a composition!");
  }
}
void CommandScheduler::ResetForTesting() {
  CancelAll();
  m_impl->defaultButtonLoop.Clear();
  m_impl->activeButtonLoop = &m_impl->defaultButtonLoop;
  m_impl->subsystems.clear();
  m_impl->defaults.clear();
  m_impl->scheduled.clear();
  m_impl->requirements.clear();
  m_impl->toSchedule.clear();
  m_impl->toCancel.clear();
  m_impl->initActions.clear();
  m_impl->executeActions.clear();
  m_impl->interruptActions.clear();
  m_impl->finishActions.clear();
  m_impl->disabled = false;
}
void CommandScheduler::InitSendable(wpi::SendableBuilder& builder) {
  builder.SetSmartDashboardType("Scheduler");
}

// --------------------------------------------------------------------------------------- command
Command::Command() = default;
Command::~Command() { CommandScheduler::GetInstance().Cancel(this); }

void Command::AddRequirements(Requirements requirements) {
  for (auto* r : requirements)
    m_requirements.insert(r);
}
void Command::AddRequirements(wpi::SmallSet<Subsystem*, 4> requirements) {
  for (auto* r : requirements)
    m_requirements.insert(r);
}
void Command::AddRequirements(Subsystem* requirement) { m_requirements.insert(requirement); }

CommandPtr Command::WithTimeout(units::second_t d) && {
  return std::move(*this).ToPtr().WithTimeout(d);
}
CommandPtr Command::Until(std::function<bool()> c) && {
  return std::move(*this).ToPtr().Until(std::move(c));
}
CommandPtr Command::OnlyWhile(std::function<bool()> c) && {
  return std::move(*this).ToPtr().OnlyWhile(std::move(c));
}
CommandPtr Command::IgnoringDisable(bool b) && {
  return std::move(*this).ToPtr().IgnoringDisable(b);
}
CommandPtr Command::WithInterruptBehavior(InterruptionBehavior b) && {
  return std::move(*this).ToPtr().WithInterruptBehavior(b);
}
CommandPtr Command::AndThen(std::function<void()> toRun, Requirements r) && {
  return std::move(*this).ToPtr().AndThen(std::move(toRun), r);
}
CommandPtr Command::AndThen(CommandPtr&& next) && {
  return std::move(*this).ToPtr().AndThen(std::move(next));
}
CommandPtr Command::BeforeStarting(std::function<void()> toRun, Requirements r) && {
  return std::move(*this).ToPtr().BeforeStarting(std::move(toRun), r);
}
CommandPtr Command::BeforeStarting(CommandPtr&& b) && {
  return std::move(*this).ToPtr().BeforeStarting(std::move(b));
}
CommandPtr Command::Repeatedly() && { return std::move(*this).ToPtr().Repeatedly(); }
CommandPtr Command::AsProxy() && { return std::move(*this).ToPtr().AsProxy(); }
CommandPtr Command::Unless(std::function<bool()> c) && {
  return std::move(*this).ToPtr().Unless(std::move(c));
}
CommandPtr Command::OnlyIf(std::function<bool()> c) && {
  return std::move(*this).ToPtr().OnlyIf(std::move(c));
}
CommandPtr Command::DeadlineFor(CommandPtr&& p) && {
  return std::move(*this).ToPtr().DeadlineFor(std::move(p));
}
CommandPtr Command::AlongWith(CommandPtr&& p) && {
  return std::move(*this).ToPtr().AlongWith(std::move(p));
}
CommandPtr Command::RaceWith(CommandPtr&& p) && {
  return std::move(*this).ToPtr().RaceWith(std::move(p));
}
CommandPtr Command::FinallyDo(std::function<void(bool)> e) && {
  return std::move(*this).ToPtr().FinallyDo(std::move(e));
}
CommandPtr Command::FinallyDo(std::function<void()> e) && {
  return std::move(*this).ToPtr().FinallyDo(std::move(e));
}
CommandPtr Command::HandleInterrupt(std::function<void()> h) && {
  return std::move(*this).ToPtr().HandleInterrupt(std::move(h));
}
CommandPtr Command::WithName(std::string_view name) && {
  return std::move(*this).ToPtr().WithName(name);
}

void Command::Cancel() { CommandScheduler::GetInstance().Cancel(this); }
bool Command::IsScheduled() const { return CommandScheduler::GetInstance().IsScheduled(this); }
bool Command::HasRequirement(Subsystem* requirement) const {
  for (auto* r : GetRequirements()) {
    if (r == requirement)
      return true;
  }
  return false;
}
void Command::InitSendable(wpi::SendableBuilder& builder) {
  builder.SetSmartDashboardType("Command");
  builder.AddStringProperty(".name", [this] { return GetName(); }, nullptr);
}

bool RequirementsDisjoint(Command* first, Command* second) {
  for (auto* r : first->GetRequirements()) {
    if (second->HasRequirement(r))
      return false;
  }
  return true;
}

// ------------------------------------------------------------------------------------ commandptr
CommandPtr::CommandPtr(std::unique_ptr<Command>&& command) : m_ptr(std::move(command)) {
  AssertValid();
}
void CommandPtr::AssertValid() const {
  if (!m_ptr) {
    throw std::runtime_error("Moved-from CommandPtr object used!");
  }
}

namespace {
class ShimWrapper : public CommandHelper<WrapperCommand, ShimWrapper> {
 public:
  explicit ShimWrapper(std::unique_ptr<Command>&& c) : CommandHelper(std::move(c)) {}
  std::function<void(bool)> onEnd;
  std::optional<bool> runsWhenDisabled;
  std::optional<InterruptionBehavior> interruptBehavior;
  void End(bool interrupted) override {
    m_command->End(interrupted);
    if (onEnd)
      onEnd(interrupted);
  }
  bool RunsWhenDisabled() const override {
    return runsWhenDisabled ? *runsWhenDisabled : m_command->RunsWhenDisabled();
  }
  InterruptionBehavior GetInterruptionBehavior() const override {
    return interruptBehavior ? *interruptBehavior : m_command->GetInterruptionBehavior();
  }
};

std::vector<std::unique_ptr<Command>> Two(std::unique_ptr<Command>&& a,
                                          std::unique_ptr<Command>&& b) {
  std::vector<std::unique_ptr<Command>> v;
  v.emplace_back(std::move(a));
  v.emplace_back(std::move(b));
  return v;
}
}  // namespace

CommandPtr CommandPtr::Repeatedly() && {
  AssertValid();
  return CommandPtr(std::make_unique<RepeatCommand>(std::move(m_ptr)));
}
CommandPtr CommandPtr::AsProxy() && {
  AssertValid();
  return CommandPtr(std::make_unique<ProxyCommand>(std::move(m_ptr)));
}
CommandPtr CommandPtr::IgnoringDisable(bool doesRunWhenDisabled) && {
  AssertValid();
  auto w = std::make_unique<ShimWrapper>(std::move(m_ptr));
  w->runsWhenDisabled = doesRunWhenDisabled;
  return CommandPtr(std::move(w));
}
CommandPtr CommandPtr::WithInterruptBehavior(Command::InterruptionBehavior b) && {
  AssertValid();
  auto w = std::make_unique<ShimWrapper>(std::move(m_ptr));
  w->interruptBehavior = b;
  return CommandPtr(std::move(w));
}
CommandPtr CommandPtr::AndThen(std::function<void()> toRun, Requirements requirements) && {
  AssertValid();
  return std::move(*this).AndThen(cmd::RunOnce(std::move(toRun), requirements));
}
CommandPtr CommandPtr::AndThen(CommandPtr&& next) && {
  AssertValid();
  return CommandPtr(
      std::make_unique<SequentialCommandGroup>(Two(std::move(m_ptr), std::move(next).Unwrap())));
}
CommandPtr CommandPtr::BeforeStarting(std::function<void()> toRun, Requirements requirements) && {
  AssertValid();
  return std::move(*this).BeforeStarting(cmd::RunOnce(std::move(toRun), requirements));
}
CommandPtr CommandPtr::BeforeStarting(CommandPtr&& before) && {
  AssertValid();
  return CommandPtr(
      std::make_unique<SequentialCommandGroup>(Two(std::move(before).Unwrap(), std::move(m_ptr))));
}
CommandPtr CommandPtr::WithTimeout(units::second_t duration) && {
  AssertValid();
  return CommandPtr(std::make_unique<ParallelRaceGroup>(
      Two(std::move(m_ptr), std::make_unique<WaitCommand>(duration))));
}
CommandPtr CommandPtr::Until(std::function<bool()> condition) && {
  AssertValid();
  return CommandPtr(std::make_unique<ParallelRaceGroup>(
      Two(std::move(m_ptr), std::make_unique<WaitUntilCommand>(std::move(condition)))));
}
CommandPtr CommandPtr::OnlyWhile(std::function<bool()> condition) && {
  return std::move(*this).Until([c = std::move(condition)] { return !c(); });
}
CommandPtr CommandPtr::Unless(std::function<bool()> condition) && {
  AssertValid();
  return CommandPtr(std::make_unique<ConditionalCommand>(std::make_unique<InstantCommand>(),
                                                         std::move(m_ptr), std::move(condition)));
}
CommandPtr CommandPtr::OnlyIf(std::function<bool()> condition) && {
  return std::move(*this).Unless([c = std::move(condition)] { return !c(); });
}
CommandPtr CommandPtr::DeadlineFor(CommandPtr&& parallel) && {
  AssertValid();
  std::vector<std::unique_ptr<Command>> v;
  v.emplace_back(std::move(parallel).Unwrap());
  return CommandPtr(std::make_unique<ParallelDeadlineGroup>(std::move(m_ptr), std::move(v)));
}
CommandPtr CommandPtr::AlongWith(CommandPtr&& parallel) && {
  AssertValid();
  return CommandPtr(
      std::make_unique<ParallelCommandGroup>(Two(std::move(m_ptr), std::move(parallel).Unwrap())));
}
CommandPtr CommandPtr::RaceWith(CommandPtr&& parallel) && {
  AssertValid();
  return CommandPtr(
      std::make_unique<ParallelRaceGroup>(Two(std::move(m_ptr), std::move(parallel).Unwrap())));
}
CommandPtr CommandPtr::FinallyDo(std::function<void(bool)> end) && {
  AssertValid();
  auto w = std::make_unique<ShimWrapper>(std::move(m_ptr));
  w->onEnd = std::move(end);
  return CommandPtr(std::move(w));
}
CommandPtr CommandPtr::FinallyDo(std::function<void()> end) && {
  return std::move(*this).FinallyDo([e = std::move(end)](bool) { e(); });
}
CommandPtr CommandPtr::HandleInterrupt(std::function<void()> handler) && {
  return std::move(*this).FinallyDo([h = std::move(handler)](bool interrupted) {
    if (interrupted)
      h();
  });
}
CommandPtr CommandPtr::WithName(std::string_view name) && {
  AssertValid();
  auto w = std::make_unique<ShimWrapper>(std::move(m_ptr));
  w->SetName(name);
  return CommandPtr(std::move(w));
}
Command* CommandPtr::get() const& {
  AssertValid();
  return m_ptr.get();
}
std::unique_ptr<Command> CommandPtr::Unwrap() && {
  AssertValid();
  return std::move(m_ptr);
}
void CommandPtr::Schedule() const& { CommandScheduler::GetInstance().Schedule(*this); }
void CommandPtr::Cancel() const& { CommandScheduler::GetInstance().Cancel(*this); }
bool CommandPtr::IsScheduled() const& { return CommandScheduler::GetInstance().IsScheduled(*this); }
bool CommandPtr::HasRequirement(Subsystem* requirement) const& {
  return m_ptr->HasRequirement(requirement);
}
CommandPtr::operator bool() const& { return m_ptr.operator bool(); }
std::vector<std::unique_ptr<Command>> CommandPtr::UnwrapVector(std::vector<CommandPtr>&& vec) {
  std::vector<std::unique_ptr<Command>> ptrs;
  for (auto&& ptr : vec)
    ptrs.emplace_back(std::move(ptr).Unwrap());
  return ptrs;
}

// ---------------------------------------------------------------------------- concrete commands
FunctionalCommand::FunctionalCommand(std::function<void()> onInit, std::function<void()> onExecute,
                                     std::function<void(bool)> onEnd,
                                     std::function<bool()> isFinished, Requirements requirements)
    : m_onInit{std::move(onInit)},
      m_onExecute{std::move(onExecute)},
      m_onEnd{std::move(onEnd)},
      m_isFinished{std::move(isFinished)} {
  AddRequirements(requirements);
}
void FunctionalCommand::Initialize() { m_onInit(); }
void FunctionalCommand::Execute() { m_onExecute(); }
void FunctionalCommand::End(bool interrupted) { m_onEnd(interrupted); }
bool FunctionalCommand::IsFinished() { return m_isFinished(); }

InstantCommand::InstantCommand(std::function<void()> toRun, Requirements requirements)
    : CommandHelper(std::move(toRun), [] {}, [](bool) {}, [] { return true; }, requirements) {}
InstantCommand::InstantCommand() : InstantCommand([] {}) {}

RunCommand::RunCommand(std::function<void()> toRun, Requirements requirements)
    : CommandHelper([] {}, std::move(toRun), [](bool) {}, [] { return false; }, requirements) {}

StartEndCommand::StartEndCommand(std::function<void()> onInit, std::function<void()> onEnd,
                                 Requirements requirements)
    : CommandHelper(
          std::move(onInit), [] {}, [onEnd = std::move(onEnd)](bool) { onEnd(); },
          [] { return false; }, requirements) {}

WaitCommand::WaitCommand(units::second_t duration) : m_duration{duration} { SetName("Wait"); }
void WaitCommand::Initialize() { m_timer.Restart(); }
void WaitCommand::End(bool) { m_timer.Stop(); }
bool WaitCommand::IsFinished() { return m_timer.HasElapsed(m_duration); }

WaitUntilCommand::WaitUntilCommand(std::function<bool()> condition)
    : m_condition{std::move(condition)} {}
bool WaitUntilCommand::IsFinished() { return m_condition(); }

PrintCommand::PrintCommand(std::string_view message)
    : CommandHelper{[str = std::string(message)] { std::puts(str.c_str()); }, {}} {}

WrapperCommand::WrapperCommand(std::unique_ptr<Command>&& command) {
  CommandScheduler::GetInstance().RequireUngroupedAndUnscheduled(command.get());
  m_command = std::move(command);
  m_command->SetComposed(true);
  SetName(m_command->GetName());
}
void WrapperCommand::Initialize() { m_command->Initialize(); }
void WrapperCommand::Execute() { m_command->Execute(); }
bool WrapperCommand::IsFinished() { return m_command->IsFinished(); }
void WrapperCommand::End(bool interrupted) { m_command->End(interrupted); }
bool WrapperCommand::RunsWhenDisabled() const { return m_command->RunsWhenDisabled(); }
Command::InterruptionBehavior WrapperCommand::GetInterruptionBehavior() const {
  return m_command->GetInterruptionBehavior();
}
wpi::SmallSet<Subsystem*, 4> WrapperCommand::GetRequirements() const {
  return m_command->GetRequirements();
}

// sequential
SequentialCommandGroup::SequentialCommandGroup(std::vector<std::unique_ptr<Command>>&& commands) {
  AddCommands(std::move(commands));
}
void SequentialCommandGroup::AddCommands(std::vector<std::unique_ptr<Command>>&& commands) {
  for (auto&& command : commands) {
    CommandScheduler::GetInstance().RequireUngroupedAndUnscheduled(command.get());
    command->SetComposed(true);
    AddRequirements(command->GetRequirements());
    m_runWhenDisabled &= command->RunsWhenDisabled();
    if (command->GetInterruptionBehavior() == InterruptionBehavior::kCancelSelf) {
      m_interruptBehavior = InterruptionBehavior::kCancelSelf;
    }
    m_commands.emplace_back(std::move(command));
  }
}
void SequentialCommandGroup::Initialize() {
  m_currentCommandIndex = 0;
  if (!m_commands.empty()) {
    m_commands[0]->Initialize();
  }
}
void SequentialCommandGroup::Execute() {
  if (m_commands.empty()) {
    return;
  }
  auto& currentCommand = m_commands[m_currentCommandIndex];
  currentCommand->Execute();
  if (currentCommand->IsFinished()) {
    currentCommand->End(false);
    m_currentCommandIndex++;
    if (m_currentCommandIndex < m_commands.size()) {
      m_commands[m_currentCommandIndex]->Initialize();
    }
  }
}
void SequentialCommandGroup::End(bool interrupted) {
  if (interrupted && !m_commands.empty() && m_currentCommandIndex != static_cast<size_t>(-1) &&
      m_currentCommandIndex < m_commands.size()) {
    m_commands[m_currentCommandIndex]->End(interrupted);
  }
  m_currentCommandIndex = static_cast<size_t>(-1);
}
bool SequentialCommandGroup::IsFinished() { return m_currentCommandIndex == m_commands.size(); }

// parallel
ParallelCommandGroup::ParallelCommandGroup(std::vector<std::unique_ptr<Command>>&& commands) {
  AddCommands(std::move(commands));
}
void ParallelCommandGroup::AddCommands(std::vector<std::unique_ptr<Command>>&& commands) {
  for (auto&& command : commands) {
    CommandScheduler::GetInstance().RequireUngroupedAndUnscheduled(command.get());
    for (auto& [existing, running] : m_commands) {
      if (!RequirementsDisjoint(existing.get(), command.get())) {
        throw std::runtime_error(
            "Multiple commands in a parallel composition cannot require the "
            "same subsystems");
      }
    }
    command->SetComposed(true);
    AddRequirements(command->GetRequirements());
    m_runWhenDisabled &= command->RunsWhenDisabled();
    if (command->GetInterruptionBehavior() == InterruptionBehavior::kCancelSelf) {
      m_interruptBehavior = InterruptionBehavior::kCancelSelf;
    }
    m_commands.emplace_back(std::move(command), false);
  }
}
void ParallelCommandGroup::Initialize() {
  for (auto& [command, running] : m_commands) {
    command->Initialize();
    running = true;
  }
  isRunning = true;
}
void ParallelCommandGroup::Execute() {
  for (auto& [command, running] : m_commands) {
    if (!running)
      continue;
    command->Execute();
    if (command->IsFinished()) {
      command->End(false);
      running = false;
    }
  }
}
void ParallelCommandGroup::End(bool interrupted) {
  if (interrupted) {
    for (auto& [command, running] : m_commands) {
      if (running)
        command->End(true);
    }
  }
  isRunning = false;
}
bool ParallelCommandGroup::IsFinished() {
  for (auto& [command, running] : m_commands) {
    if (running)
      return false;
  }
  return true;
}

// race
ParallelRaceGroup::ParallelRaceGroup(std::vector<std::unique_ptr<Command>>&& commands) {
  AddCommands(std::move(commands));
}
void ParallelRaceGroup::AddCommands(std::vector<std::unique_ptr<Command>>&& commands) {
  for (auto&& command : commands) {
    CommandScheduler::GetInstance().RequireUngroupedAndUnscheduled(command.get());
    for (auto& existing : m_commands) {
      if (!RequirementsDisjoint(existing.get(), command.get())) {
        throw std::runtime_error(
            "Multiple commands in a parallel composition cannot require the "
            "same subsystems");
      }
    }
    command->SetComposed(true);
    AddRequirements(command->GetRequirements());
    m_runWhenDisabled &= command->RunsWhenDisabled();
    if (command->GetInterruptionBehavior() == InterruptionBehavior::kCancelSelf) {
      m_interruptBehavior = InterruptionBehavior::kCancelSelf;
    }
    m_commands.emplace_back(std::move(command));
  }
}
void ParallelRaceGroup::Initialize() {
  m_finished = false;
  for (auto& command : m_commands)
    command->Initialize();
  isRunning = true;
}
void ParallelRaceGroup::Execute() {
  for (auto& command : m_commands) {
    command->Execute();
    if (command->IsFinished())
      m_finished = true;
  }
}
void ParallelRaceGroup::End(bool interrupted) {
  for (auto& command : m_commands)
    command->End(!command->IsFinished());
  isRunning = false;
}
bool ParallelRaceGroup::IsFinished() { return m_finished; }

// deadline
ParallelDeadlineGroup::ParallelDeadlineGroup(std::unique_ptr<Command>&& deadline,
                                             std::vector<std::unique_ptr<Command>>&& commands) {
  m_deadline = deadline.get();
  std::vector<std::unique_ptr<Command>> all;
  all.emplace_back(std::move(deadline));
  for (auto&& c : commands)
    all.emplace_back(std::move(c));
  AddCommands(std::move(all));
}
void ParallelDeadlineGroup::AddCommands(std::vector<std::unique_ptr<Command>>&& commands) {
  for (auto&& command : commands) {
    CommandScheduler::GetInstance().RequireUngroupedAndUnscheduled(command.get());
    command->SetComposed(true);
    AddRequirements(command->GetRequirements());
    m_runWhenDisabled &= command->RunsWhenDisabled();
    if (command->GetInterruptionBehavior() == InterruptionBehavior::kCancelSelf) {
      m_interruptBehavior = InterruptionBehavior::kCancelSelf;
    }
    m_commands.emplace_back(std::move(command), false);
  }
}
void ParallelDeadlineGroup::Initialize() {
  for (auto& [command, running] : m_commands) {
    command->Initialize();
    running = true;
  }
  m_finished = false;
}
void ParallelDeadlineGroup::Execute() {
  for (auto& [command, running] : m_commands) {
    if (!running)
      continue;
    command->Execute();
    if (command->IsFinished()) {
      command->End(false);
      running = false;
      if (command.get() == m_deadline)
        m_finished = true;
    }
  }
}
void ParallelDeadlineGroup::End(bool interrupted) {
  for (auto& [command, running] : m_commands) {
    if (running)
      command->End(true);
  }
}
bool ParallelDeadlineGroup::IsFinished() { return m_finished; }

// repeat
RepeatCommand::RepeatCommand(std::unique_ptr<Command>&& command) {
  CommandScheduler::GetInstance().RequireUngroupedAndUnscheduled(command.get());
  m_command = std::move(command);
  m_command->SetComposed(true);
  AddRequirements(m_command->GetRequirements());
  SetName(std::string{"Repeat("} + m_command->GetName() + ")");
}
void RepeatCommand::Initialize() {
  m_ended = false;
  m_command->Initialize();
}
void RepeatCommand::Execute() {
  if (m_ended) {
    m_ended = false;
    m_command->Initialize();
  }
  m_command->Execute();
  if (m_command->IsFinished()) {
    m_command->End(false);
    m_ended = true;
  }
}
bool RepeatCommand::IsFinished() { return false; }
void RepeatCommand::End(bool interrupted) {
  if (!m_ended) {
    m_command->End(interrupted);
    m_ended = true;
  }
}
bool RepeatCommand::RunsWhenDisabled() const { return m_command->RunsWhenDisabled(); }
Command::InterruptionBehavior RepeatCommand::GetInterruptionBehavior() const {
  return m_command->GetInterruptionBehavior();
}

// proxy
ProxyCommand::ProxyCommand(wpi::unique_function<Command*()> supplier)
    : m_supplier(std::move(supplier)) {}
ProxyCommand::ProxyCommand(wpi::unique_function<CommandPtr()> supplier)
    : ProxyCommand(
          [supplier = std::move(supplier), holder = std::optional<CommandPtr>{}]() mutable {
            holder = supplier();
            return holder->get();
          }) {}
ProxyCommand::ProxyCommand(Command* command) : ProxyCommand([command] { return command; }) {
  SetName(std::string{"Proxy("} + command->GetName() + ")");
}
ProxyCommand::ProxyCommand(std::unique_ptr<Command> command) {
  m_owned = std::shared_ptr<Command>(std::move(command));
  SetName(std::string{"Proxy("} + m_owned->GetName() + ")");
  m_supplier = [c = m_owned.get()] { return c; };
}
void ProxyCommand::Initialize() {
  m_command = m_supplier();
  CommandScheduler::GetInstance().Schedule(m_command);
}
void ProxyCommand::End(bool interrupted) {
  if (interrupted && m_command) {
    m_command->Cancel();
  }
  m_command = nullptr;
}
bool ProxyCommand::IsFinished() { return m_command == nullptr || !m_command->IsScheduled(); }

// deferred
DeferredCommand::DeferredCommand(wpi::unique_function<CommandPtr()> supplier,
                                 Requirements requirements)
    : m_supplier{std::move(supplier)} {
  AddRequirements(requirements);
}
void DeferredCommand::Initialize() {
  m_command.emplace(m_supplier());
  CommandScheduler::GetInstance().RequireUngrouped(m_command->get());
  m_command->get()->SetComposed(true);
  m_command->get()->Initialize();
}
void DeferredCommand::Execute() { m_command->get()->Execute(); }
void DeferredCommand::End(bool interrupted) {
  m_command->get()->End(interrupted);
  m_command.reset();
}
bool DeferredCommand::IsFinished() { return m_command->get()->IsFinished(); }

// conditional
ConditionalCommand::ConditionalCommand(std::unique_ptr<Command>&& onTrue,
                                       std::unique_ptr<Command>&& onFalse,
                                       std::function<bool()> condition)
    : m_condition{std::move(condition)} {
  m_onTrue = std::move(onTrue);
  m_onFalse = std::move(onFalse);
  m_onTrue->SetComposed(true);
  m_onFalse->SetComposed(true);
  AddRequirements(m_onTrue->GetRequirements());
  AddRequirements(m_onFalse->GetRequirements());
}
void ConditionalCommand::Initialize() {
  m_selectedCommand = m_condition() ? m_onTrue.get() : m_onFalse.get();
  m_selectedCommand->Initialize();
}
void ConditionalCommand::Execute() { m_selectedCommand->Execute(); }
void ConditionalCommand::End(bool interrupted) { m_selectedCommand->End(interrupted); }
bool ConditionalCommand::IsFinished() { return m_selectedCommand->IsFinished(); }
bool ConditionalCommand::RunsWhenDisabled() const {
  return m_onTrue->RunsWhenDisabled() && m_onFalse->RunsWhenDisabled();
}

ScheduleCommand::ScheduleCommand(Command* toSchedule) : m_toSchedule{toSchedule} {}
void ScheduleCommand::Initialize() { CommandScheduler::GetInstance().Schedule(m_toSchedule); }

// ------------------------------------------------------------------------------------- factories
namespace cmd {
CommandPtr None() { return InstantCommand().ToPtr(); }
CommandPtr Idle(Requirements requirements) {
  return Run([] {}, requirements);
}
CommandPtr RunOnce(std::function<void()> action, Requirements requirements) {
  return InstantCommand(std::move(action), requirements).ToPtr();
}
CommandPtr Run(std::function<void()> action, Requirements requirements) {
  return RunCommand(std::move(action), requirements).ToPtr();
}
CommandPtr StartEnd(std::function<void()> start, std::function<void()> end,
                    Requirements requirements) {
  return StartEndCommand(std::move(start), std::move(end), requirements).ToPtr();
}
CommandPtr RunEnd(std::function<void()> run, std::function<void()> end, Requirements requirements) {
  return FunctionalCommand([] {}, std::move(run), [end = std::move(end)](bool) { end(); },
                           [] { return false; }, requirements)
      .ToPtr();
}
CommandPtr StartRun(std::function<void()> start, std::function<void()> run,
                    Requirements requirements) {
  return FunctionalCommand(
             std::move(start), std::move(run), [](bool) {}, [] { return false; }, requirements)
      .ToPtr();
}
CommandPtr Print(std::string_view msg) { return PrintCommand(msg).ToPtr(); }
CommandPtr Wait(units::second_t duration) { return WaitCommand(duration).ToPtr(); }
CommandPtr WaitUntil(std::function<bool()> condition) {
  return WaitUntilCommand(std::move(condition)).ToPtr();
}
CommandPtr Either(CommandPtr&& onTrue, CommandPtr&& onFalse, std::function<bool()> selector) {
  return CommandPtr(std::make_unique<ConditionalCommand>(
      std::move(onTrue).Unwrap(), std::move(onFalse).Unwrap(), std::move(selector)));
}
CommandPtr Defer(wpi::unique_function<CommandPtr()> supplier, Requirements requirements) {
  return DeferredCommand(std::move(supplier), requirements).ToPtr();
}
CommandPtr DeferredProxy(wpi::unique_function<Command*()> supplier) {
  return ProxyCommand(std::move(supplier)).ToPtr();
}
CommandPtr DeferredProxy(wpi::unique_function<CommandPtr()> supplier) {
  return ProxyCommand(std::move(supplier)).ToPtr();
}
CommandPtr Sequence(std::vector<CommandPtr>&& commands) {
  return CommandPtr(
      std::make_unique<SequentialCommandGroup>(CommandPtr::UnwrapVector(std::move(commands))));
}
CommandPtr RepeatingSequence(std::vector<CommandPtr>&& commands) {
  return Sequence(std::move(commands)).Repeatedly();
}
CommandPtr Parallel(std::vector<CommandPtr>&& commands) {
  return CommandPtr(
      std::make_unique<ParallelCommandGroup>(CommandPtr::UnwrapVector(std::move(commands))));
}
CommandPtr Race(std::vector<CommandPtr>&& commands) {
  return CommandPtr(
      std::make_unique<ParallelRaceGroup>(CommandPtr::UnwrapVector(std::move(commands))));
}
CommandPtr Deadline(CommandPtr&& deadline, std::vector<CommandPtr>&& others) {
  return CommandPtr(std::make_unique<ParallelDeadlineGroup>(
      std::move(deadline).Unwrap(), CommandPtr::UnwrapVector(std::move(others))));
}
}  // namespace cmd

// ------------------------------------------------------------------------------------ subsystems
Subsystem::~Subsystem() { CommandScheduler::GetInstance().UnregisterSubsystem(this); }
std::string Subsystem::GetName() const { return "Subsystem"; }
void Subsystem::SetName(std::string_view) {}
void Subsystem::SetDefaultCommand(CommandPtr&& defaultCommand) {
  CommandScheduler::GetInstance().SetDefaultCommand(this, std::move(defaultCommand));
}
void Subsystem::RemoveDefaultCommand() {
  CommandScheduler::GetInstance().RemoveDefaultCommand(this);
}
Command* Subsystem::GetDefaultCommand() const {
  return CommandScheduler::GetInstance().GetDefaultCommand(this);
}
Command* Subsystem::GetCurrentCommand() const {
  return CommandScheduler::GetInstance().Requiring(this);
}
void Subsystem::Register() { CommandScheduler::GetInstance().RegisterSubsystem(this); }
CommandPtr Subsystem::RunOnce(std::function<void()> action) {
  return cmd::RunOnce(std::move(action), {this});
}
CommandPtr Subsystem::Run(std::function<void()> action) {
  return cmd::Run(std::move(action), {this});
}
CommandPtr Subsystem::StartEnd(std::function<void()> start, std::function<void()> end) {
  return cmd::StartEnd(std::move(start), std::move(end), {this});
}
CommandPtr Subsystem::RunEnd(std::function<void()> run, std::function<void()> end) {
  return cmd::RunEnd(std::move(run), std::move(end), {this});
}
CommandPtr Subsystem::StartRun(std::function<void()> start, std::function<void()> run) {
  return cmd::StartRun(std::move(start), std::move(run), {this});
}
CommandPtr Subsystem::Defer(wpi::unique_function<CommandPtr()> supplier) {
  return cmd::Defer(std::move(supplier), {this});
}

SubsystemBase::SubsystemBase() {
  wpi::SendableRegistry::Add(this, "Unnamed");
  CommandScheduler::GetInstance().RegisterSubsystem(this);
}
SubsystemBase::SubsystemBase(std::string_view name) : m_name{name} {
  wpi::SendableRegistry::Add(this, name);
  CommandScheduler::GetInstance().RegisterSubsystem(this);
}
void SubsystemBase::InitSendable(wpi::SendableBuilder& builder) {
  builder.SetSmartDashboardType("Subsystem");
}

// -------------------------------------------------------------------------------------- triggers
void Trigger::AddBinding(wpi::unique_function<void(bool, bool)>&& body) {
  m_loop->Bind(
      [condition = m_condition, previous = m_condition(), body = std::move(body)]() mutable {
        bool current = condition();
        body(previous, current);
        previous = current;
      });
}

#define FRC2SHIM_BIND_PTR(Method, Body)                                      \
  Trigger Trigger::Method(CommandPtr&& command) {                            \
    AddBinding([command = std::move(command)](bool previous, bool current) { \
      Command* c = command.get();                                            \
      Body                                                                   \
    });                                                                      \
    return *this;                                                            \
  }                                                                          \
  Trigger Trigger::Method(Command* c) {                                      \
    AddBinding([c](bool previous, bool current) { Body });                   \
    return *this;                                                            \
  }

FRC2SHIM_BIND_PTR(
    OnChange, if (previous != current) { CommandScheduler::GetInstance().Schedule(c); })
FRC2SHIM_BIND_PTR(
    OnTrue, if (!previous && current) { CommandScheduler::GetInstance().Schedule(c); })
FRC2SHIM_BIND_PTR(
    OnFalse, if (previous && !current) { CommandScheduler::GetInstance().Schedule(c); })
FRC2SHIM_BIND_PTR(
    WhileTrue,
    if (!previous && current) {
      CommandScheduler::GetInstance().Schedule(c);
    } else if (previous && !current) { CommandScheduler::GetInstance().Cancel(c); })
FRC2SHIM_BIND_PTR(
    WhileFalse,
    if (previous && !current) {
      CommandScheduler::GetInstance().Schedule(c);
    } else if (!previous && current) { CommandScheduler::GetInstance().Cancel(c); })
FRC2SHIM_BIND_PTR(
    ToggleOnTrue, if (!previous && current) {
      if (c->IsScheduled()) {
        CommandScheduler::GetInstance().Cancel(c);
      } else {
        CommandScheduler::GetInstance().Schedule(c);
      }
    })
#undef FRC2SHIM_BIND_PTR

Trigger Trigger::Debounce(units::second_t debounceTime, frc::Debouncer::DebounceType type) {
  return Trigger(m_loop,
                 [debouncer = std::make_shared<frc::Debouncer>(debounceTime, type),
                  condition = m_condition]() mutable { return debouncer->Calculate(condition()); });
}

Trigger CommandGenericHID::Button(int button, frc::EventLoop* loop) const {
  return Trigger(loop, [hid = &m_hid, button] { return hid->GetRawButton(button); });
}
Trigger CommandGenericHID::POV(int angle, frc::EventLoop* loop) const {
  return POV(0, angle, loop);
}
Trigger CommandGenericHID::POV(int pov, int angle, frc::EventLoop* loop) const {
  return Trigger(loop, [hid = &m_hid, pov, angle] { return hid->GetPOV(pov) == angle; });
}
Trigger CommandGenericHID::POVUp(frc::EventLoop* loop) const { return POV(0, loop); }
Trigger CommandGenericHID::POVUpRight(frc::EventLoop* loop) const { return POV(45, loop); }
Trigger CommandGenericHID::POVRight(frc::EventLoop* loop) const { return POV(90, loop); }
Trigger CommandGenericHID::POVDownRight(frc::EventLoop* loop) const { return POV(135, loop); }
Trigger CommandGenericHID::POVDown(frc::EventLoop* loop) const { return POV(180, loop); }
Trigger CommandGenericHID::POVDownLeft(frc::EventLoop* loop) const { return POV(225, loop); }
Trigger CommandGenericHID::POVLeft(frc::EventLoop* loop) const { return POV(270, loop); }
Trigger CommandGenericHID::POVUpLeft(frc::EventLoop* loop) const { return POV(315, loop); }
Trigger CommandGenericHID::POVCenter(frc::EventLoop* loop) const { return POV(-1, loop); }
Trigger CommandGenericHID::AxisLessThan(int axis, double threshold, frc::EventLoop* loop) const {
  return Trigger(loop,
                 [hid = &m_hid, axis, threshold] { return hid->GetRawAxis(axis) < threshold; });
}
Trigger CommandGenericHID::AxisGreaterThan(int axis, double threshold, frc::EventLoop* loop) const {
  return Trigger(loop,
                 [hid = &m_hid, axis, threshold] { return hid->GetRawAxis(axis) > threshold; });
}

}  // namespace frc2
