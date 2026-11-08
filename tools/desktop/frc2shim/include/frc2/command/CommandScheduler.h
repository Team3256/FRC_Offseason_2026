// Desktop-only shim of WPILib's frc2 command framework. See tools/desktop/frc2shim/README.md.
#pragma once

#include <functional>
#include <memory>
#include <span>
#include <utility>
#include <vector>

#include <frc/event/EventLoop.h>
#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableHelper.h>

#include "frc2/command/CommandPtr.h"

namespace frc2 {

class Command;
class Subsystem;

class CommandScheduler final : public wpi::Sendable, public wpi::SendableHelper<CommandScheduler> {
 public:
  static CommandScheduler& GetInstance();
  ~CommandScheduler() override;
  CommandScheduler(const CommandScheduler&) = delete;
  CommandScheduler& operator=(const CommandScheduler&) = delete;

  using Action = std::function<void(const Command&)>;

  void SetPeriod(units::second_t period) {}
  frc::EventLoop* GetActiveButtonLoop() const;
  void SetActiveButtonLoop(frc::EventLoop* loop);
  frc::EventLoop* GetDefaultButtonLoop() const;

  void Schedule(const CommandPtr& command);
  void Schedule(CommandPtr&& command);
  void Schedule(Command* command);
  void Schedule(std::span<Command* const> commands);
  void Schedule(std::initializer_list<Command*> commands);

  void Run();

  void RegisterSubsystem(Subsystem* subsystem);
  void UnregisterSubsystem(Subsystem* subsystem);
  void UnregisterAllSubsystems();

  void SetDefaultCommand(Subsystem* subsystem, CommandPtr&& defaultCommand);
  void RemoveDefaultCommand(Subsystem* subsystem);
  Command* GetDefaultCommand(const Subsystem* subsystem) const;

  void Cancel(Command* command);
  void Cancel(const CommandPtr& command);
  void Cancel(std::span<Command* const> commands);
  void CancelAll();

  bool IsScheduled(const Command* command) const;
  bool IsScheduled(const CommandPtr& command) const;
  Command* Requiring(const Subsystem* subsystem) const;

  void Disable();
  void Enable();

  void OnCommandInitialize(Action action);
  void OnCommandExecute(Action action);
  void OnCommandInterrupt(Action action);
  void OnCommandFinish(Action action);

  void RequireUngrouped(const Command* command);
  void RequireUngroupedAndUnscheduled(const Command* command);

  /** Shim only: drops all state (commands, subsystems, bindings). Used between unit tests. */
  void ResetForTesting();

  void InitSendable(wpi::SendableBuilder& builder) override;

 private:
  CommandScheduler();
  struct Impl;
  std::unique_ptr<Impl> m_impl;
  friend class CommandPtr;
};

}  // namespace frc2
