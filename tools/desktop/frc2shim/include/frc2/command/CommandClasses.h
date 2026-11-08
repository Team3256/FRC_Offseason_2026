// Desktop-only shim of WPILib's frc2 command framework. See tools/desktop/frc2shim/README.md.
// All concrete command types live here; the WPILib-named headers forward to this file.
#pragma once

#include <functional>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include <frc/Timer.h>
#include <units/time.h>
#include <wpi/FunctionExtras.h>

#include "frc2/command/Command.h"
#include "frc2/command/CommandHelper.h"
#include "frc2/command/CommandPtr.h"

namespace frc2 {

class FunctionalCommand : public CommandHelper<Command, FunctionalCommand> {
 public:
  FunctionalCommand(std::function<void()> onInit, std::function<void()> onExecute,
                    std::function<void(bool)> onEnd, std::function<bool()> isFinished,
                    Requirements requirements = {});
  FunctionalCommand(FunctionalCommand&& other) = default;
  FunctionalCommand(const FunctionalCommand& other) = default;
  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

 private:
  std::function<void()> m_onInit;
  std::function<void()> m_onExecute;
  std::function<void(bool)> m_onEnd;
  std::function<bool()> m_isFinished;
};

class InstantCommand : public CommandHelper<FunctionalCommand, InstantCommand> {
 public:
  explicit InstantCommand(std::function<void()> toRun, Requirements requirements = {});
  InstantCommand();
};

class RunCommand : public CommandHelper<FunctionalCommand, RunCommand> {
 public:
  RunCommand(std::function<void()> toRun, Requirements requirements = {});
};

class StartEndCommand : public CommandHelper<FunctionalCommand, StartEndCommand> {
 public:
  StartEndCommand(std::function<void()> onInit, std::function<void()> onEnd,
                  Requirements requirements = {});
};

class WaitCommand : public CommandHelper<Command, WaitCommand> {
 public:
  explicit WaitCommand(units::second_t duration);
  void Initialize() override;
  void End(bool interrupted) override;
  bool IsFinished() override;
  bool RunsWhenDisabled() const override { return true; }

 protected:
  frc::Timer m_timer;

 private:
  units::second_t m_duration;
};

class WaitUntilCommand : public CommandHelper<Command, WaitUntilCommand> {
 public:
  explicit WaitUntilCommand(std::function<bool()> condition);
  bool IsFinished() override;
  bool RunsWhenDisabled() const override { return true; }

 private:
  std::function<bool()> m_condition;
};

class PrintCommand : public CommandHelper<InstantCommand, PrintCommand> {
 public:
  explicit PrintCommand(std::string_view message);
  bool RunsWhenDisabled() const override { return true; }
};

/** Base for decorators that forward to an inner command. */
class WrapperCommand : public CommandHelper<Command, WrapperCommand> {
 public:
  explicit WrapperCommand(std::unique_ptr<Command>&& command);
  template <std::derived_from<Command> T>
  explicit WrapperCommand(T&& command)
      : WrapperCommand(
            std::unique_ptr<Command>(std::make_unique<std::decay_t<T>>(std::forward<T>(command)))) {
  }
  WrapperCommand(WrapperCommand&& other) = default;
  WrapperCommand(const WrapperCommand& other) = delete;
  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End(bool interrupted) override;
  bool RunsWhenDisabled() const override;
  InterruptionBehavior GetInterruptionBehavior() const override;
  wpi::SmallSet<Subsystem*, 4> GetRequirements() const override;

 protected:
  std::unique_ptr<Command> m_command;
};

class SequentialCommandGroup : public CommandHelper<Command, SequentialCommandGroup> {
 public:
  explicit SequentialCommandGroup(std::vector<std::unique_ptr<Command>>&& commands);
  SequentialCommandGroup(SequentialCommandGroup&& other) = default;
  SequentialCommandGroup(const SequentialCommandGroup&) = delete;
  void AddCommands(std::vector<std::unique_ptr<Command>>&& commands);
  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;
  bool RunsWhenDisabled() const override { return m_runWhenDisabled; }
  InterruptionBehavior GetInterruptionBehavior() const override { return m_interruptBehavior; }

 private:
  std::vector<std::unique_ptr<Command>> m_commands;
  size_t m_currentCommandIndex = static_cast<size_t>(-1);
  bool m_runWhenDisabled = true;
  InterruptionBehavior m_interruptBehavior = InterruptionBehavior::kCancelIncoming;
};

class ParallelCommandGroup : public CommandHelper<Command, ParallelCommandGroup> {
 public:
  explicit ParallelCommandGroup(std::vector<std::unique_ptr<Command>>&& commands);
  ParallelCommandGroup(ParallelCommandGroup&& other) = default;
  ParallelCommandGroup(const ParallelCommandGroup&) = delete;
  void AddCommands(std::vector<std::unique_ptr<Command>>&& commands);
  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;
  bool RunsWhenDisabled() const override { return m_runWhenDisabled; }
  InterruptionBehavior GetInterruptionBehavior() const override { return m_interruptBehavior; }

 private:
  std::vector<std::pair<std::unique_ptr<Command>, bool>> m_commands;
  bool isRunning = false;
  bool m_runWhenDisabled = true;
  InterruptionBehavior m_interruptBehavior = InterruptionBehavior::kCancelIncoming;
};

class ParallelRaceGroup : public CommandHelper<Command, ParallelRaceGroup> {
 public:
  explicit ParallelRaceGroup(std::vector<std::unique_ptr<Command>>&& commands);
  ParallelRaceGroup(ParallelRaceGroup&& other) = default;
  ParallelRaceGroup(const ParallelRaceGroup&) = delete;
  void AddCommands(std::vector<std::unique_ptr<Command>>&& commands);
  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;
  bool RunsWhenDisabled() const override { return m_runWhenDisabled; }
  InterruptionBehavior GetInterruptionBehavior() const override { return m_interruptBehavior; }

 private:
  std::vector<std::unique_ptr<Command>> m_commands;
  bool m_runWhenDisabled = true;
  bool m_finished = false;
  bool isRunning = false;
  InterruptionBehavior m_interruptBehavior = InterruptionBehavior::kCancelIncoming;
};

class ParallelDeadlineGroup : public CommandHelper<Command, ParallelDeadlineGroup> {
 public:
  ParallelDeadlineGroup(std::unique_ptr<Command>&& deadline,
                        std::vector<std::unique_ptr<Command>>&& commands);
  ParallelDeadlineGroup(ParallelDeadlineGroup&& other) = default;
  ParallelDeadlineGroup(const ParallelDeadlineGroup&) = delete;
  void AddCommands(std::vector<std::unique_ptr<Command>>&& commands);
  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;
  bool RunsWhenDisabled() const override { return m_runWhenDisabled; }
  InterruptionBehavior GetInterruptionBehavior() const override { return m_interruptBehavior; }

 private:
  std::vector<std::pair<std::unique_ptr<Command>, bool>> m_commands;
  Command* m_deadline = nullptr;
  bool m_runWhenDisabled = true;
  bool m_finished = true;
  InterruptionBehavior m_interruptBehavior = InterruptionBehavior::kCancelIncoming;
};

class RepeatCommand : public CommandHelper<Command, RepeatCommand> {
 public:
  explicit RepeatCommand(std::unique_ptr<Command>&& command);
  RepeatCommand(RepeatCommand&& other) = default;
  void Initialize() override;
  void Execute() override;
  bool IsFinished() override;
  void End(bool interrupted) override;
  bool RunsWhenDisabled() const override;
  InterruptionBehavior GetInterruptionBehavior() const override;

 private:
  std::unique_ptr<Command> m_command;
  bool m_ended = false;
};

class ProxyCommand : public CommandHelper<Command, ProxyCommand> {
 public:
  explicit ProxyCommand(wpi::unique_function<Command*()> supplier);
  explicit ProxyCommand(wpi::unique_function<CommandPtr()> supplier);
  explicit ProxyCommand(Command* command);
  explicit ProxyCommand(std::unique_ptr<Command> command);
  ProxyCommand(ProxyCommand&& other) = default;
  void Initialize() override;
  void End(bool interrupted) override;
  void Execute() override {}
  bool IsFinished() override;
  bool RunsWhenDisabled() const override { return true; }

 private:
  wpi::unique_function<Command*()> m_supplier;
  Command* m_command = nullptr;
  std::shared_ptr<Command> m_owned;  // keeps an owned command alive
};

class DeferredCommand : public CommandHelper<Command, DeferredCommand> {
 public:
  DeferredCommand(wpi::unique_function<CommandPtr()> supplier, Requirements requirements);
  DeferredCommand(DeferredCommand&& other) = default;
  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

 private:
  wpi::unique_function<CommandPtr()> m_supplier;
  std::optional<CommandPtr> m_command;
};

class ConditionalCommand : public CommandHelper<Command, ConditionalCommand> {
 public:
  ConditionalCommand(std::unique_ptr<Command>&& onTrue, std::unique_ptr<Command>&& onFalse,
                     std::function<bool()> condition);
  ConditionalCommand(ConditionalCommand&& other) = default;
  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;
  bool RunsWhenDisabled() const override;

 private:
  std::unique_ptr<Command> m_onTrue;
  std::unique_ptr<Command> m_onFalse;
  std::function<bool()> m_condition;
  Command* m_selectedCommand = nullptr;
};

class ScheduleCommand : public CommandHelper<Command, ScheduleCommand> {
 public:
  explicit ScheduleCommand(Command* toSchedule);
  ScheduleCommand(ScheduleCommand&& other) = default;
  void Initialize() override;
  bool IsFinished() override { return true; }
  bool RunsWhenDisabled() const override { return true; }

 private:
  Command* m_toSchedule;
};

}  // namespace frc2
