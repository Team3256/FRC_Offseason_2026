// Desktop-only shim of WPILib's frc2 command framework. See tools/desktop/frc2shim/README.md.
#pragma once

#include <concepts>
#include <functional>
#include <memory>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

#include <units/time.h>

#include "frc2/command/Command.h"

namespace frc2 {

class CommandPtr final {
 public:
  explicit CommandPtr(std::unique_ptr<Command>&& command);

  template <std::derived_from<Command> T>
  explicit CommandPtr(T&& command)
      : CommandPtr(
            std::unique_ptr<Command>(std::make_unique<std::decay_t<T>>(std::forward<T>(command)))) {
  }

  CommandPtr(CommandPtr&&) = default;
  CommandPtr& operator=(CommandPtr&&) = default;
  explicit CommandPtr(std::nullptr_t) = delete;

  [[nodiscard]] CommandPtr Repeatedly() &&;
  [[nodiscard]] CommandPtr AsProxy() &&;
  [[nodiscard]] CommandPtr IgnoringDisable(bool doesRunWhenDisabled) &&;
  [[nodiscard]] CommandPtr WithInterruptBehavior(
      Command::InterruptionBehavior interruptBehavior) &&;
  [[nodiscard]] CommandPtr AndThen(std::function<void()> toRun, Requirements requirements = {}) &&;
  [[nodiscard]] CommandPtr AndThen(CommandPtr&& next) &&;
  [[nodiscard]] CommandPtr BeforeStarting(std::function<void()> toRun,
                                          Requirements requirements = {}) &&;
  [[nodiscard]] CommandPtr BeforeStarting(CommandPtr&& before) &&;
  [[nodiscard]] CommandPtr WithTimeout(units::second_t duration) &&;
  [[nodiscard]] CommandPtr Until(std::function<bool()> condition) &&;
  [[nodiscard]] CommandPtr OnlyWhile(std::function<bool()> condition) &&;
  [[nodiscard]] CommandPtr Unless(std::function<bool()> condition) &&;
  [[nodiscard]] CommandPtr OnlyIf(std::function<bool()> condition) &&;
  [[nodiscard]] CommandPtr DeadlineFor(CommandPtr&& parallel) &&;
  [[nodiscard]] CommandPtr AlongWith(CommandPtr&& parallel) &&;
  [[nodiscard]] CommandPtr RaceWith(CommandPtr&& parallel) &&;
  [[nodiscard]] CommandPtr FinallyDo(std::function<void(bool)> end) &&;
  [[nodiscard]] CommandPtr FinallyDo(std::function<void()> end) &&;
  [[nodiscard]] CommandPtr HandleInterrupt(std::function<void()> handler) &&;
  [[nodiscard]] CommandPtr WithName(std::string_view name) &&;

  Command* get() const&;
  Command* get() && = delete;
  std::unique_ptr<Command> Unwrap() &&;
  void Schedule() const&;
  void Cancel() const&;
  bool IsScheduled() const&;
  bool HasRequirement(Subsystem* requirement) const&;
  explicit operator bool() const&;

  static std::vector<std::unique_ptr<Command>> UnwrapVector(std::vector<CommandPtr>&& vec);

 private:
  std::unique_ptr<Command> m_ptr;
  void AssertValid() const;
};

}  // namespace frc2
