// Desktop-only shim of WPILib's frc2 command framework. See tools/desktop/frc2shim/README.md.
#pragma once

#include <concepts>
#include <functional>
#include <memory>
#include <string_view>
#include <utility>
#include <vector>

#include <units/time.h>
#include <wpi/FunctionExtras.h>

#include "frc2/command/CommandClasses.h"
#include "frc2/command/CommandPtr.h"

namespace frc2::cmd {

[[nodiscard]] CommandPtr None();
[[nodiscard]] CommandPtr Idle(Requirements requirements = {});
[[nodiscard]] CommandPtr RunOnce(std::function<void()> action, Requirements requirements = {});
[[nodiscard]] CommandPtr Run(std::function<void()> action, Requirements requirements = {});
[[nodiscard]] CommandPtr StartEnd(std::function<void()> start, std::function<void()> end,
                                  Requirements requirements = {});
[[nodiscard]] CommandPtr RunEnd(std::function<void()> run, std::function<void()> end,
                                Requirements requirements = {});
[[nodiscard]] CommandPtr StartRun(std::function<void()> start, std::function<void()> run,
                                  Requirements requirements = {});
[[nodiscard]] CommandPtr Print(std::string_view msg);
[[nodiscard]] CommandPtr Wait(units::second_t duration);
[[nodiscard]] CommandPtr WaitUntil(std::function<bool()> condition);
[[nodiscard]] CommandPtr Either(CommandPtr&& onTrue, CommandPtr&& onFalse,
                                std::function<bool()> selector);
[[nodiscard]] CommandPtr Defer(wpi::unique_function<CommandPtr()> supplier,
                               Requirements requirements);
[[nodiscard]] CommandPtr DeferredProxy(wpi::unique_function<Command*()> supplier);
[[nodiscard]] CommandPtr DeferredProxy(wpi::unique_function<CommandPtr()> supplier);

namespace impl {
template <typename... Args>
std::vector<CommandPtr> MakeVector(Args&&... args) {
  std::vector<CommandPtr> data;
  data.reserve(sizeof...(Args));
  (data.emplace_back(std::forward<Args>(args)), ...);
  return data;
}
}  // namespace impl

[[nodiscard]] CommandPtr Sequence(std::vector<CommandPtr>&& commands);
template <std::convertible_to<CommandPtr>... CommandPtrs>
[[nodiscard]] CommandPtr Sequence(CommandPtrs&&... commands) {
  return Sequence(impl::MakeVector(std::forward<CommandPtrs>(commands)...));
}
[[nodiscard]] CommandPtr RepeatingSequence(std::vector<CommandPtr>&& commands);
template <std::convertible_to<CommandPtr>... CommandPtrs>
[[nodiscard]] CommandPtr RepeatingSequence(CommandPtrs&&... commands) {
  return RepeatingSequence(impl::MakeVector(std::forward<CommandPtrs>(commands)...));
}
[[nodiscard]] CommandPtr Parallel(std::vector<CommandPtr>&& commands);
template <std::convertible_to<CommandPtr>... CommandPtrs>
[[nodiscard]] CommandPtr Parallel(CommandPtrs&&... commands) {
  return Parallel(impl::MakeVector(std::forward<CommandPtrs>(commands)...));
}
[[nodiscard]] CommandPtr Race(std::vector<CommandPtr>&& commands);
template <std::convertible_to<CommandPtr>... CommandPtrs>
[[nodiscard]] CommandPtr Race(CommandPtrs&&... commands) {
  return Race(impl::MakeVector(std::forward<CommandPtrs>(commands)...));
}
[[nodiscard]] CommandPtr Deadline(CommandPtr&& deadline, std::vector<CommandPtr>&& others);
template <std::convertible_to<CommandPtr>... CommandPtrs>
[[nodiscard]] CommandPtr Deadline(CommandPtr&& deadline, CommandPtrs&&... commands) {
  return Deadline(std::move(deadline), impl::MakeVector(std::forward<CommandPtrs>(commands)...));
}

}  // namespace frc2::cmd
