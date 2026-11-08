// Desktop-only shim of WPILib's frc2 command framework. See tools/desktop/frc2shim/README.md.
#pragma once

#include <concepts>
#include <functional>
#include <utility>

#include <frc/event/EventLoop.h>
#include <frc/filter/Debouncer.h>
#include <units/time.h>
#include <wpi/FunctionExtras.h>

#include "frc2/command/Command.h"
#include "frc2/command/CommandPtr.h"
#include "frc2/command/CommandScheduler.h"

namespace frc2 {

class Trigger {
 public:
  explicit Trigger(std::function<bool()> condition)
      : Trigger{CommandScheduler::GetInstance().GetDefaultButtonLoop(), std::move(condition)} {}
  Trigger(frc::EventLoop* loop, std::function<bool()> condition)
      : m_loop{loop}, m_condition{std::move(condition)} {}
  Trigger() : Trigger([] { return false; }) {}
  Trigger(const Trigger& other) = default;

  Trigger OnChange(Command* command);
  Trigger OnChange(CommandPtr&& command);
  Trigger OnTrue(Command* command);
  Trigger OnTrue(CommandPtr&& command);
  Trigger OnFalse(Command* command);
  Trigger OnFalse(CommandPtr&& command);
  Trigger WhileTrue(Command* command);
  Trigger WhileTrue(CommandPtr&& command);
  Trigger WhileFalse(Command* command);
  Trigger WhileFalse(CommandPtr&& command);
  Trigger ToggleOnTrue(Command* command);
  Trigger ToggleOnTrue(CommandPtr&& command);

  template <std::convertible_to<std::function<bool()>> F>
  Trigger operator&&(F&& rhs) {
    return Trigger(m_loop,
                   [condition = m_condition, rhs = std::function<bool()>(std::forward<F>(rhs))] {
                     return condition() && rhs();
                   });
  }
  Trigger operator&&(Trigger rhs) {
    return Trigger(m_loop, [condition = m_condition, rhs] { return condition() && rhs.Get(); });
  }
  template <std::convertible_to<std::function<bool()>> F>
  Trigger operator||(F&& rhs) {
    return Trigger(m_loop,
                   [condition = m_condition, rhs = std::function<bool()>(std::forward<F>(rhs))] {
                     return condition() || rhs();
                   });
  }
  Trigger operator||(Trigger rhs) {
    return Trigger(m_loop, [condition = m_condition, rhs] { return condition() || rhs.Get(); });
  }
  Trigger operator!() {
    return Trigger(m_loop, [condition = m_condition] { return !condition(); });
  }

  Trigger Debounce(units::second_t debounceTime,
                   frc::Debouncer::DebounceType type = frc::Debouncer::DebounceType::kRising);

  bool Get() const { return m_condition(); }

 private:
  void AddBinding(wpi::unique_function<void(bool, bool)>&& body);
  frc::EventLoop* m_loop;
  std::function<bool()> m_condition;
};

}  // namespace frc2
