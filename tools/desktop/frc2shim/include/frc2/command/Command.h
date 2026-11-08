// Desktop-only shim of WPILib's frc2 command framework. See tools/desktop/frc2shim/README.md.
#pragma once

#include <functional>
#include <initializer_list>
#include <memory>
#include <span>
#include <string>
#include <string_view>
#include <utility>
#include <vector>

#include <units/time.h>
#include <wpi/FunctionExtras.h>
#include <wpi/SmallSet.h>
#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableHelper.h>

namespace frc2 {

class Subsystem;
class CommandPtr;

/** Same shape as frc2::Requirements: a list of subsystem pointers. */
class Requirements {
 public:
  Requirements() = default;
  Requirements(std::initializer_list<Subsystem*> requirements)  // NOLINT
      : m_subsystems{requirements.begin(), requirements.end()} {}
  Requirements(std::span<Subsystem* const> requirements)  // NOLINT
      : m_subsystems{requirements.begin(), requirements.end()} {}
  Requirements(const wpi::SmallSet<Subsystem*, 4>& requirements)  // NOLINT
      : m_subsystems{requirements.begin(), requirements.end()} {}
  std::vector<Subsystem*>::const_iterator begin() const { return m_subsystems.begin(); }
  std::vector<Subsystem*>::const_iterator end() const { return m_subsystems.end(); }

 private:
  std::vector<Subsystem*> m_subsystems;
};

class Command : public wpi::Sendable, public wpi::SendableHelper<Command> {
 public:
  enum class InterruptionBehavior { kCancelSelf, kCancelIncoming };

  Command();
  ~Command() override;
  Command(const Command&) = default;
  Command& operator=(const Command& rhs) = default;
  Command(Command&&) = default;
  Command& operator=(Command&&) = default;

  virtual void Initialize() {}
  virtual void Execute() {}
  virtual void End(bool interrupted) {}
  virtual bool IsFinished() { return false; }

  virtual wpi::SmallSet<Subsystem*, 4> GetRequirements() const { return m_requirements; }
  void AddRequirements(Requirements requirements);
  void AddRequirements(wpi::SmallSet<Subsystem*, 4> requirements);
  void AddRequirements(Subsystem* requirement);

  std::string GetName() const { return m_name; }
  void SetName(std::string_view name) { m_name = name; }
  std::string GetSubsystem() const { return m_subsystem; }
  void SetSubsystem(std::string_view subsystem) { m_subsystem = subsystem; }

  [[nodiscard]] CommandPtr WithTimeout(units::second_t duration) &&;
  [[nodiscard]] CommandPtr Until(std::function<bool()> condition) &&;
  [[nodiscard]] CommandPtr OnlyWhile(std::function<bool()> condition) &&;
  [[nodiscard]] CommandPtr IgnoringDisable(bool doesRunWhenDisabled) &&;
  [[nodiscard]] CommandPtr WithInterruptBehavior(InterruptionBehavior interruptBehavior) &&;
  [[nodiscard]] CommandPtr AndThen(std::function<void()> toRun, Requirements requirements = {}) &&;
  [[nodiscard]] CommandPtr AndThen(CommandPtr&& next) &&;
  [[nodiscard]] CommandPtr BeforeStarting(std::function<void()> toRun,
                                          Requirements requirements = {}) &&;
  [[nodiscard]] CommandPtr BeforeStarting(CommandPtr&& before) &&;
  [[nodiscard]] CommandPtr Repeatedly() &&;
  [[nodiscard]] CommandPtr AsProxy() &&;
  [[nodiscard]] CommandPtr Unless(std::function<bool()> condition) &&;
  [[nodiscard]] CommandPtr OnlyIf(std::function<bool()> condition) &&;
  [[nodiscard]] CommandPtr DeadlineFor(CommandPtr&& parallel) &&;
  [[nodiscard]] CommandPtr AlongWith(CommandPtr&& parallel) &&;
  [[nodiscard]] CommandPtr RaceWith(CommandPtr&& parallel) &&;
  [[nodiscard]] CommandPtr FinallyDo(std::function<void(bool)> end) &&;
  [[nodiscard]] CommandPtr FinallyDo(std::function<void()> end) &&;
  [[nodiscard]] CommandPtr HandleInterrupt(std::function<void()> handler) &&;
  [[nodiscard]] CommandPtr WithName(std::string_view name) &&;

  void Cancel();
  bool IsScheduled() const;
  bool HasRequirement(Subsystem* requirement) const;
  bool IsComposed() const { return m_isComposed; }
  void SetComposed(bool isComposed) { m_isComposed = isComposed; }

  virtual bool RunsWhenDisabled() const { return false; }
  virtual InterruptionBehavior GetInterruptionBehavior() const {
    return InterruptionBehavior::kCancelSelf;
  }

  virtual CommandPtr ToPtr() && = 0;

  void InitSendable(wpi::SendableBuilder& builder) override;

 protected:
  wpi::SmallSet<Subsystem*, 4> m_requirements;

 private:
  std::string m_name{"Command"};
  std::string m_subsystem{"Ungrouped"};
  bool m_isComposed = false;
};

bool RequirementsDisjoint(Command* first, Command* second);

}  // namespace frc2

#include "frc2/command/CommandPtr.h"
