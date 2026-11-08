// Desktop-only shim of WPILib's frc2 command framework. See tools/desktop/frc2shim/README.md.
#pragma once

#include <concepts>
#include <functional>
#include <string>
#include <string_view>
#include <utility>

#include <wpi/FunctionExtras.h>

#include "frc2/command/CommandPtr.h"

namespace frc2 {

class Command;

class Subsystem {
 public:
  virtual ~Subsystem();
  virtual void Periodic() {}
  virtual void SimulationPeriodic() {}
  virtual std::string GetName() const;
  virtual void SetName(std::string_view name);

  template <std::derived_from<Command> T>
  void SetDefaultCommand(T&& defaultCommand) {
    SetDefaultCommand(CommandPtr{std::forward<T>(defaultCommand)});
  }
  void SetDefaultCommand(CommandPtr&& defaultCommand);
  void RemoveDefaultCommand();
  Command* GetDefaultCommand() const;
  Command* GetCurrentCommand() const;
  void Register();

  [[nodiscard]] CommandPtr RunOnce(std::function<void()> action);
  [[nodiscard]] CommandPtr Run(std::function<void()> action);
  [[nodiscard]] CommandPtr StartEnd(std::function<void()> start, std::function<void()> end);
  [[nodiscard]] CommandPtr RunEnd(std::function<void()> run, std::function<void()> end);
  [[nodiscard]] CommandPtr StartRun(std::function<void()> start, std::function<void()> run);
  [[nodiscard]] CommandPtr Defer(wpi::unique_function<CommandPtr()> supplier);
};

}  // namespace frc2
