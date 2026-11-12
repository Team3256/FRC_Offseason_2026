// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <functional>
#include <map>
#include <optional>
#include <string>
#include <vector>

#include <frc2/command/CommandPtr.h>
#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableHelper.h>

/**
 * Dashboard chooser for autonomous routines. The selected routine's command is built while
 * disabled (so the first autonomous loop does not pay for it) and run as a proxy.
 */
class AutoChooser : public wpi::Sendable, public wpi::SendableHelper<AutoChooser> {
 public:
  static constexpr const char* kNoneName = "Nothing";

  AutoChooser();

  /** Adds an option; {@code generator} runs only when this option is selected. */
  AutoChooser& AddCmd(const std::string& name, std::function<frc2::CommandPtr()> generator);
  /** Selects an option by name, as the dashboard would. Unknown names are ignored. */
  void Select(const std::string& name);
  const std::string& Selected() const { return m_selected; }
  std::vector<std::string> Options() const { return m_order; }
  /** The command for the selected option, generating it if the selection changed. */
  frc2::Command* SelectedCommand();
  /** Runs the selected auto as a proxy; bind it to RobotModeTriggers::Autonomous().WhileTrue. */
  frc2::CommandPtr SelectedCommandScheduler();

  void InitSendable(wpi::SendableBuilder& builder) override;

 private:
  void Build();

  std::map<std::string, std::function<frc2::CommandPtr()>> m_generators;
  std::vector<std::string> m_order;
  int m_instance;
  std::string m_selected = kNoneName;
  std::string m_builtFor = kNoneName;
  std::optional<frc2::CommandPtr> m_built;
};
