// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableHelper.h>

/**
 * An autonomous routine. RobotContainer calls Start() when autonomous is enabled, Periodic() every
 * autonomous loop after that (starting with the same loop as Start()), and Stop() when autonomous
 * ends. While a routine runs, it owns the drivetrain.
 */
class AutoRoutine {
 public:
  virtual ~AutoRoutine() = default;
  virtual void Start() = 0;
  virtual void Periodic() = 0;
  virtual void Stop() = 0;
};

/**
 * Dashboard chooser for autonomous routines. The selected routine is built while disabled (so the
 * first autonomous loop does not pay for it).
 */
class AutoChooser : public wpi::Sendable, public wpi::SendableHelper<AutoChooser> {
 public:
  static constexpr const char* kNoneName = "Nothing";

  AutoChooser();

  /** Adds an option; {@code generator} runs only when this option is selected. */
  AutoChooser& AddRoutine(const std::string& name,
                          std::function<std::unique_ptr<AutoRoutine>()> generator);
  /** Selects an option by name, as the dashboard would. Unknown names are ignored. */
  void Select(const std::string& name);
  const std::string& Selected() const { return m_selected; }
  std::vector<std::string> Options() const { return m_order; }
  /**
   * The routine for the selected option, building it if the selection changed; null for
   * "Nothing". Stays valid until the next call after the selection changes.
   */
  AutoRoutine* SelectedRoutine();

  void InitSendable(wpi::SendableBuilder& builder) override;

 private:
  void Build();

  std::map<std::string, std::function<std::unique_ptr<AutoRoutine>()>> m_generators;
  std::vector<std::string> m_order;
  int m_instance;
  std::string m_selected = kNoneName;
  std::string m_builtFor = kNoneName;
  std::unique_ptr<AutoRoutine> m_built;
};
