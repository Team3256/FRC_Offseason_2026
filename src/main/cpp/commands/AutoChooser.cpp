// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "commands/AutoChooser.h"

#include <atomic>

#include <frc/DriverStation.h>
#include <wpi/sendable/SendableBuilder.h>
#include <wpi/sendable/SendableRegistry.h>

#include "utils/DSReport.h"

namespace {
std::atomic<int> g_instances{0};
}  // namespace

AutoChooser::AutoChooser() : m_instance{g_instances.fetch_add(1)} {
  m_generators[kNoneName] = [] { return nullptr; };
  m_order.push_back(kNoneName);
  wpi::SendableRegistry::Add(this, "AutoChooser", m_instance);
}

AutoChooser& AutoChooser::AddRoutine(const std::string& name,
                                     std::function<std::unique_ptr<AutoRoutine>()> generator) {
  if (!m_generators.contains(name)) {
    m_order.push_back(name);
  }
  m_generators[name] = std::move(generator);
  return *this;
}

void AutoChooser::Select(const std::string& name) {
  if (!m_generators.contains(name)) {
    ds_report::Warning("AutoChooser has no option named '" + name + "'");
    return;
  }
  m_selected = name;
  if (frc::DriverStation::IsDisabled()) {
    Build();
  }
}

AutoRoutine* AutoChooser::SelectedRoutine() {
  Build();
  return m_built.get();
}

void AutoChooser::Build() {
  if (m_selected != m_builtFor) {
    m_built.reset();  // first: routines publish to the same NT/log entries
    m_built = m_generators[m_selected]();
    m_builtFor = m_selected;
  }
}

void AutoChooser::InitSendable(wpi::SendableBuilder& builder) {
  builder.SetSmartDashboardType("String Chooser");
  builder.PublishConstInteger(".instance", m_instance);
  builder.AddStringProperty("default", [] { return std::string{kNoneName}; }, nullptr);
  builder.AddStringArrayProperty("options", [this] { return m_order; }, nullptr);
  builder.AddStringProperty("active", [this] { return m_selected; }, nullptr);
  builder.AddStringProperty("selected", nullptr,
                            [this](std::string_view val) { Select(std::string{val}); });
}
