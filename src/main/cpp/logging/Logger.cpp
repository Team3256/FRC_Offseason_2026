// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "logging/Logger.h"

#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <system_error>
#include <vector>

#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc/RobotBase.h>
#include <networktables/NetworkTableInstance.h>

namespace logging {

namespace {
wpi::log::DataLog* g_log = nullptr;
std::vector<NT_DataLogger> g_ntLoggers;
NT_ConnectionDataLogger g_ntConnectionLogger = 0;

// Runs before DataLogManager's static log is destroyed (registered after it was created), so
// NetworkTables never writes to a destroyed log while the program exits.
void DetachNetworkTablesLoggers() {
  for (auto h : g_ntLoggers) {
    nt::NetworkTableInstance::StopEntryDataLog(h);
  }
  g_ntLoggers.clear();
  if (g_ntConnectionLogger != 0) {
    nt::NetworkTableInstance::StopConnectionDataLog(g_ntConnectionLogger);
    g_ntConnectionLogger = 0;
  }
}
}  // namespace

void Logger::Start(std::string_view dir) {
  if (g_log != nullptr || dir.empty()) {
    return;
  }
  std::error_code ec;
  std::filesystem::create_directories(std::string{dir}, ec);
  frc::DataLogManager::Start(dir);
  // Our values are written straight to the log; only mirror the NetworkTables data that does not
  // come from this logger (dashboard toggles, choosers, Phoenix swerve telemetry, Mayhem
  // telemetry).
  frc::DataLogManager::LogNetworkTables(false);
  g_log = &frc::DataLogManager::GetLog();
  auto inst = nt::NetworkTableInstance::GetDefault();
  g_ntLoggers.push_back(inst.StartEntryDataLog(*g_log, "/SmartDashboard/", "NT:/SmartDashboard/"));
  g_ntLoggers.push_back(inst.StartEntryDataLog(*g_log, "/DriveState/", "NT:/DriveState/"));
  g_ntLoggers.push_back(inst.StartEntryDataLog(*g_log, "/Mayhem/", "NT:/Mayhem/"));
  g_ntConnectionLogger = inst.StartConnectionDataLog(*g_log, "NTConnection");
  std::atexit(DetachNetworkTablesLoggers);
  // Driver Station state and joysticks (AdvantageKit logged these too).
  frc::DriverStation::StartDataLog(*g_log, true);
}

wpi::log::DataLog* Logger::GetLog() { return g_log; }

void Logger::Periodic() {
  // Same rule as the old NT4PublisherNoFMS: keep NetworkTables quiet on the field.
  s_ntActive = !(frc::RobotBase::IsReal() && frc::DriverStation::IsFMSAttached());
}

void Logger::Message(std::string_view message) {
  if (g_log != nullptr) {
    frc::DataLogManager::Log(message);
  } else {
    std::fwrite(message.data(), 1, message.size(), stdout);
    std::fputc('\n', stdout);
  }
}

std::string Logger::LogKey(std::string_view key) { return std::string{"/"}.append(key); }

std::string Logger::NtKey(std::string_view key) {
  return std::string{"/AdvantageKit/"}.append(key);
}

void InputsLogger::Add(std::string_view name, const double* value) {
  m_doubles.emplace_back(value, std::make_unique<LogValue<double>>(Key(name)));
}

void InputsLogger::Add(std::string_view name, const bool* value) {
  m_bools.emplace_back(value, std::make_unique<LogValue<bool>>(Key(name)));
}

void InputsLogger::Add(std::string_view name, std::span<const double> values) {
  m_arrays.emplace_back(values, std::make_unique<LogValue<std::vector<double>>>(Key(name)));
}

void InputsLogger::Publish() {
  for (auto& [value, entry] : m_doubles) {
    entry->Set(*value);
  }
  for (auto& [value, entry] : m_bools) {
    entry->Set(*value);
  }
  for (auto& [values, entry] : m_arrays) {
    entry->Set(values);
  }
}

}  // namespace logging
