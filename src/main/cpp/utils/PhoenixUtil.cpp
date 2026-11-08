// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "utils/PhoenixUtil.h"

#include <string>
#include <vector>

#include <frc/RobotBase.h>
#include "utils/DSReport.h"

#include "utils/PhoenixConfigEquality.h"

namespace phoenix_util {

namespace {
std::vector<ctre::phoenix6::BaseStatusSignal*> g_canivoreSignals;
std::vector<ctre::phoenix6::BaseStatusSignal*> g_rioSignals;

std::string Attempt(int i, int numTries) {
  return " (attempt " + std::to_string(i + 1) + " of " + std::to_string(numTries) + ")";
}

template <typename Device, typename Config>
bool ReadAndVerify(Device& device, const Config& config, const char* kind) {
  Config readConfig{};
  if (!SpamGetStatusCode([&] { return device.GetConfigurator().Refresh(readConfig); })) {
    ds_report::Warning(std::string{"Failed to read config for "} + kind + " [" +
                       std::to_string(device.GetDeviceID()) + "]");
    return false;
  }
  if (!phoenix_config_equality::IsEqual(config, readConfig)) {
    ds_report::Warning(std::string{"Configuration verification failed for "} + kind + " [" +
                       std::to_string(device.GetDeviceID()) + "]");
    return false;
  }
  return true;
}

template <typename Device, typename Config>
bool ApplyAndVerify(Device& device, const Config& config, int numTries, const char* kind) {
  for (int i = 0; i < numTries; i++) {
    if (frc::RobotBase::IsSimulation()) {
      return device.GetConfigurator().Apply(config).IsOK();
    }
    if (SpamGetStatusCode([&] { return device.GetConfigurator().Apply(config); })) {
      // API says we applied config, lets make sure it's right
      if (ReadAndVerify(device, config, kind)) {
        return true;
      }
      ds_report::Warning(std::string{"Failed to verify config for "} + kind + " [" +
                         std::to_string(device.GetDeviceID()) + "]" + Attempt(i, numTries));
    } else {
      ds_report::Warning(std::string{"Failed to apply config for "} + kind + " [" +
                         std::to_string(device.GetDeviceID()) + "]" + Attempt(i, numTries));
    }
  }
  ds_report::Error(std::string{"Failed to apply config for "} + kind + " after " +
                   std::to_string(numTries) + " attempts");
  return false;
}
}  // namespace

bool SpamGetStatusCode(const std::function<ctre::phoenix::StatusCode()>& function, int numTries) {
  ctre::phoenix::StatusCode code = function();
  int tries = 0;
  while (!code.IsOK() && tries < numTries) {
    ds_report::Warning(std::string{"Retrying CTRE Device Config "} + code.GetName());
    code = function();
    tries++;
  }
  if (!code.IsOK()) {
    ds_report::Error("Failed to execute phoenix pro api call after " + std::to_string(numTries) +
                     " attempts");
    return false;
  }
  return true;
}

bool ReadAndVerifyConfiguration(hw::TalonFX& talon, const cfg::TalonFXConfiguration& config) {
  return ReadAndVerify(talon, config, "talon");
}
bool ReadAndVerifyConfiguration(hw::CANcoder& cancoder, const cfg::CANcoderConfiguration& config) {
  return ReadAndVerify(cancoder, config, "CANCoder");
}
bool ReadAndVerifyConfiguration(hw::CANdi& candi, const cfg::CANdiConfiguration& config) {
  return ReadAndVerify(candi, config, "CANdi");
}
bool ReadAndVerifyConfiguration(hw::CANrange& canrange, const cfg::CANrangeConfiguration& config) {
  return ReadAndVerify(canrange, config, "CANrange");
}

bool ApplyMotorConfigs(hw::TalonFX& motor, const cfg::TalonFXConfiguration& config, int numTries) {
  return ApplyAndVerify(motor, config, numTries, "talon");
}
bool ApplyCancoderConfig(hw::CANcoder& cancoder, const cfg::CANcoderConfiguration& config,
                         int numTries) {
  return ApplyAndVerify(cancoder, config, numTries, "cancoder");
}
bool ApplyCANdiConfigs(hw::CANdi& candi, const cfg::CANdiConfiguration& config, int numTries) {
  return ApplyAndVerify(candi, config, numTries, "candi");
}
bool ApplyCanRangeConfigs(hw::CANrange& canrange, const cfg::CANrangeConfiguration& config,
                          int numTries) {
  return ApplyAndVerify(canrange, config, numTries, "canrange");
}

void RegisterSignals(bool canivore,
                     std::initializer_list<ctre::phoenix6::BaseStatusSignal*> signals) {
  auto& dst = canivore ? g_canivoreSignals : g_rioSignals;
  dst.insert(dst.end(), signals.begin(), signals.end());
}

void RefreshAll() {
  if (!g_canivoreSignals.empty()) {
    ctre::phoenix6::BaseStatusSignal::RefreshAll(g_canivoreSignals);
  }
  if (!g_rioSignals.empty()) {
    ctre::phoenix6::BaseStatusSignal::RefreshAll(g_rioSignals);
  }
}

size_t RegisteredSignalCount() { return g_canivoreSignals.size() + g_rioSignals.size(); }

void ClearRegisteredSignals() {
  g_canivoreSignals.clear();
  g_rioSignals.clear();
}

}  // namespace phoenix_util
