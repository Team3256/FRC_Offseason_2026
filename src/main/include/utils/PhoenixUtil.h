// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <functional>
#include <initializer_list>

#include <ctre/phoenix/StatusCodes.h>
#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/CANdi.hpp>
#include <ctre/phoenix6/CANrange.hpp>
#include <ctre/phoenix6/StatusSignal.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

namespace phoenix_util {

namespace hw = ctre::phoenix6::hardware;
namespace cfg = ctre::phoenix6::configs;

/** Retries a Phoenix call until it returns OK or runs out of attempts. */
bool SpamGetStatusCode(const std::function<ctre::phoenix::StatusCode()>& function,
                       int numTries = 5);

bool ReadAndVerifyConfiguration(hw::TalonFX& talon, const cfg::TalonFXConfiguration& config);
bool ReadAndVerifyConfiguration(hw::CANcoder& cancoder, const cfg::CANcoderConfiguration& config);
bool ReadAndVerifyConfiguration(hw::CANdi& candi, const cfg::CANdiConfiguration& config);
bool ReadAndVerifyConfiguration(hw::CANrange& canrange, const cfg::CANrangeConfiguration& config);

/** Applies a config, reads it back and verifies it, retrying up to {@code numTries} times. */
bool ApplyMotorConfigs(hw::TalonFX& motor, const cfg::TalonFXConfiguration& config, int numTries);
bool ApplyCancoderConfig(hw::CANcoder& cancoder, const cfg::CANcoderConfiguration& config,
                         int numTries);
bool ApplyCANdiConfigs(hw::CANdi& candi, const cfg::CANdiConfiguration& config, int numTries);
bool ApplyCanRangeConfigs(hw::CANrange& canrange, const cfg::CANrangeConfiguration& config,
                          int numTries);

/**
 * Registers signals to be refreshed together once per loop by RefreshAll(), which is cheaper than
 * refreshing each signal on its own (one synchronized batch per CAN bus).
 */
void RegisterSignals(bool canivore,
                     std::initializer_list<ctre::phoenix6::BaseStatusSignal*> signals);

/** Refreshes every registered signal. Call once at the start of each robot loop. */
void RefreshAll();

/** Number of registered signals (diagnostics/tests). */
size_t RegisteredSignalCount();

/** Forgets every registered signal (tests). */
void ClearRegisteredSignals();

}  // namespace phoenix_util
