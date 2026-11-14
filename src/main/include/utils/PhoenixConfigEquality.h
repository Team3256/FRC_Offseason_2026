// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <ctre/phoenix6/CANcoder.hpp>
#include <ctre/phoenix6/CANdi.hpp>
#include <ctre/phoenix6/CANrange.hpp>
#include <ctre/phoenix6/TalonFX.hpp>

/** Compares Phoenix configs read back from a device with the ones we applied (from 254). */
namespace phoenix_config_equality {

inline constexpr bool kEnableLoggingIneq = true;
inline constexpr double kTalonConfigEpsilon = 0.05;
inline constexpr double kCancoderConfigEpsilon = 0.05;
inline constexpr double kCandiConfigEpsilon = 0.05;
inline constexpr double kCanrangeConfigEpsilon = 0.05;

namespace cfg = ctre::phoenix6::configs;

bool IsEqual(const cfg::TalonFXConfiguration& a, const cfg::TalonFXConfiguration& b);
bool IsEqual(const cfg::CANrangeConfiguration& a, const cfg::CANrangeConfiguration& b);
bool IsEqual(const cfg::CANdiConfiguration& a, const cfg::CANdiConfiguration& b);
bool IsEqual(const cfg::CANcoderConfiguration& a, const cfg::CANcoderConfiguration& b);
bool IsEqual(const cfg::FovParamsConfigs& a, const cfg::FovParamsConfigs& b);
bool IsEqual(const cfg::PWM1Configs& a, const cfg::PWM1Configs& b);
bool IsEqual(const cfg::PWM2Configs& a, const cfg::PWM2Configs& b);
bool IsEqual(const cfg::ProximityParamsConfigs& a, const cfg::ProximityParamsConfigs& b);
bool IsEqual(const cfg::ToFParamsConfigs& a, const cfg::ToFParamsConfigs& b);
bool IsEqual(const cfg::QuadratureConfigs& a, const cfg::QuadratureConfigs& b);
bool IsEqual(const cfg::DigitalInputsConfigs& a, const cfg::DigitalInputsConfigs& b);
bool IsEqual(const cfg::MagnetSensorConfigs& a, const cfg::MagnetSensorConfigs& b);
bool IsEqual(const cfg::Slot0Configs& a, const cfg::Slot0Configs& b);
bool IsEqual(const cfg::Slot1Configs& a, const cfg::Slot1Configs& b);
bool IsEqual(const cfg::Slot2Configs& a, const cfg::Slot2Configs& b);
bool IsEqual(const cfg::MotorOutputConfigs& a, const cfg::MotorOutputConfigs& b);
bool IsEqual(const cfg::CurrentLimitsConfigs& a, const cfg::CurrentLimitsConfigs& b);
bool IsEqual(const cfg::VoltageConfigs& a, const cfg::VoltageConfigs& b);
bool IsEqual(const cfg::TorqueCurrentConfigs& a, const cfg::TorqueCurrentConfigs& b);
bool IsEqual(const cfg::FeedbackConfigs& a, const cfg::FeedbackConfigs& b);
bool IsEqual(const cfg::OpenLoopRampsConfigs& a, const cfg::OpenLoopRampsConfigs& b);
bool IsEqual(const cfg::ClosedLoopRampsConfigs& a, const cfg::ClosedLoopRampsConfigs& b);
bool IsEqual(const cfg::HardwareLimitSwitchConfigs& a, const cfg::HardwareLimitSwitchConfigs& b);
bool IsEqual(const cfg::AudioConfigs& a, const cfg::AudioConfigs& b);
bool IsEqual(const cfg::SoftwareLimitSwitchConfigs& a, const cfg::SoftwareLimitSwitchConfigs& b);
bool IsEqual(const cfg::MotionMagicConfigs& a, const cfg::MotionMagicConfigs& b);

}  // namespace phoenix_config_equality
