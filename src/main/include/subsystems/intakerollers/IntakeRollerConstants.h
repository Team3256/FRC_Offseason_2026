// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <units/frequency.h>
#include <ctre/phoenix6/TalonFX.hpp>

#include "subsystems/rollers/RollerPairIO.h"

namespace intake_rollers {

inline constexpr int kIntakeRollerMotorIDLeft = 19;
inline constexpr int kIntakeRollerMotorIDRight = 28;
inline constexpr units::hertz_t kUpdateFrequency = 50_Hz;
inline constexpr int kFlashConfigRetries = 5;

inline constexpr double kIntakeVoltage = 8.0;
inline constexpr double kUnjamVoltage = -4.0;
inline constexpr double kOuttakeVoltage = -8.0;

inline ctre::phoenix6::configs::TalonFXConfiguration MotorConfigs() {
  using namespace ctre::phoenix6;
  return configs::TalonFXConfiguration{}
      .WithSlot0(configs::Slot0Configs{}.WithKS(0).WithKV(0.12).WithKP(1).WithKI(0).WithKD(0))
      .WithMotorOutput(configs::MotorOutputConfigs{}
                           .WithNeutralMode(signals::NeutralModeValue::Coast)
                           .WithInverted(signals::InvertedValue::Clockwise_Positive))
      .WithCurrentLimits(configs::CurrentLimitsConfigs{}
                             .WithStatorCurrentLimitEnable(true)
                             .WithStatorCurrentLimit(80_A)
                             .WithSupplyCurrentLimit(50_A)
                             .WithSupplyCurrentLimitEnable(true)
                             .WithSupplyCurrentLowerLimit(0_A)
                             .WithSupplyCurrentLowerTime(0_s));
}

inline RollerPairConfig IOConfig() {
  return {kIntakeRollerMotorIDLeft, kIntakeRollerMotorIDRight, MotorConfigs(), kFlashConfigRetries,
          kUpdateFrequency};
}

inline constexpr RollerPairKeys kInputKeys = {
    "rollerVoltageLeft",       "rollerVelocityLeft",       "rollerStatorCurrentLeft",
    "rollerSupplyCurrentLeft", "rollerTemperatureLeft",    "rollerVoltageRight",
    "rollerVelocityRight",     "rollerStatorCurrentRight", "rollerSupplyCurrentRight",
    "rollerTemperatureRight"};

namespace sim {
inline constexpr double kRollerGearingRatio = 2.0;  // 2:1 gearing ratio from motor to roller
inline constexpr double kRollerMomentOfInertia = 10;
// Scale down the angular velocity so we can actually see what is happening
inline constexpr double kAngularVelocityScalar = 1000;
}  // namespace sim

}  // namespace intake_rollers
