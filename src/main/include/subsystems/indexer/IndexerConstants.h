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

namespace indexer {

inline constexpr bool kUseFOC = true;
// Keep these out of 1-8 (swerve) and unique: a second TalonFX on an ID (the right motor used to
// share ID 4 with the swerve front right drive motor) makes the sim wheel readback flip and the
// pose jump around
inline constexpr int kIndexerMotorLeft = 46;
inline constexpr int kIndexerMotorRight = 27;
inline constexpr units::hertz_t kUpdateFrequency = 50_Hz;
inline constexpr double kIndexerVelocity = 60;
inline constexpr int kFlashConfigRetries = 5;

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
                             .WithSupplyCurrentLowerLimit(30_A)
                             .WithSupplyCurrentLowerTime(0.1_s));
}

inline RollerPairConfig IOConfig() {
  return {kIndexerMotorLeft, kIndexerMotorRight, MotorConfigs(), kFlashConfigRetries,
          kUpdateFrequency};
}

inline constexpr RollerPairKeys kInputKeys = {
    "indexerVoltageLeft",       "indexerVelocityLeft",       "indexerStatorCurrentLeft",
    "indexerSupplyCurrentLeft", "indexerTemperatureLeft",    "indexerVoltageRight",
    "indexerVelocityRight",     "indexerStatorCurrentRight", "indexerSupplyCurrentRight",
    "indexerTemperatureRight"};

namespace sim {
inline constexpr double kIndexerGearingRatio = 1.5;
inline constexpr double kIndexerMomentOfInertia = 0.05;
inline constexpr double kAngularVelocityScalar = 5;
}  // namespace sim

}  // namespace indexer
