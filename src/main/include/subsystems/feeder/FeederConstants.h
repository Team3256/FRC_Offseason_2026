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

namespace feeder {

inline constexpr bool kUseFOC = true;
inline constexpr int kFeederMotorLeftID = 47;
inline constexpr int kFeederMotorRightID = 43;
inline constexpr double kFeederVelocity = 60;
inline constexpr double kOuttakeVelocity = -80;
inline constexpr units::hertz_t kUpdateFrequency = 50_Hz;
inline constexpr int kFlashConfigRetries = 5;

inline ctre::phoenix6::configs::TalonFXConfiguration MotorConfigs() {
  using namespace ctre::phoenix6;
  return configs::TalonFXConfiguration{}
      .WithSlot0(configs::Slot0Configs{}.WithKS(0.0).WithKV(0.13).WithKP(0.3).WithKI(0).WithKD(0))
      .WithMotorOutput(configs::MotorOutputConfigs{}
                           .WithNeutralMode(signals::NeutralModeValue::Coast)
                           .WithInverted(signals::InvertedValue::Clockwise_Positive))
      .WithCurrentLimits(configs::CurrentLimitsConfigs{}
                             .WithStatorCurrentLimitEnable(true)
                             .WithStatorCurrentLimit(80_A)
                             .WithSupplyCurrentLimit(45_A)
                             .WithSupplyCurrentLimitEnable(true)
                             .WithSupplyCurrentLowerLimit(30_A)
                             .WithSupplyCurrentLowerTime(0.1_s));
}

inline RollerPairConfig IOConfig() {
  return {kFeederMotorLeftID, kFeederMotorRightID, MotorConfigs(), kFlashConfigRetries,
          kUpdateFrequency};
}

inline constexpr RollerPairKeys kInputKeys = {
    "feederMotorLeftVoltage",        "feederMotorLeftVelocity",
    "feederMotorLeftStatorCurrent",  "feederMotorLeftSupplyCurrent",
    "feederMotorLeftTemperature",    "feederMotorRightVoltage",
    "feederMotorRightVelocity",      "feederMotorRightStatorCurrent",
    "feederMotorRightSupplyCurrent", "feederMotorRightTemperature"};

namespace sim {
inline constexpr double kRollerGearingRatio = 1.0;
inline constexpr double kRollerMomentOfInertia = 0.03;
inline constexpr double kAngularVelocityScalar = 1;
}  // namespace sim

}  // namespace feeder
