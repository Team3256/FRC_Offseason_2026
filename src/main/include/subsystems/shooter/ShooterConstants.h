// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <array>

#include <units/frequency.h>
#include <ctre/phoenix6/TalonFX.hpp>

#include "utils/InterpolatingMap.h"

namespace shooter {

// field oriented control
inline constexpr bool kUseFOC = true;

inline constexpr int kNumMotors = 4;
/** CAN IDs; shooters[0] is the leader, the rest follow it. */
inline constexpr std::array<int, kNumMotors> kMotorIds = {
    44,  // top left
    45,  // bottom left
    42,  // top right
    41,  // bottom right
};
/** Which followers spin opposite the leader. */
inline constexpr std::array<bool, kNumMotors> kMotorOpposed = {false, true, false, true};

inline constexpr units::hertz_t kUpdateFrequency = 50_Hz;
inline constexpr int kFlashConfigRetries = 5;

inline ctre::phoenix6::configs::TalonFXConfiguration MotorConfigs() {
  using namespace ctre::phoenix6;
  return configs::TalonFXConfiguration{}
      .WithSlot0(
          configs::Slot0Configs{}.WithKS(0.304).WithKV(.12).WithKA(0).WithKP(.1).WithKI(0).WithKD(
              0))
      // For regenerative braking
      .WithMotorOutput(configs::MotorOutputConfigs{}
                           .WithNeutralMode(signals::NeutralModeValue::Coast)
                           .WithInverted(signals::InvertedValue::Clockwise_Positive))
      .WithMotionMagic(configs::MotionMagicConfigs{}.WithMotionMagicAcceleration(800_tr_per_s_sq))
      .WithCurrentLimits(configs::CurrentLimitsConfigs{}
                             .WithStatorCurrentLimitEnable(true)
                             .WithStatorCurrentLimit(80_A)
                             .WithStatorCurrentLimitEnable(true)
                             .WithSupplyCurrentLimit(50_A)
                             .WithSupplyCurrentLowerLimit(40_A)
                             .WithSupplyCurrentLowerTime(0.1_s));
}

/** Flywheel speed [rot/s] by distance to the hub [m]. */
inline constexpr util::InterpolatingMap<5> kHubLUT{{{
    {1.62, 65.8},
    {2.71, 74.8},
    {3.15, 79.8},
    {3.95, 84.4},
    {4.92, 100.0},
}}};

/** Flywheel speed [rot/s] by distance to the corner feed target [m]. */
inline constexpr util::InterpolatingMap<10> kFeedLUT{{{
    {4.38, 35.6},
    {6.11, 39.6},
    {8.0, 44.4},
    {9.42, 53.4},
    {10.8, 59.6},
    {12.834, 75.2},
    {14.0, 80.0},
    {15.0, 85.0},
    {16.0, 90.0},
    {17.0, 95.0},
}}};

namespace sim {
inline constexpr double kLeftGearingRatio = 1;         // TODO: Update this value
inline constexpr double kLeftMomentOfInertia = 0.001;  // TODO: Update this value
inline constexpr double kAngularVelocityScalar = .03;
}  // namespace sim

}  // namespace shooter
