// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <units/angle.h>
#include <units/frequency.h>
#include <units/length.h>
#include <units/mass.h>
#include <ctre/phoenix6/TalonFX.hpp>

#include "utils/InterpolatingMap.h"

namespace shooter_pivot {

inline constexpr int kPivotMotorId = 51;
inline constexpr bool kUseFOC = false;
inline constexpr bool kUseMotionMagic = false;
inline constexpr int kFlashConfigRetries = 5;
inline constexpr units::hertz_t kUpdateFrequency = 50_Hz;

inline ctre::phoenix6::configs::TalonFXConfiguration MotorConfigs() {
  using namespace ctre::phoenix6;
  return configs::TalonFXConfiguration{}
      .WithSlot0(configs::Slot0Configs{}
                     .WithKS(0)
                     .WithKV(2.7)
                     .WithKP(600)
                     .WithKI(0)
                     .WithKD(7)
                     .WithKA(0)
                     .WithKG(0)
                     .WithGravityType(signals::GravityTypeValue::Arm_Cosine))
      .WithMotorOutput(configs::MotorOutputConfigs{}
                           .WithNeutralMode(signals::NeutralModeValue::Brake)
                           .WithInverted(signals::InvertedValue::Clockwise_Positive))
      .WithMotionMagic(configs::MotionMagicConfigs{}
                           .WithMotionMagicAcceleration(6.2_tr_per_s_sq)
                           .WithMotionMagicCruiseVelocity(1.304_tps)
                           .WithMotionMagicExpo_kV(0.119999_V / 1_tps)
                           .WithMotionMagicExpo_kA(0.1_V / 1_tr_per_s_sq))
      .WithCurrentLimits(configs::CurrentLimitsConfigs{}
                             .WithStatorCurrentLimitEnable(true)
                             .WithStatorCurrentLimit(70_A)
                             .WithSupplyCurrentLimit(50_A)
                             .WithSupplyCurrentLimitEnable(true)
                             .WithSupplyCurrentLowerLimit(20_A)
                             .WithSupplyCurrentLowerTime(0.1_s))
      .WithFeedback(configs::FeedbackConfigs{}.WithSensorToMechanismRatio(30));
}

/** Hood position [mechanism rotations] by distance to the hub [m]. */
inline constexpr util::InterpolatingMap<5> kHubLUT{{{
    {1.62, 0.031},
    {2.71, 0.054},
    {3.15, 0.064},
    {3.95, 0.069},
    {4.92, 0.071},
}}};

/** Hood position [mechanism rotations] by distance to the corner feed target [m]. */
inline constexpr util::InterpolatingMap<6> kFeedLUT{{{
    {4.38, 0.01},
    {6.11, 0.01},
    {8.0, 0.01},
    {9.42, 0.01},
    {10.8, 0.01},
    {12.834, 0.01},
}}};

namespace sim {
/**
 * Motor rotations per hood rotation in the physics model. The Java sim used 4 here while the
 * TalonFX config uses SensorToMechanismRatio = 30, so in simulation the hood could never get past
 * ~0.016 mechanism rotations (0.05 home / 0.07 intake were unreachable and INTAKE waited forever).
 * Using the configured ratio keeps the simulated hood consistent with the real one.
 */
inline constexpr double kGearing = 30;
/** The value the Java simulation used (kept for reference). */
inline constexpr double kJavaSimGearing = 4;
inline constexpr units::meter_t kLength = 11.083714_in;
inline constexpr units::kilogram_t kMass = 0.01_kg;
inline constexpr double kMoiKgM2 = 1;
inline constexpr units::degree_t kMinAngle = 0_deg;
inline constexpr units::degree_t kMaxAngle = 42.152_deg;
inline constexpr units::degree_t kStartingAngle = 0_deg;
}  // namespace sim

}  // namespace shooter_pivot
