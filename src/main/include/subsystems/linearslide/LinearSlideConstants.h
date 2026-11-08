// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <numbers>

#include <units/frequency.h>
#include <units/length.h>
#include <units/mass.h>
#include <ctre/phoenix6/TalonFX.hpp>

namespace linear_slide {

inline constexpr int kRightMotorID = 39;
inline constexpr int kLeftMotorID = 40;
inline constexpr bool kUseFOC = true;
inline constexpr bool kUseMotionMagic = true;
inline constexpr int kFlashConfigRetries = 5;

inline constexpr double kStowPosition = 0;  // dk yet
inline constexpr double kIntakeInitialPosition = .86;
inline constexpr double kIntakeFinalPosition = .86;
inline constexpr double kJitterPosition = 0.7;         // dk yet
inline constexpr double kJitterIntermediate = 0.3745;  // dk yet
/** Both motors inside [0, kBetweenPosMax] means the slide is in the zone where the hood can hit it.
 */
inline constexpr double kBetweenPosMax = 0.3745;

inline constexpr units::hertz_t kUpdateFrequency = 50_Hz;

inline ctre::phoenix6::configs::TalonFXConfiguration RightMotorConfigs() {
  using namespace ctre::phoenix6;
  return configs::TalonFXConfiguration{}
      .WithSlot0(configs::Slot0Configs{}
                     .WithKS(0.39)
                     .WithKV(2.7)
                     .WithKP(20)
                     .WithKI(0)
                     .WithKD(0)
                     .WithKA(0)
                     .WithKG(0.3)
                     .WithGravityType(signals::GravityTypeValue::Elevator_Static))
      .WithSlot1(configs::Slot1Configs{}
                     .WithKS(0)
                     .WithKV(0)
                     .WithKP(0)
                     .WithKI(0)
                     .WithKD(0)
                     .WithKA(0)
                     .WithKG(0))
      .WithMotorOutput(configs::MotorOutputConfigs{}
                           .WithNeutralMode(signals::NeutralModeValue::Coast)
                           .WithInverted(signals::InvertedValue::Clockwise_Positive))
      .WithMotionMagic(configs::MotionMagicConfigs{}
                           .WithMotionMagicAcceleration(30_tr_per_s_sq)
                           .WithMotionMagicCruiseVelocity(4.3_tps)
                           .WithMotionMagicJerk(450_tr_per_s_cu)
                           .WithMotionMagicExpo_kV(0.11999999731779099_V / 1_tps)
                           .WithMotionMagicExpo_kA(0.1_V / 1_tr_per_s_sq))
      .WithCurrentLimits(configs::CurrentLimitsConfigs{}
                             .WithStatorCurrentLimitEnable(true)
                             .WithStatorCurrentLimit(15_A)
                             .WithSupplyCurrentLimit(50_A)
                             .WithSupplyCurrentLimitEnable(true)
                             .WithSupplyCurrentLowerTime(.1_s)
                             .WithSupplyCurrentLowerLimit(20_A))
      .WithFeedback(configs::FeedbackConfigs{}.WithSensorToMechanismRatio(30));
}

/** The left motor is mounted opposite the right one: same configs with the inversion flipped. */
inline ctre::phoenix6::configs::TalonFXConfiguration LeftMotorConfigs() {
  using namespace ctre::phoenix6;
  auto c = RightMotorConfigs();
  c.WithMotorOutput(configs::MotorOutputConfigs{}
                        .WithNeutralMode(signals::NeutralModeValue::Coast)
                        .WithInverted(signals::InvertedValue::CounterClockwise_Positive));
  return c;
}

namespace sim {
inline constexpr double kGearing = 30;
inline constexpr units::meter_t kLength = 10_in;
inline constexpr units::meter_t kDrumRadius = 3_in;
inline constexpr units::meter_t kMinLength = 1_in;
inline constexpr units::meter_t kMaxLength = 20_in;
inline constexpr units::kilogram_t kMass = 0.1_kg;
inline constexpr double kMoiKgM2 = 10;
inline constexpr units::meter_t kStartingHeight = kMinLength;

inline constexpr units::meter_t ConvertRotationsToMeters(double rotations) {
  return kDrumRadius * (2 * std::numbers::pi * rotations);
}
/** Note: like the Java helper, the argument is in inches. */
inline constexpr double ConvertMetersToRotations(double distInches) {
  return units::meter_t{units::inch_t{distInches}}.value() /
         (kDrumRadius.value() * (2 * std::numbers::pi));
}
}  // namespace sim

}  // namespace linear_slide
