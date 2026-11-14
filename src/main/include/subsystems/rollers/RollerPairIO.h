// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <array>
#include <string_view>

#include <units/frequency.h>
#include <ctre/phoenix6/TalonFX.hpp>
#include <ctre/phoenix6/controls/Follower.hpp>
#include <ctre/phoenix6/controls/VelocityVoltage.hpp>

#include "logging/Logger.h"

/** Inputs of a leader ("left") + opposed follower ("right") TalonFX pair. */
struct RollerPairInputs {
  double leftVoltage = 0.0;
  double leftVelocity = 0.0;
  double leftStatorCurrent = 0.0;
  double leftSupplyCurrent = 0.0;
  double leftTemperature = 0.0;
  double rightVoltage = 0.0;
  double rightVelocity = 0.0;
  double rightStatorCurrent = 0.0;
  double rightSupplyCurrent = 0.0;
  double rightTemperature = 0.0;
};

/** Log key for each RollerPairInputs field, in declaration order (kept from the Java IO classes).
 */
using RollerPairKeys = std::array<std::string_view, 10>;

/** Registers every RollerPairInputs field with an InputsLogger under the given names. */
void AddRollerPairInputs(logging::InputsLogger& log, const RollerPairInputs& inputs,
                         const RollerPairKeys& keys);

/** IO for intake rollers, feeder and indexer: two TalonFXs, the right one following the left. */
class RollerPairIO {
 public:
  virtual ~RollerPairIO() = default;
  virtual void UpdateInputs(RollerPairInputs& inputs) {}
  virtual void SetVoltage(double voltage) {}
  virtual void SetVelocity(double velocity) {}
  virtual void Off() {}
};

struct RollerPairConfig {
  int leftId;
  int rightId;
  ctre::phoenix6::configs::TalonFXConfiguration motorConfigs;
  int flashConfigRetries;
  units::hertz_t updateFrequency;
  /** Name for dashboard alerts, e.g. "Feeder". */
  const char* name = "Rollers";
};

class RollerPairIOTalonFX : public RollerPairIO {
 public:
  explicit RollerPairIOTalonFX(const RollerPairConfig& config);
  void UpdateInputs(RollerPairInputs& inputs) override;
  void SetVoltage(double voltage) override;
  void SetVelocity(double velocity) override;
  void Off() override;

  ctre::phoenix6::hardware::TalonFX& GetLeftMotor() { return m_left; }
  ctre::phoenix6::hardware::TalonFX& GetRightMotor() { return m_right; }

 private:
  ctre::phoenix6::hardware::TalonFX m_left;
  ctre::phoenix6::hardware::TalonFX m_right;
  ctre::phoenix6::controls::VelocityVoltage m_velocityRequest =
      ctre::phoenix6::controls::VelocityVoltage{0_tps}.WithSlot(0);
  ctre::phoenix6::controls::Follower m_followerRequest;

  ctre::phoenix6::StatusSignal<units::volt_t>& m_leftVoltage;
  ctre::phoenix6::StatusSignal<units::turns_per_second_t>& m_leftVelocity;
  ctre::phoenix6::StatusSignal<units::ampere_t>& m_leftStator;
  ctre::phoenix6::StatusSignal<units::ampere_t>& m_leftSupply;
  ctre::phoenix6::StatusSignal<units::celsius_t>& m_leftTemp;
  ctre::phoenix6::StatusSignal<units::volt_t>& m_rightVoltage;
  ctre::phoenix6::StatusSignal<units::turns_per_second_t>& m_rightVelocity;
  ctre::phoenix6::StatusSignal<units::ampere_t>& m_rightStator;
  ctre::phoenix6::StatusSignal<units::ampere_t>& m_rightSupply;
  ctre::phoenix6::StatusSignal<units::celsius_t>& m_rightTemp;
};
