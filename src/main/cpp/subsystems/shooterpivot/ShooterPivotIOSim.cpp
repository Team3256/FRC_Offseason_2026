// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "subsystems/shooterpivot/ShooterPivotIOSim.h"

#include <numbers>

#include <frc/RobotController.h>
#include <frc/TimedRobot.h>
#include <frc/system/plant/DCMotor.h>

#include "sim/SimBattery.h"
#include "sim/SimMechs.h"

using namespace shooter_pivot;

ShooterPivotIOSim::ShooterPivotIOSim()
    : m_pivotSim{frc::DCMotor::KrakenX44(1),
                 sim::kGearing,
                 units::kilogram_square_meter_t{sim::kMoiKgM2},
                 sim::kLength,
                 sim::kMinAngle,
                 sim::kMaxAngle,
                 false,
                 sim::kStartingAngle} {
  GetMotor().GetSimState().Orientation = ctre::phoenix6::sim::ChassisReference::Clockwise_Positive;
}

void ShooterPivotIOSim::UpdateInputs(ShooterPivotIOInputs& inputs) {
  auto& simState = GetMotor().GetSimState();
  simState.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
  m_pivotSim.SetInputVoltage(simState.GetMotorVoltage());
  m_pivotSim.Update(frc::TimedRobot::kDefaultPeriod);

  // Arm rotations times the gearing at the rotor (see sim::kGearing).
  constexpr double kRadToRot = 1.0 / (2.0 * std::numbers::pi);
  simState.SetRawRotorPosition(
      units::turn_t{m_pivotSim.GetAngle().value() * kRadToRot * sim::kGearing});
  simState.SetRotorVelocity(
      units::turns_per_second_t{m_pivotSim.GetVelocity().value() * kRadToRot * sim::kGearing});
  sim_battery::AddCurrent(simState.GetSupplyCurrent());

  ShooterPivotIOTalonFX::UpdateInputs(inputs);
  SimMechs::GetInstance().UpdateShooterPivot(m_pivotSim.GetAngle());
}
