// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "subsystems/shooter/ShooterIOSim.h"

#include <frc/RobotController.h>
#include <frc/TimedRobot.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/LinearSystemId.h>

#include "sim/SimBattery.h"
#include "sim/SimMechs.h"

namespace {
const frc::DCMotor kMotor =
    shooter::kUseFOC ? frc::DCMotor::KrakenX60FOC(4) : frc::DCMotor::KrakenX60(4);
}

ShooterIOSim::ShooterIOSim()
    : m_flywheelSim{frc::LinearSystemId::FlywheelSystem(
                        kMotor, units::kilogram_square_meter_t{shooter::sim::kLeftMomentOfInertia},
                        shooter::sim::kLeftGearingRatio),
                    kMotor} {}

void ShooterIOSim::UpdateInputs(ShooterIOInputs& inputs) {
  constexpr double dt = frc::TimedRobot::kDefaultPeriod.value();
  auto& motors = GetMotors();
  for (auto& m : motors) {
    m.GetSimState().SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
  }
  // Update physics models
  m_flywheelSim.SetInputVoltage(motors[0].GetSimState().GetMotorVoltage());
  m_flywheelSim.Update(frc::TimedRobot::kDefaultPeriod);

  const double motorRps = units::turns_per_second_t{m_flywheelSim.GetAngularVelocity()}.value();
  for (auto& m : motors) {
    auto& simState = m.GetSimState();
    simState.SetRotorVelocity(units::turns_per_second_t{motorRps});
    simState.AddRotorPosition(units::turn_t{motorRps * dt});
  }
  // Battery load: the simulated TalonFXs' supply currents (these include the current limits).
  for (auto& m : motors) {
    sim_battery::AddCurrent(m.GetSimState().GetSupplyCurrent());
  }

  ShooterIOTalonFX::UpdateInputs(inputs);
  SimMechs::GetInstance().UpdateShooterWheel(
      units::degree_t{motorRps * 360 * dt * shooter::sim::kAngularVelocityScalar});
}
