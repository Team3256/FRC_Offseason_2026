// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "subsystems/linearslide/LinearSlideIOSim.h"

#include <numbers>

#include <frc/RobotController.h>
#include <frc/TimedRobot.h>
#include <frc/system/plant/DCMotor.h>

#include "sim/SimBattery.h"
#include "sim/SimMechs.h"

using namespace linear_slide;

LinearSlideIOSim::LinearSlideIOSim()
    : m_slideSim{frc::DCMotor::KrakenX44(2),
                 sim::kGearing,
                 sim::kMass,
                 sim::kDrumRadius,
                 sim::kMinLength,
                 sim::kMaxLength,
                 false,
                 sim::kStartingHeight} {
  GetRightMotor().GetSimState().Orientation =
      ctre::phoenix6::sim::ChassisReference::Clockwise_Positive;
  GetLeftMotor().GetSimState().Orientation =
      ctre::phoenix6::sim::ChassisReference::CounterClockwise_Positive;
}

void LinearSlideIOSim::UpdateInputs(LinearSlideIOInputs& inputs) {
  auto& right = GetRightMotor().GetSimState();
  auto& left = GetLeftMotor().GetSimState();
  right.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
  left.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());

  m_slideSim.SetInputVoltage((right.GetMotorVoltage() + left.GetMotorVoltage()) / 2.0);
  m_slideSim.Update(frc::TimedRobot::kDefaultPeriod);

  const double rotorRotationsPerMeter =
      sim::kGearing / (2.0 * std::numbers::pi * sim::kDrumRadius.value());
  const double rotorPosition = m_slideSim.GetPosition().value() * rotorRotationsPerMeter;
  const double rotorVelocity = m_slideSim.GetVelocity().value() * rotorRotationsPerMeter;
  right.SetRawRotorPosition(units::turn_t{rotorPosition});
  right.SetRotorVelocity(units::turns_per_second_t{rotorVelocity});
  left.SetRawRotorPosition(units::turn_t{rotorPosition});
  left.SetRotorVelocity(units::turns_per_second_t{rotorVelocity});
  sim_battery::AddCurrent(right.GetSupplyCurrent() + left.GetSupplyCurrent());

  LinearSlideIOTalonFX::UpdateInputs(inputs);
  m_positionLog.Set(m_slideSim.GetPosition().value());
  m_velocityLog.Set(m_slideSim.GetVelocity().value());
  SimMechs::GetInstance().UpdateLinearSlide(m_slideSim.GetPosition());
}
