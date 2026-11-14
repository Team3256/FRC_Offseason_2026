// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "subsystems/shooterpivot/ShooterPivotIOSim.h"

#include "sim/PhysicsWorld.h"
#include "sim/SimMechs.h"

ShooterPivotIOSim::ShooterPivotIOSim() {
  sim::PhysicsWorld::Get().AttachMotor(
      sim::Mechanism::kHood, GetMotor(), shooter_pivot::MotorConfigs(),
      mayhem::sim::DcMotor::KrakenX44(), mayhem::sim::DcMotor::KrakenX44Foc());
}

void ShooterPivotIOSim::UpdateInputs(ShooterPivotIOInputs& inputs) {
  const auto& m = sim::PhysicsWorld::Get().Motor(GetMotor());
  inputs.shooterPivotMotorVoltage = m.MotorVoltage();
  inputs.shooterPivotMotorVelocity = m.Velocity();
  inputs.shooterPivotMotorPosition = m.Position();
  inputs.shooterPivotMotorStatorCurrent = m.StatorCurrent();
  inputs.shooterPivotMotorSupplyCurrent = m.SupplyCurrent();
  SimMechs::GetInstance().UpdateShooterPivot(units::radian_t{SimAngleRadians()});
}

void ShooterPivotIOSim::ResetPosition(double angle) {
  ShooterPivotIOTalonFX::ResetPosition(angle);
  sim::PhysicsWorld::Get().SetMechanismPosition(GetMotor(), angle);
}

double ShooterPivotIOSim::SimAngleRadians() const { return sim::PhysicsWorld::Get().HoodAngle(); }
