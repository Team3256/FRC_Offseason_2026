// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "subsystems/shooter/ShooterIOSim.h"

#include "sim/PhysicsWorld.h"
#include "sim/SimMechs.h"

ShooterIOSim::ShooterIOSim() {
  auto& world = sim::PhysicsWorld::Get();
  for (auto& m : GetMotors()) {
    world.AttachMotor(sim::Mechanism::kShooter, m, shooter::MotorConfigs(),
                      mayhem::sim::DcMotor::KrakenX60(), mayhem::sim::DcMotor::KrakenX60Foc());
  }
}

void ShooterIOSim::UpdateInputs(ShooterIOInputs& inputs) {
  const auto& world = sim::PhysicsWorld::Get();
  auto& motors = GetMotors();
  for (size_t i = 0; i < motors.size(); i++) {
    const auto& m = world.Motor(motors[i]);
    inputs.shooterMotorVoltages[i] = m.MotorVoltage();
    inputs.shooterMotorVelocitys[i] = m.Velocity();
    inputs.shooterMotorStatorCurrents[i] = m.StatorCurrent();
    inputs.shooterMotorSupplyCurrents[i] = m.SupplyCurrent();
    inputs.shooterMotorTemperatures[i] = 25;
    inputs.shooterMotorIDs[i] = shooter::kMotorIds[i];
  }
  SimMechs::GetInstance().UpdateShooterWheel(units::degree_t{
      inputs.shooterMotorVelocitys[0] * 360 * 0.02 * shooter::sim::kAngularVelocityScalar});
}
