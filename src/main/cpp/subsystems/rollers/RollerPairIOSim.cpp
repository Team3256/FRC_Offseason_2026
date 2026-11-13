// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "subsystems/rollers/RollerPairIOSim.h"

#include <numbers>
#include <utility>

#include "sim/SimMechs.h"
#include "subsystems/feeder/FeederConstants.h"
#include "subsystems/indexer/IndexerConstants.h"
#include "subsystems/intakerollers/IntakeRollerConstants.h"

RollerPairIOSim::RollerPairIOSim(const RollerPairConfig& config, sim::Mechanism mechanism,
                                 Visualize visualize, double angularVelocityScalar)
    : RollerPairIOTalonFX(config),
      m_visualize{std::move(visualize)},
      m_angularVelocityScalar{angularVelocityScalar} {
  auto& world = sim::PhysicsWorld::Get();
  for (auto* motor : {&GetLeftMotor(), &GetRightMotor()}) {
    world.AttachMotor(mechanism, *motor, config.motorConfigs, mayhem::sim::DcMotor::KrakenX60(),
                      mayhem::sim::DcMotor::KrakenX60Foc());
  }
}

void RollerPairIOSim::UpdateInputs(RollerPairInputs& inputs) {
  const auto& world = sim::PhysicsWorld::Get();
  const auto& left = world.Motor(GetLeftMotor());
  const auto& right = world.Motor(GetRightMotor());
  inputs.leftVoltage = left.MotorVoltage();
  inputs.leftVelocity = left.Velocity();
  inputs.leftStatorCurrent = left.StatorCurrent();
  inputs.leftSupplyCurrent = left.SupplyCurrent();
  inputs.leftTemperature = 25;
  inputs.rightVoltage = right.MotorVoltage();
  inputs.rightVelocity = right.Velocity();
  inputs.rightStatorCurrent = right.StatorCurrent();
  inputs.rightSupplyCurrent = right.SupplyCurrent();
  inputs.rightTemperature = 25;
  m_lastRps = left.Velocity();
  if (m_visualize) {
    // (the Java sim fed rotations/s through Math.toDegrees; kept so the drawing looks the same)
    m_visualize(
        units::degree_t{m_lastRps * 180.0 / std::numbers::pi * 0.02 * m_angularVelocityScalar});
  }
}

std::unique_ptr<RollerPairIO> MakeIntakeRollersIO(bool simulation) {
  if (!simulation) {
    return std::make_unique<RollerPairIOTalonFX>(intake_rollers::IOConfig());
  }
  return std::make_unique<RollerPairIOSim>(
      intake_rollers::IOConfig(), ::sim::Mechanism::kIntakeRollers,
      [](units::degree_t d) { SimMechs::GetInstance().UpdateRollers(d); },
      intake_rollers::sim::kAngularVelocityScalar);
}

std::unique_ptr<RollerPairIO> MakeFeederIO(bool simulation) {
  if (!simulation) {
    return std::make_unique<RollerPairIOTalonFX>(feeder::IOConfig());
  }
  return std::make_unique<RollerPairIOSim>(
      feeder::IOConfig(), ::sim::Mechanism::kFeeder,
      [](units::degree_t d) { SimMechs::GetInstance().UpdateFeeder(d); },
      feeder::sim::kAngularVelocityScalar);
}

std::unique_ptr<RollerPairIO> MakeIndexerIO(bool simulation) {
  if (!simulation) {
    return std::make_unique<RollerPairIOTalonFX>(indexer::IOConfig());
  }
  return std::make_unique<RollerPairIOSim>(
      indexer::IOConfig(), ::sim::Mechanism::kIndexer,
      [](units::degree_t d) { SimMechs::GetInstance().UpdateIndexer(d); },
      indexer::sim::kAngularVelocityScalar);
}
