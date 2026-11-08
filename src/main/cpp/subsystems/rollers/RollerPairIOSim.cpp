// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "subsystems/rollers/RollerPairIOSim.h"

#include <memory>
#include <numbers>

#include <frc/RobotController.h>
#include <frc/TimedRobot.h>
#include <frc/system/plant/DCMotor.h>
#include <frc/system/plant/LinearSystemId.h>

#include "sim/SimBattery.h"
#include "sim/SimMechs.h"
#include "subsystems/feeder/FeederConstants.h"
#include "subsystems/indexer/IndexerConstants.h"
#include "subsystems/intakerollers/IntakeRollerConstants.h"

RollerPairIOSim::RollerPairIOSim(const RollerPairConfig& config,
                                 const frc::LinearSystem<1, 1, 1>& plant,
                                 const frc::DCMotor& gearbox, Options options)
    : RollerPairIOTalonFX(config), m_sim{plant, gearbox}, m_options{std::move(options)} {}

void RollerPairIOSim::UpdateInputs(RollerPairInputs& inputs) {
  constexpr double dt = frc::TimedRobot::kDefaultPeriod.value();
  auto& left = GetLeftMotor().GetSimState();
  auto& right = GetRightMotor().GetSimState();
  left.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
  if (m_options.setFollowerSupply) {
    right.SetSupplyVoltage(frc::RobotController::GetBatteryVoltage());
  }

  // Drive the physics model off the leader's commanded voltage
  m_sim.SetInput(frc::Vectord<1>{left.GetMotorVoltage().value()});
  m_sim.Update(frc::TimedRobot::kDefaultPeriod);
  const double motorRps = units::turns_per_second_t{m_sim.GetAngularVelocity()}.value();
  m_lastRps = motorRps;

  left.SetRotorVelocity(units::turns_per_second_t{motorRps});
  left.AddRotorPosition(units::turn_t{motorRps * dt});
  const double followerRps = m_options.followerReversed ? -motorRps : motorRps;
  right.SetRotorVelocity(units::turns_per_second_t{followerRps});
  right.AddRotorPosition(units::turn_t{followerRps * dt});

  sim_battery::AddCurrent(left.GetSupplyCurrent() + right.GetSupplyCurrent());

  RollerPairIOTalonFX::UpdateInputs(inputs);
  if (m_options.visualize) {
    // (the Java sim fed rotations/s through Math.toDegrees; kept so the drawing looks the same)
    m_options.visualize(units::degree_t{motorRps * 180.0 / std::numbers::pi * dt *
                                        m_options.angularVelocityScalar});
  }
}

std::unique_ptr<RollerPairIO> MakeIntakeRollersIO(bool simulation) {
  using namespace intake_rollers;
  if (!simulation) {
    return std::make_unique<RollerPairIOTalonFX>(IOConfig());
  }
  const auto motor = frc::DCMotor::KrakenX60(2);
  // Same arguments as the Java sim (which passed the gearing as the inertia and vice versa).
  const auto plant = frc::LinearSystemId::FlywheelSystem(
      motor, units::kilogram_square_meter_t{sim::kRollerGearingRatio}, sim::kRollerMomentOfInertia);
  return std::make_unique<RollerPairIOSim>(
      IOConfig(), plant, motor,
      RollerPairIOSim::Options{
          .followerReversed = true,
          .setFollowerSupply = true,
          .visualize = [](units::degree_t d) { SimMechs::GetInstance().UpdateRollers(d); },
          .angularVelocityScalar = sim::kAngularVelocityScalar});
}

std::unique_ptr<RollerPairIO> MakeFeederIO(bool simulation) {
  using namespace feeder;
  if (!simulation) {
    return std::make_unique<RollerPairIOTalonFX>(IOConfig());
  }
  const auto motor = kUseFOC ? frc::DCMotor::KrakenX60FOC(2) : frc::DCMotor::KrakenX60(2);
  const auto plant = frc::LinearSystemId::FlywheelSystem(
      motor, units::kilogram_square_meter_t{sim::kRollerMomentOfInertia}, sim::kRollerGearingRatio);
  return std::make_unique<RollerPairIOSim>(
      IOConfig(), plant, motor,
      RollerPairIOSim::Options{
          .followerReversed = false,
          .setFollowerSupply = true,
          .visualize = [](units::degree_t d) { SimMechs::GetInstance().UpdateFeeder(d); },
          .angularVelocityScalar = sim::kAngularVelocityScalar});
}

std::unique_ptr<RollerPairIO> MakeIndexerIO(bool simulation) {
  using namespace indexer;
  if (!simulation) {
    return std::make_unique<RollerPairIOTalonFX>(IOConfig());
  }
  const auto motor = kUseFOC ? frc::DCMotor::KrakenX60FOC(2) : frc::DCMotor::KrakenX60(2);
  const auto plant = frc::LinearSystemId::FlywheelSystem(
      motor, units::kilogram_square_meter_t{sim::kIndexerMomentOfInertia},
      sim::kIndexerGearingRatio);
  return std::make_unique<RollerPairIOSim>(
      IOConfig(), plant, motor,
      RollerPairIOSim::Options{
          .followerReversed = false,
          .setFollowerSupply = false,
          .visualize = [](units::degree_t d) { SimMechs::GetInstance().UpdateIndexer(d); },
          .angularVelocityScalar = sim::kAngularVelocityScalar});
}
