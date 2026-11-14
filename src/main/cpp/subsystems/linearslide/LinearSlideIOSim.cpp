// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "subsystems/linearslide/LinearSlideIOSim.h"

#include "sim/PhysicsWorld.h"
#include "sim/SimMechs.h"

LinearSlideIOSim::LinearSlideIOSim() {
  auto& world = ::sim::PhysicsWorld::Get();
  world.AttachMotor(::sim::Mechanism::kSlide, GetRightMotor(), linear_slide::RightMotorConfigs(),
                    mayhem::sim::DcMotor::KrakenX44(), mayhem::sim::DcMotor::KrakenX44Foc());
  world.AttachMotor(::sim::Mechanism::kSlide, GetLeftMotor(), linear_slide::LeftMotorConfigs(),
                    mayhem::sim::DcMotor::KrakenX44(), mayhem::sim::DcMotor::KrakenX44Foc());
}

void LinearSlideIOSim::UpdateInputs(LinearSlideIOInputs& inputs) {
  const auto& world = ::sim::PhysicsWorld::Get();
  const auto& right = world.Motor(GetRightMotor());
  const auto& left = world.Motor(GetLeftMotor());
  inputs.rightMotorVoltage = right.MotorVoltage();
  inputs.rightMotorVelocity = right.Velocity();
  inputs.rightMotorPosition = right.Position();
  inputs.rightMotorStatorCurrent = right.StatorCurrent();
  inputs.rightMotorSupplyCurrent = right.SupplyCurrent();
  inputs.leftMotorVoltage = left.MotorVoltage();
  inputs.leftMotorVelocity = left.Velocity();
  inputs.leftMotorPosition = left.Position();
  inputs.leftMotorStatorCurrent = left.StatorCurrent();
  inputs.leftMotorSupplyCurrent = left.SupplyCurrent();
  m_positionLog.Set(world.SlideExtension());
  SimMechs::GetInstance().UpdateLinearSlide(units::meter_t{world.SlideExtension()});
}

void LinearSlideIOSim::ResetPosition(double angle) {
  LinearSlideIOTalonFX::ResetPosition(angle);
  ::sim::PhysicsWorld::Get().SetMechanismPosition(GetRightMotor(), angle);
  ::sim::PhysicsWorld::Get().SetMechanismPosition(GetLeftMotor(), angle);
}

void LinearSlideIOSim::Zero() {
  LinearSlideIOTalonFX::Zero();
  ::sim::PhysicsWorld::Get().SetMechanismPosition(GetRightMotor(), 0);
  ::sim::PhysicsWorld::Get().SetMechanismPosition(GetLeftMotor(), 0);
}
