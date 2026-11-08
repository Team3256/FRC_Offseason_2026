// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <functional>
#include <memory>

#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/Trigger.h>

#include "logging/Logger.h"
#include "subsystems/shooter/ShooterIO.h"
#include "utils/DisableSubsystem.h"

class Shooter : public DisableSubsystem {
 public:
  Shooter(bool enabled, std::unique_ptr<ShooterIO> io);

  void Periodic() override;

  frc2::CommandPtr SetVoltage(double voltage);
  frc2::CommandPtr SetVelocity(double velocity);
  frc2::CommandPtr SetVelocity(std::function<double()> velocity);
  frc2::CommandPtr ShootHub(std::function<double()> distance,
                            std::function<double()> velMultiplier);
  frc2::CommandPtr FeedCorner(std::function<double()> distance);
  frc2::CommandPtr Off();

  /** Leader flywheel within 5 rot/s of the requested speed. */
  bool ReachedVelocity() const;
  double RequestedVelocity() const { return m_reqVelocity; }
  const ShooterIOInputs& Inputs() const { return m_inputs; }

  frc2::Trigger reachedVelocity{[this] { return ReachedVelocity(); }};

 private:
  std::unique_ptr<ShooterIO> m_io;
  ShooterIOInputs m_inputs;
  double m_reqVelocity = 0.0;
  logging::InputsLogger m_inputsLog{"Shooter"};
  logging::Output<double> m_reqVelocityLog{"Shooter/reqVelocity"};
};
