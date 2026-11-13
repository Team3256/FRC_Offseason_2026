// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <memory>

#include "logging/Logger.h"
#include "subsystems/shooter/ShooterIO.h"
#include "utils/DisableSubsystem.h"

class Shooter : public DisableSubsystem {
 public:
  Shooter(bool enabled, std::unique_ptr<ShooterIO> io);

  void Periodic() override;

  // Requests: call every loop the request should hold.
  void SetVoltage(double voltage);
  void SetVelocity(double velocity);
  /** Hub shot: speed from the hub table at {@code distance} [m], times {@code velMultiplier}. */
  void ShootHub(double distance, double velMultiplier);
  /** Feed shot to a corner: speed from the feed table at {@code distance} [m]. */
  void FeedCorner(double distance);
  void Off();

  /** Leader flywheel within 5 rot/s of the requested speed. */
  bool ReachedVelocity() const;
  double RequestedVelocity() const { return m_reqVelocity; }
  const ShooterIOInputs& Inputs() const { return m_inputs; }

 private:
  std::unique_ptr<ShooterIO> m_io;
  ShooterIOInputs m_inputs;
  double m_reqVelocity = 0.0;
  logging::InputsLogger m_inputsLog{"Shooter"};
  logging::Output<double> m_reqVelocityLog{"Shooter/reqVelocity"};
};
