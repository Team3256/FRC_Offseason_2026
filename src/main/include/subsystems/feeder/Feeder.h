// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <memory>

#include "logging/Logger.h"
#include "subsystems/rollers/RollerPairIO.h"
#include "utils/DisableSubsystem.h"

class Feeder : public DisableSubsystem {
 public:
  Feeder(bool enabled, std::unique_ptr<RollerPairIO> io);

  void Periodic() override;

  // Requests: call every loop the request should hold.
  void SetVoltage(double voltage);
  void SetVelocity(double velocity);
  void Off();
  void SetFeedVel();
  void Outtake();
  const RollerPairInputs& Inputs() const { return m_inputs; }

 private:
  std::unique_ptr<RollerPairIO> m_io;
  RollerPairInputs m_inputs;
  logging::InputsLogger m_inputsLog{"feeder"};
};
