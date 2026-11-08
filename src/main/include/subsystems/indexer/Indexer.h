// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <memory>

#include <frc2/command/CommandPtr.h>

#include "logging/Logger.h"
#include "subsystems/rollers/RollerPairIO.h"
#include "utils/DisableSubsystem.h"

class Indexer : public DisableSubsystem {
 public:
  Indexer(bool enabled, std::unique_ptr<RollerPairIO> io);

  void Periodic() override;

  frc2::CommandPtr SetVoltage(double voltage);
  frc2::CommandPtr SetVelocity(double velocity);
  frc2::CommandPtr Off();
  frc2::CommandPtr SetIndexVel();
  const RollerPairInputs& Inputs() const { return m_inputs; }

 private:
  std::unique_ptr<RollerPairIO> m_io;
  RollerPairInputs m_inputs;
  logging::InputsLogger m_inputsLog{"indexer"};
};
