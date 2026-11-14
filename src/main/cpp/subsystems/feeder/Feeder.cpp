// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "subsystems/feeder/Feeder.h"

#include "logging/LoggedTracer.h"
#include "subsystems/feeder/FeederConstants.h"

Feeder::Feeder(bool enabled, std::unique_ptr<RollerPairIO> io)
    : DisableSubsystem(enabled, "Feeder"), m_io{std::move(io)} {
  AddRollerPairInputs(m_inputsLog, m_inputs, feeder::kInputKeys);
}

void Feeder::Periodic() {
  m_io->UpdateInputs(m_inputs);
  m_inputsLog.Publish();
  logging::LoggedTracer::Record("Feeder");
}

void Feeder::SetVoltage(double voltage) {
  if (Request()) {
    m_io->SetVoltage(voltage);
  }
}

void Feeder::SetVelocity(double velocity) {
  if (Request()) {
    m_io->SetVelocity(velocity);
  }
}

void Feeder::Off() {
  if (Request() && GoalChanged()) {
    m_io->Off();
  }
}

void Feeder::SetFeedVel() { SetVelocity(feeder::kFeederVelocity); }

void Feeder::Outtake() { SetVelocity(feeder::kOuttakeVelocity); }
