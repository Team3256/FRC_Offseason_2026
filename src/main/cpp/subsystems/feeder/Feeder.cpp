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
  DisableSubsystem::Periodic();
  m_io->UpdateInputs(m_inputs);
  m_inputsLog.Publish();
  logging::LoggedTracer::Record("Feeder");
}

frc2::CommandPtr Feeder::SetVoltage(double voltage) {
  return Run([this, voltage] { m_io->SetVoltage(voltage); }).FinallyDo([this] { m_io->Off(); });
}

frc2::CommandPtr Feeder::SetVelocity(double velocity) {
  return Run([this, velocity] { m_io->SetVelocity(velocity); }).FinallyDo([this] { m_io->Off(); });
}

frc2::CommandPtr Feeder::Off() {
  return RunOnce([this] { m_io->Off(); });
}

frc2::CommandPtr Feeder::SetFeedVel() { return SetVelocity(feeder::kFeederVelocity); }

frc2::CommandPtr Feeder::Outtake() { return SetVelocity(feeder::kOuttakeVelocity); }
