// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "subsystems/indexer/Indexer.h"

#include "logging/LoggedTracer.h"
#include "subsystems/indexer/IndexerConstants.h"

Indexer::Indexer(bool enabled, std::unique_ptr<RollerPairIO> io)
    : DisableSubsystem(enabled, "Indexer"), m_io{std::move(io)} {
  AddRollerPairInputs(m_inputsLog, m_inputs, indexer::kInputKeys);
}

void Indexer::Periodic() {
  DisableSubsystem::Periodic();
  m_io->UpdateInputs(m_inputs);
  m_inputsLog.Publish();
  logging::LoggedTracer::Record("Indexer");
}

frc2::CommandPtr Indexer::SetVoltage(double voltage) {
  return Run([this, voltage] { m_io->SetVoltage(voltage); }).FinallyDo([this] { m_io->Off(); });
}

frc2::CommandPtr Indexer::SetVelocity(double velocity) {
  return Run([this, velocity] { m_io->SetVelocity(velocity); }).FinallyDo([this] { m_io->Off(); });
}

frc2::CommandPtr Indexer::Off() {
  return RunOnce([this] { m_io->Off(); });
}

frc2::CommandPtr Indexer::SetIndexVel() { return SetVelocity(indexer::kIndexerVelocity); }
