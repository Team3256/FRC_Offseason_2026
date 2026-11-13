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
  m_io->UpdateInputs(m_inputs);
  m_inputsLog.Publish();
  logging::LoggedTracer::Record("Indexer");
}

void Indexer::SetVoltage(double voltage) {
  if (Request()) {
    m_io->SetVoltage(voltage);
  }
}

void Indexer::SetVelocity(double velocity) {
  if (Request()) {
    m_io->SetVelocity(velocity);
  }
}

void Indexer::Off() {
  if (Request() && GoalChanged()) {
    m_io->Off();
  }
}

void Indexer::SetIndexVel() { SetVelocity(indexer::kIndexerVelocity); }
