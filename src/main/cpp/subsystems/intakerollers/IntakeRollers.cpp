// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "subsystems/intakerollers/IntakeRollers.h"

#include "logging/LoggedTracer.h"
#include "subsystems/intakerollers/IntakeRollerConstants.h"

IntakeRollers::IntakeRollers(bool enabled, std::unique_ptr<RollerPairIO> io)
    : DisableSubsystem(enabled, "IntakeRollers"), m_io{std::move(io)} {
  AddRollerPairInputs(m_inputsLog, m_inputs, intake_rollers::kInputKeys);
}

void IntakeRollers::Periodic() {
  m_io->UpdateInputs(m_inputs);
  m_inputsLog.Publish();
  logging::LoggedTracer::Record("IntakeRollers");
}

void IntakeRollers::SetVoltage(double voltage) {
  if (Request()) {
    m_io->SetVoltage(voltage);
  }
}

void IntakeRollers::SetVelocity(double velocity) {
  if (Request()) {
    m_io->SetVelocity(velocity);
  }
}

void IntakeRollers::Off() {
  if (Request() && GoalChanged()) {
    m_io->Off();
  }
}

void IntakeRollers::Unjam() { SetVoltage(intake_rollers::kUnjamVoltage); }

void IntakeRollers::Intake() { SetVoltage(intake_rollers::kIntakeVoltage); }

void IntakeRollers::Outtake() { SetVoltage(intake_rollers::kOuttakeVoltage); }
