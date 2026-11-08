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
  DisableSubsystem::Periodic();
  m_io->UpdateInputs(m_inputs);
  m_inputsLog.Publish();
}

frc2::CommandPtr IntakeRollers::SetVoltage(double voltage) {
  return Run([this, voltage] { m_io->SetVoltage(voltage); }).FinallyDo([this] { m_io->Off(); });
}

frc2::CommandPtr IntakeRollers::SetVelocity(double velocity) {
  return Run([this, velocity] { m_io->SetVelocity(velocity); }).FinallyDo([this] { m_io->Off(); });
}

frc2::CommandPtr IntakeRollers::Off() {
  return RunOnce([this] { m_io->Off(); });
}

frc2::CommandPtr IntakeRollers::Unjam() { return SetVoltage(intake_rollers::kUnjamVoltage); }

frc2::CommandPtr IntakeRollers::Intake() { return SetVoltage(intake_rollers::kIntakeVoltage); }

frc2::CommandPtr IntakeRollers::Outtake() { return SetVoltage(intake_rollers::kOuttakeVoltage); }
