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
#include "subsystems/linearslide/LinearSlideIO.h"
#include "utils/DisableSubsystem.h"

/** The intake's linear slide. Positions are mechanism rotations. */
class LinearSlide : public DisableSubsystem {
 public:
  LinearSlide(bool enabled, std::unique_ptr<LinearSlideIO> io);

  void Periodic() override;

  frc2::CommandPtr SetPosition(double position);
  frc2::CommandPtr SetPosition(std::function<double()> position);
  frc2::CommandPtr SetNonExtendedPosition(double position);
  frc2::CommandPtr SetVoltage(double voltage);
  frc2::CommandPtr Off();
  frc2::CommandPtr GoToStow();
  frc2::CommandPtr GoToGroundIntake();
  frc2::CommandPtr GoToIntakeSequence();
  frc2::CommandPtr GoToJitter();
  frc2::CommandPtr GoToJitterIntermediate();
  frc2::CommandPtr JitterIntake();

  /** Both motors within 0.01 rotations of the request. */
  bool ReachedPosition() const;
  /** Both motors inside [0, 0.3745]: the zone where the slide and hood can collide. */
  bool AtPosition() const;
  double RequestedPosition() const { return m_reqPosition; }
  const LinearSlideIOInputs& Inputs() const { return m_inputs; }

  frc2::Trigger reachedPosition{[this] { return ReachedPosition(); }};
  frc2::Trigger isBetweenPos{[this] { return AtPosition(); }};

 private:
  bool ReachedPosition(double tolerance) const;
  bool IsExtended() const;

  std::unique_ptr<LinearSlideIO> m_io;
  LinearSlideIOInputs m_inputs;
  double m_reqPosition = 0.0;
  logging::InputsLogger m_inputsLog{"LinearSlide"};
  logging::Output<double> m_reqPositionLog{"LinearSlide/reqPosition"};
};
