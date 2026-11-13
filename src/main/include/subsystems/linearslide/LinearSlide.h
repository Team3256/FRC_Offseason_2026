// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <memory>

#include "logging/Logger.h"
#include "subsystems/linearslide/LinearSlideIO.h"
#include "utils/DisableSubsystem.h"

/** The intake's linear slide. Positions are mechanism rotations. */
class LinearSlide : public DisableSubsystem {
 public:
  LinearSlide(bool enabled, std::unique_ptr<LinearSlideIO> io);

  void Periodic() override;

  // Requests: call every loop the request should hold.
  void SetPosition(double position);
  void SetNonExtendedPosition(double position);
  void SetVoltage(double voltage);
  void Off();
  void GoToStow();
  void GoToGroundIntake();
  void GoToJitter();
  void GoToJitterIntermediate();
  /**
   * Out to the initial intake position until both motors are within 0.0005 of it (or 2 s pass),
   * then to the final intake position. Starts over whenever another request came in between.
   */
  void GoToIntakeSequence();
  /**
   * Alternates the jitter and intermediate positions, 0.2 s each, slot 0. Starts over whenever
   * another request came in between.
   */
  void JitterIntake();

  /** Both motors within 0.01 rotations of the request. */
  bool ReachedPosition() const;
  /** Both motors inside [0, 0.3745]: the zone where the slide and hood can collide. */
  bool AtPosition() const;
  double RequestedPosition() const { return m_reqPosition; }
  const LinearSlideIOInputs& Inputs() const { return m_inputs; }

 private:
  bool ReachedPosition(double tolerance) const;
  bool IsExtended() const;
  /** Position control in the extended slot once the slide is out, slot 0 otherwise. */
  void Drive(double position);
  void DriveNonExtended(double position);

  std::unique_ptr<LinearSlideIO> m_io;
  LinearSlideIOInputs m_inputs;
  double m_reqPosition = 0.0;
  // Start of the running intake/jitter sequence, and whether the intake one reached its first leg.
  double m_sequenceStart = 0.0;
  bool m_sequenceFinalLeg = false;
  logging::InputsLogger m_inputsLog{"LinearSlide"};
  logging::Output<double> m_reqPositionLog{"LinearSlide/reqPosition"};
};
