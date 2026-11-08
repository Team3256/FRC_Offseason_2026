// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <functional>
#include <memory>

#include <frc/filter/LinearFilter.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/button/Trigger.h>

#include "logging/Logger.h"
#include "subsystems/shooterpivot/ShooterPivotIO.h"
#include "utils/DisableSubsystem.h"

/** The shooter hood. Positions are mechanism rotations. */
class ShooterPivot : public DisableSubsystem {
 public:
  ShooterPivot(bool enabled, std::unique_ptr<ShooterPivotIO> io);

  void Periodic() override;

  frc2::CommandPtr SetPosition(double position);
  frc2::CommandPtr SetPosition(std::function<double()> position);
  frc2::CommandPtr SetVoltage(double voltage);
  frc2::CommandPtr Off();
  frc2::CommandPtr ShootHub(std::function<double()> distance);
  frc2::CommandPtr FeedCorner(std::function<double()> distance);

  /** Within 0.01 rotations of the requested position. */
  bool ReachedPosition() const;
  /** While true, every position request is replaced by 0 (hood down to fit under the trench). */
  void SetTrenchOverride(bool enabled) { m_trenchOverride = enabled; }
  bool TrenchOverride() const { return m_trenchOverride; }
  double RequestedPosition() const { return m_reqPosition; }
  const ShooterPivotIOInputs& Inputs() const { return m_inputs; }

  frc2::Trigger reachedPosition{[this] { return ReachedPosition(); }};

 private:
  std::unique_ptr<ShooterPivotIO> m_io;
  ShooterPivotIOInputs m_inputs;
  double m_reqPosition = 0.0;
  double m_lastHoodAngle = 0;
  double m_hoodVel = 0;
  bool m_trenchOverride = false;
  frc::LinearFilter<double> m_hoodAngleFilter =
      frc::LinearFilter<double>::MovingAverage(static_cast<int>(0.1 / .02));

  logging::InputsLogger m_inputsLog{"ShooterPivot"};
  logging::Output<double> m_reqPositionLog{"ShooterPivot/reqPosition"};
  logging::Output<double> m_hoodVelLog{"ShooterPivot/hoodVel"};
};
