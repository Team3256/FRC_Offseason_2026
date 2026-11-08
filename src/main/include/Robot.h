// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <memory>

#include <frc/TimedRobot.h>

#include "RobotContainer.h"
#include "logging/Logger.h"
#include "utils/RobotAlerts.h"

class Robot : public frc::TimedRobot {
 public:
  Robot();
  ~Robot() override;

  void RobotPeriodic() override;
  void DisabledInit() override;
  void DisabledPeriodic() override;
  void DriverStationConnected() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void AutonomousExit() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  void TeleopExit() override;
  void TestInit() override;
  void TestPeriodic() override;
  void SimulationInit() override;
  void SimulationPeriodic() override;

  RobotContainer& Container() { return *m_container; }
  const RobotAlerts& Alerts() const { return m_alerts; }

  /** Starts logging the way the robot does (file + NT); call before anything creates log entries.
   */
  static void ConfigureLogging();

 private:
  std::unique_ptr<RobotContainer> m_container;
  std::unique_ptr<logging::Output<double>> m_loopTimeLog;
  std::unique_ptr<logging::Output<double>> m_loopPeriodLog;
  std::unique_ptr<logging::Output<double>> m_batteryLog;
  std::unique_ptr<logging::Output<bool>> m_brownoutLog;
  std::unique_ptr<logging::Output<double>> m_canUtilLog;
  RobotAlerts m_alerts;
  int64_t m_lastLoopStartUs = 0;
  double m_lastLoopTimeMs = 0.0;
  int m_canStatusCounter = 0;
};
