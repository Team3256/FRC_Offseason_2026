// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <optional>

#include <frc/Alert.h>

/**
 * Robot-wide dashboard alerts ("Alerts" on SmartDashboard / Elastic) for things the drive team
 * should notice before or during a match. Information only: none of them change what the robot
 * does. Device-level alerts (disconnected or hot motors, failed configs) live in phoenix_util.
 *
 * Every check counts robot loops instead of reading a clock, so the logic is deterministic.
 */
class RobotAlerts {
 public:
  /** A resting (disabled) battery below this is worth swapping before a match. */
  static constexpr double kLowBatteryVolts = 12.0;
  /** Loops a condition must hold before its alert changes (50 loops = 1 s). */
  static constexpr int kLowBatteryLoops = 50;
  static constexpr int kControllerLoops = 25;
  static constexpr double kHighCanUtilization = 0.9;
  static constexpr int kHighCanSamples = 5;
  static constexpr double kLoopBudgetMs = 20.0;
  static constexpr int kOverrunLoops = 10;
  static constexpr int kOverrunClearLoops = 250;

  struct Inputs {
    bool disabled = true;
    double batteryVolts = 12.5;
    bool brownedOut = false;
    /** Time spent in RobotPeriodic this loop. */
    double loopTimeMs = 0.0;
    bool driverControllerConnected = true;
    /** True when the auto chooser has a routine selected (not "Nothing"). */
    bool autoSelected = true;
    /** Bus utilization (0..1) when it was sampled this loop. */
    std::optional<double> canUtilization;
  };

  void Update(const Inputs& in);

  bool LowBattery() const { return m_lowBatteryActive; }
  bool DriverControllerDisconnected() const { return m_controllerActive; }
  bool BrownedOut() const { return m_brownoutActive; }
  bool HighCanUtilization() const { return m_canActive; }
  bool LoopOverrun() const { return m_overrunActive; }
  bool NoAutoSelected() const { return m_noAutoActive; }

 private:
  /** Counts how long {@code condition} has held and flips {@code active} after {@code loops}. */
  static void Debounce(bool condition, int setLoops, int clearLoops, int& count, bool& active);

  frc::Alert m_lowBattery{"Battery is below 12.0 V while disabled; swap it before the match.",
                          frc::Alert::AlertType::kWarning};
  frc::Alert m_controller{"Driver controller (port 0) is disconnected.",
                          frc::Alert::AlertType::kError};
  frc::Alert m_brownout{"The roboRIO browned out since it booted.", frc::Alert::AlertType::kError};
  frc::Alert m_can{"CAN bus utilization is above 90%.", frc::Alert::AlertType::kWarning};
  frc::Alert m_overrun{"Robot loop is taking longer than 20 ms.", frc::Alert::AlertType::kWarning};
  frc::Alert m_noAuto{"No autonomous routine is selected.", frc::Alert::AlertType::kWarning};

  int m_lowBatteryCount = 0;
  bool m_lowBatteryActive = false;
  int m_controllerCount = 0;
  bool m_controllerActive = false;
  bool m_brownoutActive = false;
  int m_canCount = 0;
  bool m_canActive = false;
  int m_overrunCount = 0;
  bool m_overrunActive = false;
  bool m_noAutoActive = false;
};
