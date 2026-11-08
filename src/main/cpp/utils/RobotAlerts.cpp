// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "utils/RobotAlerts.h"

void RobotAlerts::Debounce(bool condition, int setLoops, int clearLoops, int& count, bool& active) {
  // Count loops in which the condition disagrees with the current alert state.
  if (condition != active) {
    count++;
    if (count >= (condition ? setLoops : clearLoops)) {
      active = condition;
      count = 0;
    }
  } else {
    count = 0;
  }
}

void RobotAlerts::Update(const Inputs& in) {
  // Resting voltage only means something while disabled; keep the last verdict while enabled.
  if (in.disabled) {
    Debounce(in.batteryVolts < kLowBatteryVolts, kLowBatteryLoops, kLowBatteryLoops,
             m_lowBatteryCount, m_lowBatteryActive);
  }
  m_lowBattery.Set(m_lowBatteryActive);

  Debounce(!in.driverControllerConnected, kControllerLoops, 1, m_controllerCount,
           m_controllerActive);
  m_controller.Set(m_controllerActive);

  // Latched: a brownout is worth knowing about for the rest of the event.
  m_brownoutActive = m_brownoutActive || in.brownedOut;
  m_brownout.Set(m_brownoutActive);

  if (in.canUtilization) {
    Debounce(*in.canUtilization > kHighCanUtilization, kHighCanSamples, kHighCanSamples, m_canCount,
             m_canActive);
  }
  m_can.Set(m_canActive);

  Debounce(in.loopTimeMs > kLoopBudgetMs, kOverrunLoops, kOverrunClearLoops, m_overrunCount,
           m_overrunActive);
  m_overrun.Set(m_overrunActive);

  m_noAutoActive = in.disabled && !in.autoSelected;
  m_noAuto.Set(m_noAutoActive);
}
