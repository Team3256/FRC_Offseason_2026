// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <numeric>
#include <thread>
#include <vector>

#include <frc/RobotController.h>
#include <frc/simulation/DriverStationSim.h>
#include <frc/simulation/XboxControllerSim.h>
#include <frc2/command/CommandScheduler.h>
#include <hal/DriverStationTypes.h>

#include "Robot.h"
#include "TestUtil.h"
#include "utils/PhoenixUtil.h"

/** The robot program with its loop function exposed so a test can step it. */
class SimRobot : public Robot {
 public:
  using frc::IterativeRobotBase::LoopFunc;
};

/**
 * Runs the whole robot program (Robot -> RobotContainer -> every subsystem, simulated) one 20 ms
 * loop at a time, in real time: Phoenix 6's simulated devices run their control loops on the wall
 * clock, so the robot loop has to as well. Driver Station state and the driver's Xbox controller
 * are simulated through the HAL, exactly as the Driver Station would set them.
 */
class RobotHarness {
 public:
  explicit RobotHarness(bool red = false) {
    test_util::UseRepoDeployDir();
    frc::sim::DriverStationSim::ResetData();
    frc::sim::DriverStationSim::SetDsAttached(true);
    SetAlliance(red);
    frc::sim::DriverStationSim::SetEnabled(false);
    frc::sim::DriverStationSim::NotifyNewData();
    m_robot = std::make_unique<SimRobot>();
    m_driver = std::make_unique<frc::sim::XboxControllerSim>(0);
    m_operator = std::make_unique<frc::sim::XboxControllerSim>(1);
    m_next = std::chrono::steady_clock::now();
  }

  ~RobotHarness() {
    Disable();
    Step(1);
    // Tear down in the order the real program never has to: commands, then bindings, then
    // subsystems, then the objects they point at.
    auto& scheduler = frc2::CommandScheduler::GetInstance();
    scheduler.CancelAll();
    scheduler.GetDefaultButtonLoop()->Clear();
    scheduler.UnregisterAllSubsystems();
    m_robot.reset();
    phoenix_util::ClearRegisteredSignals();
  }

  /** Runs {@code loops} robot loops, 20 ms apart in real time. */
  void Step(int loops = 1) {
    for (int i = 0; i < loops; i++) {
      m_next += std::chrono::milliseconds(20);
      std::this_thread::sleep_until(m_next);
      frc::sim::DriverStationSim::NotifyNewData();
      const auto t0 = std::chrono::steady_clock::now();
      m_robot->LoopFunc();
      m_loopMicros.push_back(
          std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count());
      m_loops++;
    }
  }

  /** Steps until {@code condition} is true; returns false if it is not true within {@code
   * maxLoops}. */
  bool StepUntil(const std::function<bool()>& condition, int maxLoops) {
    for (int i = 0; i < maxLoops; i++) {
      if (condition()) {
        return true;
      }
      Step();
    }
    return condition();
  }

  void Teleop() { SetMode(true, false); }
  void Autonomous() { SetMode(true, true); }
  void Disable() { SetMode(false, false); }

  void SetAlliance(bool red) {
    frc::sim::DriverStationSim::SetAllianceStationId(red ? HAL_AllianceStationID_kRed1
                                                         : HAL_AllianceStationID_kBlue1);
    frc::sim::DriverStationSim::NotifyNewData();
  }

  /** Presses a driver button for one loop, then releases it. */
  void Tap(const std::function<void(frc::sim::XboxControllerSim&, bool)>& button) {
    button(*m_driver, true);
    Step();
    button(*m_driver, false);
    Step();
  }

  frc::sim::XboxControllerSim& Driver() { return *m_driver; }
  RobotContainer& C() { return m_robot->Container(); }
  SimRobot& R() { return *m_robot; }

  double MeanLoopMicros() const {
    return m_loopMicros.empty() ? 0
                                : std::accumulate(m_loopMicros.begin(), m_loopMicros.end(), 0.0) /
                                      static_cast<double>(m_loopMicros.size());
  }
  double PercentileLoopMicros(double p) const {
    if (m_loopMicros.empty()) {
      return 0;
    }
    std::vector<double> v = m_loopMicros;
    std::sort(v.begin(), v.end());
    return v[std::min(v.size() - 1, static_cast<size_t>(p * static_cast<double>(v.size())))];
  }
  void ClearLoopStats() { m_loopMicros.clear(); }

 private:
  void SetMode(bool enabled, bool autonomous) {
    frc::sim::DriverStationSim::SetAutonomous(autonomous);
    frc::sim::DriverStationSim::SetTest(false);
    frc::sim::DriverStationSim::SetEnabled(enabled);
    frc::sim::DriverStationSim::NotifyNewData();
  }

  std::unique_ptr<SimRobot> m_robot;
  std::unique_ptr<frc::sim::XboxControllerSim> m_driver;
  std::unique_ptr<frc::sim::XboxControllerSim> m_operator;
  std::chrono::steady_clock::time_point m_next;
  std::vector<double> m_loopMicros;
  int m_loops = 0;
};
