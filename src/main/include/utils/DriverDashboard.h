// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <string>

#include <networktables/BooleanTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/StringTopic.h>

/**
 * Plain values for the drive team's Elastic tabs (deploy/elastic_layout.json), published under
 * "SmartDashboard/Checks/..." and "SmartDashboard/Driver/...". The checks are green when good, so
 * they are the opposite of the alerts. Publishers are created once, so a loop only does sets.
 * Always published (unlike the logger's NetworkTables output, which is off while the FMS is
 * attached), so the checks still work in queue.
 */
class DriverDashboard {
 public:
  struct Inputs {
    // Pre-match checks.
    bool autoSelected = false;
    bool autoAtStartPose = false;
    bool batteryOk = false;
    bool controllerConnected = false;
    bool noBrownout = false;
    bool canOk = false;
    bool devicesConnected = false;
    bool devicesCool = false;
    bool camerasConnected = false;
    // Teleop.
    bool readyToShoot = false;
    std::string state;
    double batteryVolts = 0.0;
    double matchTimeSeconds = 0.0;
  };

  DriverDashboard();

  void Update(const Inputs& in);

 private:
  nt::BooleanPublisher m_autoSelected;
  nt::BooleanPublisher m_autoAtStart;
  nt::BooleanPublisher m_battery;
  nt::BooleanPublisher m_controller;
  nt::BooleanPublisher m_brownout;
  nt::BooleanPublisher m_can;
  nt::BooleanPublisher m_devices;
  nt::BooleanPublisher m_temps;
  nt::BooleanPublisher m_cameras;
  nt::BooleanPublisher m_allGood;
  nt::BooleanPublisher m_ready;
  nt::StringPublisher m_state;
  nt::DoublePublisher m_batteryVolts;
  nt::DoublePublisher m_matchTime;
};
