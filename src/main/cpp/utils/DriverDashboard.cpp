// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "utils/DriverDashboard.h"

#include <networktables/NetworkTableInstance.h>

DriverDashboard::DriverDashboard() {
  auto table = nt::NetworkTableInstance::GetDefault().GetTable("SmartDashboard");
  m_autoSelected = table->GetBooleanTopic("Checks/Auto Selected").Publish();
  m_autoAtStart = table->GetBooleanTopic("Checks/At Auto Start").Publish();
  m_battery = table->GetBooleanTopic("Checks/Battery").Publish();
  m_controller = table->GetBooleanTopic("Checks/Controller").Publish();
  m_brownout = table->GetBooleanTopic("Checks/No Brownout").Publish();
  m_can = table->GetBooleanTopic("Checks/CAN Bus").Publish();
  m_devices = table->GetBooleanTopic("Checks/Motors Connected").Publish();
  m_temps = table->GetBooleanTopic("Checks/Motors Cool").Publish();
  m_cameras = table->GetBooleanTopic("Checks/Cameras").Publish();
  m_allGood = table->GetBooleanTopic("Checks/All Systems Go").Publish();
  m_ready = table->GetBooleanTopic("Driver/Ready To Shoot").Publish();
  m_state = table->GetStringTopic("Driver/State").Publish();
  m_batteryVolts = table->GetDoubleTopic("Driver/Battery Voltage").Publish();
  m_matchTime = table->GetDoubleTopic("Driver/Match Time").Publish();
}

void DriverDashboard::Update(const Inputs& in) {
  m_autoSelected.Set(in.autoSelected);
  m_autoAtStart.Set(in.autoAtStartPose);
  m_battery.Set(in.batteryOk);
  m_controller.Set(in.controllerConnected);
  m_brownout.Set(in.noBrownout);
  m_can.Set(in.canOk);
  m_devices.Set(in.devicesConnected);
  m_temps.Set(in.devicesCool);
  m_cameras.Set(in.camerasConnected);
  // Everything except the start pose: that one depends on where the robot is placed, not on health.
  m_allGood.Set(in.autoSelected && in.batteryOk && in.controllerConnected && in.noBrownout &&
                in.canOk && in.devicesConnected && in.devicesCool && in.camerasConnected);
  m_ready.Set(in.readyToShoot);
  m_state.Set(in.state);
  m_batteryVolts.Set(in.batteryVolts);
  m_matchTime.Set(in.matchTimeSeconds);
}
