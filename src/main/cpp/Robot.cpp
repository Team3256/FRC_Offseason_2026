// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#include "Robot.h"

#include <cstdlib>
#include <filesystem>
#include <string>
#include <system_error>

#include <frc/DriverStation.h>
#include <frc/RobotBase.h>
#include <frc/RobotController.h>
#include <ctre/phoenix6/SignalLogger.hpp>

#include "Constants.h"
#include "logging/LoggedTracer.h"
#include "sim/PhysicsWorld.h"
#include "utils/PhoenixUtil.h"

using ctre::phoenix6::SignalLogger;

namespace {
const char* MatchTypeName(frc::DriverStation::MatchType type) {
  switch (type) {
    case frc::DriverStation::MatchType::kPractice:
      return "Practice";
    case frc::DriverStation::MatchType::kQualification:
      return "Qualification";
    case frc::DriverStation::MatchType::kElimination:
      return "Elimination";
    case frc::DriverStation::MatchType::kNone:
    default:
      return "None";
  }
}
}  // namespace

void Robot::ConfigureLogging() {
  if (!constants::feature_flags::kLoggingEnabled) {
    return;
  }
  namespace lc = constants::logging_config;
  std::string dir;
  if (frc::RobotBase::IsReal()) {
    // A FAT32 formatted USB stick must be plugged into the roboRIO for /U to exist.
    std::error_code ec;
    if (lc::kLogToUSB && std::filesystem::exists("/U", ec)) {
      dir = lc::kUsbLogDir;
    } else {
      dir = lc::kRioLogDir;  // created if missing
    }
  } else if (const char* env = std::getenv("ROBOT_LOG_DIR")) {
    dir = env;  // empty string: NetworkTables only (unit tests)
  } else {
    dir = lc::kSimLogDir;
  }
  logging::Logger::Start(dir);
}

Robot::Robot() {
  SignalLogger::EnableAutoLogging(false);
  frc::RobotController::SetBrownoutVoltage(4.75_V);
  ConfigureLogging();
  m_loopTimeLog = std::make_unique<logging::Output<double>>("LoopTimeMS");
  m_loopPeriodLog = std::make_unique<logging::Output<double>>("LoopPeriodMS");
  m_batteryLog = std::make_unique<logging::Output<double>>("SystemStats/BatteryVoltage");
  m_brownoutLog = std::make_unique<logging::Output<bool>>("SystemStats/BrownedOut");
  m_canUtilLog = std::make_unique<logging::Output<double>>("SystemStats/CANBus/Utilization");
  // Instantiate our RobotContainer. This builds every mechanism and puts the autonomous chooser on
  // the dashboard.
  m_container = std::make_unique<RobotContainer>();
}

Robot::~Robot() = default;

void Robot::RobotPeriodic() {
  const int64_t startUs = static_cast<int64_t>(frc::RobotController::GetFPGATime());
  if (m_lastLoopStartUs != 0) {
    m_loopPeriodLog->Set(static_cast<double>(startUs - m_lastLoopStartUs) / 1000.0);
  }
  m_lastLoopStartUs = startUs;

  logging::Logger::Periodic();
  logging::LoggedTracer::Reset();
  m_container->BeginLoop();
  phoenix_util::RefreshAll();
  phoenix_util::UpdateDeviceAlerts();
  logging::LoggedTracer::Record("Phoenix Refresh");
  // Inputs, buttons, autonomous, drive, then the superstructure's state machines and outputs.
  m_container->Periodic();

  RobotAlerts::Inputs alertInputs;
  alertInputs.batteryVolts = frc::RobotController::GetBatteryVoltage().value();
  alertInputs.brownedOut = frc::RobotController::IsBrownedOut();
  m_batteryLog->Set(alertInputs.batteryVolts);
  m_brownoutLog->Set(alertInputs.brownedOut);
  if (++m_canStatusCounter >= 10) {  // CAN status is a HAL call; 5 Hz is plenty
    m_canStatusCounter = 0;
    alertInputs.canUtilization = frc::RobotController::GetCANStatus().percentBusUtilization;
    m_canUtilLog->Set(*alertInputs.canUtilization);
  }
  alertInputs.disabled = frc::DriverStation::IsDisabled();
  alertInputs.loopTimeMs = m_lastLoopTimeMs;
  alertInputs.driverControllerConnected =
      frc::DriverStation::IsJoystickConnected(constants::controller::kDriverControllerPort);
  alertInputs.autoSelected = m_container->GetAutoChooser().Selected() != AutoChooser::kNoneName;
  m_alerts.Update(alertInputs);

  DriverDashboard::Inputs dash;
  dash.autoSelected = alertInputs.autoSelected;
  dash.autoAtStartPose = m_container->IsAtAutoStartPose();
  dash.batteryOk = !m_alerts.LowBattery();
  dash.controllerConnected = !m_alerts.DriverControllerDisconnected();
  dash.noBrownout = !m_alerts.BrownedOut();
  dash.canOk = !m_alerts.HighCanUtilization();
  dash.devicesConnected =
      phoenix_util::DisconnectedDeviceCount() == 0 && phoenix_util::ConfigFailureCount() == 0;
  dash.devicesCool = phoenix_util::HotDeviceCount() == 0;
  dash.camerasConnected = m_container->GetVision().AllCamerasConnected();
  dash.readyToShoot = m_container->GetSuperstructure().IsReadyToShoot();
  const auto& ss = m_container->GetSuperstructure();
  dash.state = std::string{Superstructure::ToString(ss.GetShooterMode())} + " · " +
               std::string{Superstructure::ToString(ss.GetIntakeMode())};
  dash.batteryVolts = alertInputs.batteryVolts;
  dash.matchTimeSeconds = frc::DriverStation::GetMatchTime().value();
  m_dashboard.Update(dash);

  m_lastLoopTimeMs = static_cast<double>(frc::RobotController::GetFPGATime() - startUs) / 1000.0;
  m_loopTimeLog->Set(m_lastLoopTimeMs);
}

void Robot::DisabledInit() {}
void Robot::DisabledPeriodic() {}

void Robot::DriverStationConnected() {
  if (frc::DriverStation::IsFMSAttached()) {
    SignalLogger::Start();
    SignalLogger::WriteString("MatchStatus", "MatchNotStarted");
    SignalLogger::WriteString("MatchType", MatchTypeName(frc::DriverStation::GetMatchType()));
    SignalLogger::WriteString("MatchNumber", std::to_string(frc::DriverStation::GetMatchNumber()));
    SignalLogger::WriteString("EventName", frc::DriverStation::GetEventName());
    SignalLogger::WriteString("ReplayNum", std::to_string(frc::DriverStation::GetReplayNumber()));
  }
}

void Robot::AutonomousInit() {
  // RobotContainer starts the selected routine when it sees autonomous enabled.
  SignalLogger::WriteString("MatchStatus", "Auto");
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() { SignalLogger::WriteString("MatchStatus", "AutoEnded"); }

void Robot::TeleopInit() { SignalLogger::WriteString("MatchStatus", "Teleop"); }

void Robot::TeleopExit() { SignalLogger::WriteString("MatchStatus", "TeleopEnded"); }

void Robot::TeleopPeriodic() {}

void Robot::TestInit() {
  // Start test mode from a clean slate: no request carried over.
  m_container->GetSuperstructure().StopAll();
}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {
  // 20 ms of physics (drivetrain, mechanisms, battery, field, FUEL) on simulated time, driven by the
  // controls the robot code just sent its motors.
  sim::PhysicsWorld::Get().Step(GetPeriod().value());
}
