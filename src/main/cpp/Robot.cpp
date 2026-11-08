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
#include <frc2/command/CommandScheduler.h>
#include <ctre/phoenix6/SignalLogger.hpp>

#include "Constants.h"
#include "logging/LoggedTracer.h"
#include "sim/SimBattery.h"
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
  // Instantiate our RobotContainer. This performs all the button bindings and puts the
  // autonomous chooser on the dashboard.
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
  logging::LoggedTracer::Record("Phoenix Refresh");
  // Runs the Scheduler: polls buttons, schedules commands, runs scheduled commands, removes
  // finished or interrupted commands, and runs subsystem Periodic() methods.
  frc2::CommandScheduler::GetInstance().Run();
  m_container->Periodic();

  m_batteryLog->Set(frc::RobotController::GetBatteryVoltage().value());
  m_brownoutLog->Set(frc::RobotController::IsBrownedOut());
  if (++m_canStatusCounter >= 10) {  // CAN status is a HAL call; 5 Hz is plenty
    m_canStatusCounter = 0;
    m_canUtilLog->Set(frc::RobotController::GetCANStatus().percentBusUtilization);
  }
  m_loopTimeLog->Set(static_cast<double>(frc::RobotController::GetFPGATime() - startUs) / 1000.0);
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
  // no auto command, uses trigger
  SignalLogger::WriteString("MatchStatus", "Auto");
}

void Robot::AutonomousPeriodic() {}

void Robot::AutonomousExit() { SignalLogger::WriteString("MatchStatus", "AutoEnded"); }

void Robot::TeleopInit() { SignalLogger::WriteString("MatchStatus", "Teleop"); }

void Robot::TeleopExit() { SignalLogger::WriteString("MatchStatus", "TeleopEnded"); }

void Robot::TeleopPeriodic() {}

void Robot::TestInit() {
  // Cancels all running commands at the start of test mode.
  frc2::CommandScheduler::GetInstance().CancelAll();
}

void Robot::TestPeriodic() {}

void Robot::SimulationInit() {}

void Robot::SimulationPeriodic() {
  // Apply every simulated mechanism's current draw to the simulated battery at once.
  sim_battery::Update();
}
