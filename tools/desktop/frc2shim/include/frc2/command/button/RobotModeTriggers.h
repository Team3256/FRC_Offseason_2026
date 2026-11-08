// Desktop-only shim of WPILib's frc2 command framework. See tools/desktop/frc2shim/README.md.
#pragma once

#include <frc/DriverStation.h>

#include "frc2/command/button/Trigger.h"

namespace frc2 {

class RobotModeTriggers {
 public:
  RobotModeTriggers() = delete;
  static Trigger Autonomous() { return Trigger{&frc::DriverStation::IsAutonomousEnabled}; }
  static Trigger Teleop() { return Trigger{&frc::DriverStation::IsTeleopEnabled}; }
  static Trigger Disabled() { return Trigger{&frc::DriverStation::IsDisabled}; }
  static Trigger Test() { return Trigger{&frc::DriverStation::IsTestEnabled}; }
};

}  // namespace frc2
