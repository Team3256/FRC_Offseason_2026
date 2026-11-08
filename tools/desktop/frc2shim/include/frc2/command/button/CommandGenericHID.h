// Desktop-only shim of WPILib's frc2 command framework. See tools/desktop/frc2shim/README.md.
#pragma once

#include <frc/GenericHID.h>

#include "frc2/command/CommandScheduler.h"
#include "frc2/command/button/Trigger.h"

namespace frc2 {

class CommandGenericHID {
 public:
  explicit CommandGenericHID(int port) : m_hid{port} {}
  frc::GenericHID& GetHID() { return m_hid; }

  Trigger Button(int button, frc::EventLoop* loop =
                                 CommandScheduler::GetInstance().GetDefaultButtonLoop()) const;
  Trigger POV(int angle,
              frc::EventLoop* loop = CommandScheduler::GetInstance().GetDefaultButtonLoop()) const;
  Trigger POV(int pov, int angle, frc::EventLoop* loop) const;
  Trigger POVUp(
      frc::EventLoop* loop = CommandScheduler::GetInstance().GetDefaultButtonLoop()) const;
  Trigger POVUpRight(
      frc::EventLoop* loop = CommandScheduler::GetInstance().GetDefaultButtonLoop()) const;
  Trigger POVRight(
      frc::EventLoop* loop = CommandScheduler::GetInstance().GetDefaultButtonLoop()) const;
  Trigger POVDownRight(
      frc::EventLoop* loop = CommandScheduler::GetInstance().GetDefaultButtonLoop()) const;
  Trigger POVDown(
      frc::EventLoop* loop = CommandScheduler::GetInstance().GetDefaultButtonLoop()) const;
  Trigger POVDownLeft(
      frc::EventLoop* loop = CommandScheduler::GetInstance().GetDefaultButtonLoop()) const;
  Trigger POVLeft(
      frc::EventLoop* loop = CommandScheduler::GetInstance().GetDefaultButtonLoop()) const;
  Trigger POVUpLeft(
      frc::EventLoop* loop = CommandScheduler::GetInstance().GetDefaultButtonLoop()) const;
  Trigger POVCenter(
      frc::EventLoop* loop = CommandScheduler::GetInstance().GetDefaultButtonLoop()) const;
  Trigger AxisLessThan(
      int axis, double threshold,
      frc::EventLoop* loop = CommandScheduler::GetInstance().GetDefaultButtonLoop()) const;
  Trigger AxisGreaterThan(
      int axis, double threshold,
      frc::EventLoop* loop = CommandScheduler::GetInstance().GetDefaultButtonLoop()) const;
  double GetRawAxis(int axis) const { return m_hid.GetRawAxis(axis); }

 private:
  frc::GenericHID m_hid;
};

}  // namespace frc2
