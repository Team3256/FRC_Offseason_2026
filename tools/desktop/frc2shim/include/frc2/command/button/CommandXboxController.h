// Desktop-only shim of WPILib's frc2 command framework. See tools/desktop/frc2shim/README.md.
#pragma once

#include <frc/XboxController.h>

#include "frc2/command/button/CommandGenericHID.h"
#include "frc2/command/button/Trigger.h"

namespace frc2 {

class CommandXboxController : public CommandGenericHID {
 public:
  explicit CommandXboxController(int port) : CommandGenericHID(port), m_hid{port} {}
  frc::XboxController& GetHID() { return m_hid; }

#define FRC2SHIM_BUTTON(Name)                                                                 \
  Trigger Name(frc::EventLoop* loop = CommandScheduler::GetInstance().GetDefaultButtonLoop()) \
      const {                                                                                 \
    return Trigger(loop, [hid = &m_hid] { return hid->Get##Name##Button(); });                \
  }
  FRC2SHIM_BUTTON(A)
  FRC2SHIM_BUTTON(B)
  FRC2SHIM_BUTTON(X)
  FRC2SHIM_BUTTON(Y)
  FRC2SHIM_BUTTON(LeftBumper)
  FRC2SHIM_BUTTON(RightBumper)
  FRC2SHIM_BUTTON(Back)
  FRC2SHIM_BUTTON(Start)
  FRC2SHIM_BUTTON(LeftStick)
  FRC2SHIM_BUTTON(RightStick)
#undef FRC2SHIM_BUTTON

  Trigger LeftTrigger(
      double threshold = 0.5,
      frc::EventLoop* loop = CommandScheduler::GetInstance().GetDefaultButtonLoop()) const {
    return Trigger(loop,
                   [hid = &m_hid, threshold] { return hid->GetLeftTriggerAxis() > threshold; });
  }
  Trigger RightTrigger(
      double threshold = 0.5,
      frc::EventLoop* loop = CommandScheduler::GetInstance().GetDefaultButtonLoop()) const {
    return Trigger(loop,
                   [hid = &m_hid, threshold] { return hid->GetRightTriggerAxis() > threshold; });
  }

  double GetLeftX() const { return m_hid.GetLeftX(); }
  double GetRightX() const { return m_hid.GetRightX(); }
  double GetLeftY() const { return m_hid.GetLeftY(); }
  double GetRightY() const { return m_hid.GetRightY(); }
  double GetLeftTriggerAxis() const { return m_hid.GetLeftTriggerAxis(); }
  double GetRightTriggerAxis() const { return m_hid.GetRightTriggerAxis(); }

 private:
  frc::XboxController m_hid;
};

}  // namespace frc2
