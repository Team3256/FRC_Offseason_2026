// Desktop-only shim of WPILib's frc2 command framework. See tools/desktop/frc2shim/README.md.
#pragma once

#include <string>
#include <string_view>

#include <wpi/sendable/Sendable.h>
#include <wpi/sendable/SendableHelper.h>

#include "frc2/command/Subsystem.h"

namespace frc2 {

class SubsystemBase : public Subsystem,
                      public wpi::Sendable,
                      public wpi::SendableHelper<SubsystemBase> {
 public:
  void InitSendable(wpi::SendableBuilder& builder) override;
  std::string GetName() const override { return m_name; }
  void SetName(std::string_view name) override { m_name = name; }
  std::string GetSubsystem() const { return m_name; }
  void SetSubsystem(std::string_view name) { m_name = name; }

 protected:
  SubsystemBase();
  explicit SubsystemBase(std::string_view name);

 private:
  std::string m_name{"Unnamed"};
};

}  // namespace frc2
