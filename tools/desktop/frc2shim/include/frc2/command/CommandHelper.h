// Desktop-only shim of WPILib's frc2 command framework. See tools/desktop/frc2shim/README.md.
#pragma once

#include <concepts>
#include <memory>
#include <utility>

#include "frc2/command/Command.h"
#include "frc2/command/CommandPtr.h"

namespace frc2 {

template <std::derived_from<Command> Base, typename CRTP>
class CommandHelper : public Base {
  using Base::Base;

 public:
  CommandHelper() = default;

  CommandPtr ToPtr() && override {
    return CommandPtr(
        std::unique_ptr<Command>(std::make_unique<CRTP>(std::move(*static_cast<CRTP*>(this)))));
  }
};

}  // namespace frc2
