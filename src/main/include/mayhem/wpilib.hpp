#pragma once
// Optional WPILib 2026 adapter: the DriveToPose command (needs wpilibNewCommands) and, for
// existing users, the WPILOG observer. Command-free robots include mayhem/pose_driver.hpp and
// mayhem/wpilib_log.hpp instead. Core headers remain framework-independent C++17.
#include "mayhem/pose_driver.hpp"
#include "mayhem/wpilib_log.hpp"
#include <frc/Timer.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <functional>

namespace mayhem {
// PoseDriver as a command: Initialize starts it, Execute updates it, and an interrupt cancels it.
class DriveToPose final : public frc2::CommandHelper<frc2::Command, DriveToPose> {
 public:
  DriveToPose(std::function<Pose()> pose, std::function<Velocity()> velocity,
             std::function<void(Velocity)> output, Goal goal, Constraints constraints,
             units::second_t watchdog, frc2::Requirements requirements = {})
    : driver_(std::move(pose), std::move(velocity), std::move(output), goal, constraints, watchdog.value()) { AddRequirements(requirements); }
  void Initialize() override {driver_.Start(frc::Timer::GetFPGATimestamp().value());}
  void Execute() override {driver_.Update(frc::Timer::GetFPGATimestamp().value());}
  bool IsFinished() override {return !driver_.Running();}
  void End(bool) override {driver_.Cancel();}
  PoseCommandStatus Status() const {return driver_.Status();}
 private:
  PoseDriver driver_;
};
} // namespace mayhem
