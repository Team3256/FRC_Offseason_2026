#pragma once
// Framework-free drive-to-pose with a watchdog: what the WPILib DriveToPose command does,
// for robots that run their own loop instead of the command scheduler. C++17.
#include "mayhem/pose_controller.hpp"
#include <cmath>
#include <functional>
#include <utility>

namespace mayhem {
enum class PoseCommandStatus { Idle, Running, Reached, Failed, Cancelled };
// Call Start(now) once, then Update(now) every loop with a monotonic time in seconds
// (e.g. frc::Timer::GetFPGATimestamp().value()). Update drives `output` while Running and
// writes a zero velocity once when the drive ends: reached, failed (watchdog elapsed, a
// non-positive or non-finite watchdog, or an invalid controller result), or Cancel().
// A loop with dt == 0 is skipped. Update after the end does nothing and returns the status.
class PoseDriver {
 public:
  PoseDriver(std::function<Pose()> pose, std::function<Velocity()> velocity,
             std::function<void(Velocity)> output, Goal goal, Constraints constraints, double watchdog)
    : pose_(std::move(pose)), velocity_(std::move(velocity)), output_(std::move(output)),
      goal_(goal), constraints_(constraints), watchdog_(watchdog) {}
  void Start(double now) {
    start_=last_=now;
    controller_.Reset(velocity_());status_=PoseCommandStatus::Running;
  }
  PoseCommandStatus Update(double now) {
    if(status_!=PoseCommandStatus::Running)return status_;
    const double dt=now-last_;last_=now;
    if(now-start_>=watchdog_||watchdog_<=0||!std::isfinite(watchdog_))return End(PoseCommandStatus::Failed);
    if(dt==0)return status_;
    const auto result=controller_.Calculate(pose_(),velocity_(),goal_,constraints_,dt);
    if(!result.valid)return End(PoseCommandStatus::Failed);
    output_(result.command);
    return result.reached?End(PoseCommandStatus::Reached):status_;
  }
  // Stops a running drive (status Cancelled) and always writes a zero velocity.
  void Cancel() {if(status_==PoseCommandStatus::Running)status_=PoseCommandStatus::Cancelled;output_({});}
  PoseCommandStatus Status() const {return status_;}
  bool Running() const {return status_==PoseCommandStatus::Running;}
 private:
  PoseCommandStatus End(PoseCommandStatus status){status_=status;output_({});return status_;}
  std::function<Pose()> pose_;std::function<Velocity()> velocity_;std::function<void(Velocity)> output_;
  Goal goal_;Constraints constraints_;double watchdog_,start_=0,last_=0;
  PoseController controller_;PoseCommandStatus status_=PoseCommandStatus::Idle;
};
} // namespace mayhem
