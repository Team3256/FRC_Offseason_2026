#pragma once
// mayhem::sim WPILib adapter (C++20, WPILib 2026). The only sim header besides phoenix6.hpp that
// includes WPILib: geometry conversions for logging/AdvantageScope and a stepped-time helper.
//
//   mayhem::sim::SteppedTime time;          // pauses the HAL clock (frc::sim::PauseTiming)
//   for (...) { robot loop; time.Step(20_ms); }
//
// SteppedTime advances the WPILib/HAL clock by exactly the step, so a 20 s autonomous runs as
// fast as the CPU allows. Run the physics (1 ms sub-steps) between StepTiming calls.
#include "mayhem/sim/game_pieces.hpp"
#include "mayhem/sim/swerve.hpp"

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Pose3d.h>
#include <frc/geometry/Translation3d.h>
#include <frc/simulation/SimHooks.h>
#include <units/time.h>

#include <vector>

namespace mayhem::sim {

inline frc::Pose2d ToPose2d(const Pose2& p) {
  return {units::meter_t{p.x}, units::meter_t{p.y}, frc::Rotation2d{units::radian_t{p.yaw}}};
}
inline frc::Pose3d ToPose3d(const Pose3& p) {
  return {units::meter_t{p.x}, units::meter_t{p.y}, units::meter_t{p.z},
          frc::Rotation3d{units::radian_t{p.roll}, units::radian_t{p.pitch}, units::radian_t{p.yaw}}};
}
inline frc::Translation3d ToTranslation3d(const Vec3& v) {
  return {units::meter_t{v[0]}, units::meter_t{v[1]}, units::meter_t{v[2]}};
}
inline std::vector<frc::Translation3d> ToTranslations(const std::vector<Vec3>& points) {
  std::vector<frc::Translation3d> out;
  out.reserve(points.size());
  for (const auto& p : points) out.push_back(ToTranslation3d(p));
  return out;
}
/** Pose3d per point (AdvantageScope "Game piece" objects accept Pose3d or Translation3d arrays). */
inline std::vector<frc::Pose3d> ToPoses(const std::vector<Vec3>& points) {
  std::vector<frc::Pose3d> out;
  out.reserve(points.size());
  for (const auto& p : points) out.emplace_back(ToTranslation3d(p), frc::Rotation3d{});
  return out;
}
inline Pose2 FromPose2d(const frc::Pose2d& p) { return {p.X().value(), p.Y().value(), p.Rotation().Radians().value()}; }

/** Pauses WPILib's clock on construction and resumes it on destruction; Step advances it. */
class SteppedTime {
 public:
  SteppedTime() { frc::sim::PauseTiming(); }
  ~SteppedTime() { frc::sim::ResumeTiming(); }
  SteppedTime(const SteppedTime&) = delete;
  SteppedTime& operator=(const SteppedTime&) = delete;
  void Step(units::second_t dt) { frc::sim::StepTiming(dt); elapsed_ += dt; }
  /** Advance without waiting for notifiers to run (frc::sim::StepTimingAsync). */
  void StepAsync(units::second_t dt) { frc::sim::StepTimingAsync(dt); elapsed_ += dt; }
  units::second_t Elapsed() const { return elapsed_; }
  static bool IsPaused() { return frc::sim::IsTimingPaused(); }

 private:
  units::second_t elapsed_{0};
};

}  // namespace mayhem::sim
