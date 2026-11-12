#pragma once
#include <string_view>

namespace mayhem {
// Instrument your existing state machine, behavior tree, command, or custom auto.
// This interface does not own or restrict robot execution. Implement a sink using
// WPILib DataLog/NetworkTables or another telemetry transport.
class StateObserver {
 public:
  virtual ~StateObserver() = default;
  virtual void Enter(std::string_view machine, std::string_view state, double timestamp) = 0;
  virtual void Guard(std::string_view machine, std::string_view transition,
                     std::string_view input, bool result, double timestamp) = 0;
  virtual void Transition(std::string_view machine, std::string_view transition,
                          std::string_view from, std::string_view to, double timestamp) = 0;
  virtual void GoalProgress(std::string_view goal, double translationError,
                            double rotationError, bool reached, double timestamp) = 0;
};
}  // namespace mayhem
