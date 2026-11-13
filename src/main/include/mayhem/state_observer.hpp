#pragma once
#include "mayhem/signals.hpp"
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>

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
  // Optional input logging (StateMachine::Attach with InputLogging::On). RegisterInput is called
  // once per declared input during Attach (allocate there); Input reports each declared input's
  // value on the first step of every Tick/Settle and must not allocate. Default: ignored.
  virtual void RegisterInput(std::string_view machine, const MachineInput& input) { (void)machine; (void)input; }
  virtual void Input(std::string_view machine, std::string_view key, const SignalValue& value, double timestamp) {
    (void)machine; (void)key; (void)value; (void)timestamp;
  }
};
// Seconds for observer timestamps. Without a clock, StateMachine and AutoRunner pass 0,
// which WpiLogObserver records as "now" and NtStateObserver ignores.
using Clock = double (*)();
// Input name used when a runner reports a whole transition condition as one guard.
inline constexpr std::string_view kGuardInput = "condition";
// Machine name AutoRunner reports to observers by default.
inline constexpr std::string_view kAutonomousMachine = "Autonomous";
// Transition IDs are unique within a routine but not across routines, so the
// combined Autonomous manifest and all auto observations use "<routine>/<transition>".
inline std::string QualifiedTransitionId(std::string_view routine, std::string_view transition) {
  std::string id; id.reserve(routine.size() + 1 + transition.size());
  id.append(routine.data(), routine.size()).append(1, '/').append(transition.data(), transition.size());
  return id;
}
namespace detail {
template <class T, class = void> struct HasDescribe : std::false_type {};
template <class T>
struct HasDescribe<T, std::void_t<decltype(std::declval<T&>().Describe(std::string_view{}))>> : std::true_type {};
template <class T, class = void> struct HasRegisterGuard : std::false_type {};
template <class T>
struct HasRegisterGuard<T, std::void_t<decltype(std::declval<T&>().RegisterGuard(std::string{}, std::string{}))>> : std::true_type {};
// Describe/RegisterGuard are not part of StateObserver; call them when the sink has them.
template <class O> void Describe(O& observer, std::string_view manifest) {
  if constexpr (HasDescribe<O>::value) { if (!manifest.empty()) observer.Describe(manifest); }
  else { (void)observer; (void)manifest; }
}
template <class O> void RegisterGuard(O& observer, std::string transition) {
  if constexpr (HasRegisterGuard<O>::value) observer.RegisterGuard(std::move(transition), std::string{kGuardInput});
  else { (void)observer; (void)transition; }
}
}  // namespace detail
}  // namespace mayhem
