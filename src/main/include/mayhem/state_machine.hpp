#pragma once
// Runner for exported mayhem-machine graphs (project extensions.machines). C++17.
#include "mayhem/signals.hpp"
#include "mayhem/state_observer.hpp"
#include <functional>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

namespace mayhem {
struct MachineState { std::string_view id; };
struct MachineTransition { std::string_view id, from, to; bool (*guard)(const SignalSource&); };
struct MachineView {
  std::string_view id, initial;
  const MachineState* states; std::size_t stateCount;
  const MachineTransition* transitions; std::size_t transitionCount;
  ManifestView manifest{};  // the machine's mayhem-machine JSON, for observers
};
// Built-in input: real seconds since the active state was entered.
inline constexpr std::string_view kStateTimeSignal = "stateTime";

// Evaluates one machine graph. Each Tick(dt) adds dt to StateTime(), then evaluates the
// active state's outgoing guards in declaration order (declaration order is priority)
// and takes at most one transition. Guards fail closed: a missing or mistyped input is
// false, including under Not. The machine only decides state; robot code reads State()
// and drives mechanisms. Construction resolves states to indices and allocates;
// Tick/Reset never allocate. A graph whose initial state or a transition endpoint is
// missing is !Valid(): State() is empty and Tick does nothing.
class StateMachine final : public SignalSource {
 public:
  using SignalCallback = std::function<SignalValue(std::string_view)>;
  // `signals` must outlive the machine.
  StateMachine(MachineView machine, const SignalSource& signals) : machine_(machine), source_(&signals) { Build(); }
  StateMachine(MachineView machine, SignalCallback signals) : machine_(machine), callback_(std::move(signals)) { Build(); }
  StateMachine(const StateMachine&) = delete;
  StateMachine& operator=(const StateMachine&) = delete;

  // Describe the manifest, register every transition guard (input "condition") when the
  // observer supports it, and report Enter/Guard/Transition from now on. Reports the
  // current state immediately. Allocates; call during initialization.
  template <class Observer>
  void Attach(Observer& observer, Clock clock = nullptr) {
    static_assert(std::is_base_of_v<StateObserver, Observer>, "Attach a mayhem::StateObserver");
    observer_ = &observer; clock_ = clock;
    if (!machine_.manifest.Empty()) detail::Describe(observer, machine_.manifest.Text());
    for (std::size_t i = 0; i < machine_.transitionCount; ++i)
      detail::RegisterGuard(observer, std::string{machine_.transitions[i].id});
    if (current_ >= 0) observer_->Enter(machine_.id, State(), Now());
  }
  void Detach() { observer_ = nullptr; }

  // Enter the initial state with StateTime() = 0. Returns Valid().
  bool Reset() {
    if (!valid_) return false;
    lastTransition_ = -1;
    Enter(initial_);
    return true;
  }
  // Returns true when a transition was taken. dt is real elapsed seconds; a negative or
  // non-finite dt does not advance StateTime() but guards are still evaluated.
  bool Tick(double dt) {
    if (current_ < 0) return false;
    if (std::isfinite(dt) && dt > 0) stateTime_ += dt;
    for (std::size_t i = 0; i < machine_.transitionCount; ++i) {
      if (from_[i] != current_) continue;
      const auto& t = machine_.transitions[i];
      const bool accepted = t.guard && t.guard(*this);
      if (observer_) observer_->Guard(machine_.id, t.id, kGuardInput, accepted, Now());
      if (!accepted) continue;
      lastTransition_ = static_cast<int>(i);
      if (observer_) observer_->Transition(machine_.id, t.id, t.from, t.to, Now());
      Enter(to_[i]);
      return true;
    }
    return false;
  }
  bool Valid() const { return valid_; }
  std::string_view State() const { return current_ >= 0 ? machine_.states[current_].id : std::string_view{}; }
  // Index into the MachineView's states (declaration order), or -1 before Reset / if invalid.
  int StateIndex() const { return current_; }
  double StateTime() const { return stateTime_; }
  // ID of the most recent transition taken, empty if none since Reset.
  std::string_view LastTransition() const { return lastTransition_ >= 0 ? machine_.transitions[lastTransition_].id : std::string_view{}; }
  const MachineView& View() const { return machine_; }
  // What guards see: "stateTime" is StateTime(); every other key comes from the
  // SignalSource or callback given at construction.
  SignalValue Signal(std::string_view key) const override {
    if (key == kStateTimeSignal) return stateTime_;
    if (source_) return source_->Signal(key);
    return callback_ ? callback_(key) : SignalValue{};
  }

 private:
  int Index(std::string_view id) const {
    for (std::size_t i = 0; i < machine_.stateCount; ++i) if (machine_.states[i].id == id) return static_cast<int>(i);
    return -1;
  }
  void Build() {
    from_.resize(machine_.transitionCount); to_.resize(machine_.transitionCount);
    initial_ = Index(machine_.initial);
    valid_ = initial_ >= 0 && (machine_.transitionCount == 0 || machine_.transitions);
    for (std::size_t i = 0; valid_ && i < machine_.transitionCount; ++i) {
      from_[i] = Index(machine_.transitions[i].from); to_[i] = Index(machine_.transitions[i].to);
      valid_ = from_[i] >= 0 && to_[i] >= 0;
    }
    Reset();
  }
  void Enter(int state) {
    current_ = state; stateTime_ = 0;
    if (observer_) observer_->Enter(machine_.id, State(), Now());
  }
  double Now() const { return clock_ ? clock_() : 0; }
  MachineView machine_;
  const SignalSource* source_ = nullptr; SignalCallback callback_;
  std::vector<int> from_, to_;
  int initial_ = -1, current_ = -1, lastTransition_ = -1; bool valid_ = false;
  double stateTime_ = 0;
  StateObserver* observer_ = nullptr; Clock clock_ = nullptr;
};
}  // namespace mayhem
