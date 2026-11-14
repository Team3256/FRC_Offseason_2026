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
// `from` "*" (kAnyState) leaves every state except `to` and the `except` states.
struct MachineTransition {
  std::string_view id, from, to; bool (*guard)(const SignalSource&);
  const std::string_view* except = nullptr; std::size_t exceptCount = 0;
};
struct MachineView {
  std::string_view id, initial;
  const MachineState* states; std::size_t stateCount;
  const MachineTransition* transitions; std::size_t transitionCount;
  ManifestView manifest{};  // the machine's mayhem-machine JSON, for observers
  // Manifest `selection: "priority"`: rows are checked top to bottom every tick, a "*" row also
  // in its own target state, and the first true row decides; if that is the current state the
  // machine stays (no re-entry, no lower row fires).
  bool prioritySelect = false;
  // Declared manifest inputs (exporter 0.9.29+): logged by InputLogging::On, and the `event`
  // ones are hidden after the first step of Settle. Older headers leave this empty.
  const MachineInput* inputs = nullptr; std::size_t inputCount = 0;
};
// Attach option: also log every declared input (MachineView::inputs) each Tick/Settle.
enum class InputLogging { Off, On };
// Default bound on transitions taken by one Settle().
inline constexpr int kDefaultSettleSteps = 8;
template <class Def> class TypedMachine;
// Built-in input: real seconds since the active state was entered.
inline constexpr std::string_view kStateTimeSignal = "stateTime";
// MachineTransition::from for a transition checked in every state but its target.
inline constexpr std::string_view kAnyState = "*";

// Evaluates one machine graph. Each Tick(dt) adds dt to StateTime(), then evaluates the
// active state's outgoing guards in declaration order (declaration order is priority)
// and takes at most one transition. A "*" transition is checked, at its position, in every
// state except its target and its `except` states; observers see the actual source state.
// With MachineView::prioritySelect, a true transition to the current state (including a "*"
// row in its own target) keeps the state: Tick returns false and lower rows are not checked. Guards fail closed: a missing or mistyped input is
// false, including under Not. Settle(dt) repeats the step within one loop until no transition
// fires (time advances on the first step only; one-shot event inputs are visible to that first
// step only). The machine only decides state; robot code reads State()
// and drives mechanisms. Construction resolves states to indices and allocates;
// Tick/Settle/Reset never allocate. A graph whose initial state or a transition endpoint is
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
  // current state immediately. A non-null `clock` also times Tick() and Settle() (see
  // SetClock). InputLogging::On registers every declared input (RegisterInput) and reports
  // its value (Input) on the first step of each Tick/Settle. Allocates; call during
  // initialization.
  template <class Observer>
  void Attach(Observer& observer, Clock clock = nullptr, InputLogging inputs = InputLogging::Off) {
    static_assert(std::is_base_of_v<StateObserver, Observer>, "Attach a mayhem::StateObserver");
    observer_ = &observer;
    if (clock) SetClock(clock);
    if (!machine_.manifest.Empty()) detail::Describe(observer, machine_.manifest.Text());
    for (std::size_t i = 0; i < machine_.transitionCount; ++i)
      detail::RegisterGuard(observer, std::string{machine_.transitions[i].id});
    logInputs_ = inputs == InputLogging::On && machine_.inputs && machine_.inputCount > 0;
    if (logInputs_)
      for (std::size_t i = 0; i < machine_.inputCount; ++i) observer.RegisterInput(machine_.id, machine_.inputs[i]);
    if (current_ >= 0) observer_->Enter(machine_.id, State(), Now());
  }
  void Detach() { observer_ = nullptr; logInputs_ = false; }
  // Seconds (e.g. frc::Timer::GetFPGATimestamp) for observer timestamps and for Tick()/Settle()
  // without a dt, which use the time since the previous clocked call (or SetClock/Reset).
  void SetClock(Clock clock) { clock_ = clock; last_ = Now(); }

  // Enter the initial state with StateTime() = 0. Returns Valid().
  bool Reset() {
    if (!valid_) return false;
    lastTransition_ = -1;
    last_ = Now();
    Enter(initial_);
    return true;
  }
  // One step: returns true when a transition was taken. dt is real elapsed seconds; a negative
  // or non-finite dt does not advance StateTime() but guards are still evaluated.
  bool Tick(double dt) { return Step(dt, true); }
  // Tick with dt from the clock (0 without one: guards run, StateTime() does not advance).
  bool Tick() { return Tick(ClockDt()); }
  // Steps until no transition fires, at most `maxSteps` transitions, all within one loop: the
  // first step adds dt, later steps add nothing and see one-shot event inputs as their initial
  // value (a press is used once: X toggles INTAKING on, not on and off again). Returns the
  // transitions taken; Settled() says whether the machine ended stable.
  int Settle(double dt, int maxSteps = kDefaultSettleSteps) {
    if (current_ < 0) return 0;
    int taken = Step(dt, true) ? 1 : 0;
    settled_ = taken == 0;
    if (taken) {
      masking_ = true;
      while (taken < maxSteps) {
        if (!Step(0, false)) { settled_ = true; break; }
        ++taken;
      }
      masking_ = false;
    }
    return taken;
  }
  // Settle with dt from the clock.
  int Settle() { return Settle(ClockDt()); }
  // False when the last Settle() stopped at maxSteps (possibly a cycle).
  bool Settled() const { return settled_; }
  bool Valid() const { return valid_; }
  std::string_view State() const { return current_ >= 0 ? machine_.states[current_].id : std::string_view{}; }
  // Index into the MachineView's states (declaration order), or -1 before Reset / if invalid.
  int StateIndex() const { return current_; }
  double StateTime() const { return stateTime_; }
  // ID of the most recent transition taken, empty if none since Reset.
  std::string_view LastTransition() const { return lastTransition_ >= 0 ? machine_.transitions[lastTransition_].id : std::string_view{}; }
  const MachineView& View() const { return machine_; }
  // What guards see: "stateTime" is StateTime(); every other key comes from the
  // SignalSource or callback given at construction (an event input reads as its initial
  // value after the first step of a Settle).
  SignalValue Signal(std::string_view key) const override {
    if (key == kStateTimeSignal) return stateTime_;
    if (masking_)
      for (std::size_t i = 0; i < machine_.inputCount; ++i)
        if (machine_.inputs[i].event && machine_.inputs[i].key == key) return machine_.inputs[i].initial;
    if (source_) return source_->Signal(key);
    return callback_ ? callback_(key) : SignalValue{};
  }

 private:
  template <class Def> friend class TypedMachine;
  // Typed guards (TypedMachine): evaluate transition i from `context`, and reset one-shot events
  // after the first step of a loop.
  struct Typed {
    bool (*guard)(const void* context, std::size_t transition, double stateTime) = nullptr;
    void (*consume)(void* context) = nullptr;
    void* context = nullptr;
  };
  StateMachine(MachineView machine, const SignalSource& signals, Typed typed)
      : machine_(machine), source_(&signals), typed_(typed) { Build(); }
  bool Step(double dt, bool first) {
    if (current_ < 0) return false;
    if (std::isfinite(dt) && dt > 0) stateTime_ += dt;
    if (first && logInputs_ && observer_)
      for (std::size_t i = 0; i < machine_.inputCount; ++i)
        observer_->Input(machine_.id, machine_.inputs[i].key, Signal(machine_.inputs[i].key), Now());
    bool taken = false;
    for (std::size_t i = 0; i < machine_.transitionCount; ++i) {
      if (from_[i] == kAny ? skip_[i * machine_.stateCount + current_] : from_[i] != current_) continue;
      const auto& t = machine_.transitions[i];
      const bool accepted = typed_.guard ? typed_.guard(typed_.context, i, stateTime_) : t.guard && t.guard(*this);
      if (observer_) observer_->Guard(machine_.id, t.id, kGuardInput, accepted, Now());
      if (!accepted) continue;
      if (machine_.prioritySelect && to_[i] == current_) break;  // the current state wins
      lastTransition_ = static_cast<int>(i);
      if (observer_) observer_->Transition(machine_.id, t.id, State(), t.to, Now());
      Enter(to_[i]);
      taken = true;
      break;
    }
    if (first && typed_.consume) typed_.consume(typed_.context);
    return taken;
  }
  double ClockDt() {
    if (!clock_) return 0;
    const double now = clock_(), dt = now - last_;
    last_ = now;
    return dt;
  }
  int Index(std::string_view id) const {
    for (std::size_t i = 0; i < machine_.stateCount; ++i) if (machine_.states[i].id == id) return static_cast<int>(i);
    return -1;
  }
  void Build() {
    from_.resize(machine_.transitionCount); to_.resize(machine_.transitionCount);
    initial_ = Index(machine_.initial);
    valid_ = initial_ >= 0 && (machine_.transitionCount == 0 || machine_.transitions);
    for (std::size_t i = 0; valid_ && i < machine_.transitionCount; ++i) {
      const auto& t = machine_.transitions[i];
      from_[i] = t.from == kAnyState ? kAny : Index(t.from); to_[i] = Index(t.to);
      valid_ = from_[i] != -1 && to_[i] >= 0 && (t.exceptCount == 0 || (from_[i] == kAny && t.except));
      if (!valid_ || from_[i] != kAny) continue;
      // States a "*" transition is not checked in: `except`, and its target unless prioritySelect.
      if (skip_.empty()) skip_.assign(machine_.transitionCount * machine_.stateCount, 0);
      if (!machine_.prioritySelect) skip_[i * machine_.stateCount + to_[i]] = 1;
      for (std::size_t e = 0; valid_ && e < t.exceptCount; ++e) {
        const int s = Index(t.except[e]);
        valid_ = s >= 0;
        if (valid_) skip_[i * machine_.stateCount + s] = 1;
      }
    }
    if (!machine_.inputs) machine_.inputCount = 0;
    Reset();
  }
  void Enter(int state) {
    current_ = state; stateTime_ = 0;
    if (observer_) observer_->Enter(machine_.id, State(), Now());
  }
  double Now() const { return clock_ ? clock_() : 0; }
  MachineView machine_;
  const SignalSource* source_ = nullptr; SignalCallback callback_;
  Typed typed_{};
  static constexpr int kAny = -2;
  std::vector<int> from_, to_; std::vector<char> skip_;
  int initial_ = -1, current_ = -1, lastTransition_ = -1; bool valid_ = false;
  bool masking_ = false, settled_ = true, logInputs_ = false;
  double stateTime_ = 0, last_ = 0;
  StateObserver* observer_ = nullptr; Clock clock_ = nullptr;
};

// A generated machine with compile-time names: `mayhem_generated::<Machine>::Machine` is
// TypedMachine<Def>, where the exporter's Def has `enum class State` (one value per state, in
// declaration order), `struct Inputs` (one field per declared input, with the manifest type and
// initial value; `stateTime` is built in), and typed guards that read Inputs directly. A
// misspelled input or state does not compile. Set In() fields, then Tick() or Settle() each
// loop; event inputs are reset to their initial value after the first step. Not copyable or
// movable (the machine refers to its own Inputs).
template <class Def>
class TypedMachine {
 public:
  using State = typename Def::State;
  using Inputs = typename Def::Inputs;
  TypedMachine() : source_(inputs_), machine_(Def::view, source_, StateMachine::Typed{&Guard, &Consume, &inputs_}) {}
  TypedMachine(const TypedMachine&) = delete;
  TypedMachine& operator=(const TypedMachine&) = delete;

  // This loop's inputs. Event fields are reset after the first step of Tick/Settle.
  Inputs& In() { return inputs_; }
  const Inputs& In() const { return inputs_; }
  State Current() const { return static_cast<State>(machine_.StateIndex()); }
  bool Is(State state) const { return machine_.StateIndex() == static_cast<int>(state); }
  // The state's manifest ID (e.g. for a log): Name(State::SHOOTING) == "SHOOTING".
  static std::string_view Name(State state) {
    const auto i = static_cast<std::size_t>(state);
    return i < Def::view.stateCount ? Def::view.states[i].id : std::string_view{};
  }
  std::string_view StateName() const { return machine_.State(); }

  bool Tick(double dt) { return machine_.Tick(dt); }
  bool Tick() { return machine_.Tick(); }
  int Settle(double dt, int maxSteps = kDefaultSettleSteps) { return machine_.Settle(dt, maxSteps); }
  int Settle() { return machine_.Settle(); }
  bool Settled() const { return machine_.Settled(); }
  bool Reset() { return machine_.Reset(); }
  double StateTime() const { return machine_.StateTime(); }
  std::string_view LastTransition() const { return machine_.LastTransition(); }
  bool Valid() const { return machine_.Valid(); }
  void SetClock(Clock clock) { machine_.SetClock(clock); }
  template <class Observer>
  void Attach(Observer& observer, Clock clock = nullptr, InputLogging inputs = InputLogging::Off) {
    machine_.Attach(observer, clock, inputs);
  }
  void Detach() { machine_.Detach(); }
  // The string API (Signal(key), View(), ...) over the same machine.
  const StateMachine& Untyped() const { return machine_; }

 private:
  struct Source final : SignalSource {
    explicit Source(const Inputs& in) : in_(in) {}
    SignalValue Signal(std::string_view key) const override { return Def::Signal(in_, key); }
    const Inputs& in_;
  };
  static bool Guard(const void* context, std::size_t transition, double stateTime) {
    return Def::Guard(*static_cast<const Inputs*>(context), transition, stateTime);
  }
  static void Consume(void* context) { Def::ClearEvents(*static_cast<Inputs*>(context)); }
  Inputs inputs_{};
  Source source_;
  StateMachine machine_;
};
}  // namespace mayhem
