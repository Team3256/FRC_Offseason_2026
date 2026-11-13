#pragma once
#include "mayhem/pose_controller.hpp"
#include "mayhem/signals.hpp"
#include "mayhem/state_observer.hpp"
#include "mayhem/version.hpp"
#include <functional>
#include <string>
#include <string_view>
#include <type_traits>
#include <vector>

namespace mayhem {
enum class ActionStatus { Running, Succeeded, Failed };
// Optional integration layer. Robot code owns scheduling, mechanisms, custom logic,
// and all signal acquisition. The controller and observer can be used independently.
class RobotHooks : public SignalSource {
 public:
  // Signal(key) is inherited from SignalSource.
  virtual void BeginAction(std::string_view) {}
  virtual ActionStatus TickAction(std::string_view) { return ActionStatus::Running; }
  virtual void EndAction(std::string_view, bool) {}
  virtual void StateEntered(std::string_view) {}
  virtual void GuardEvaluated(std::string_view, bool) {}
  virtual void Transitioned(std::string_view, std::string_view, std::string_view) {}
  // A path event trigger was passed (fires once per path run, in marker order).
  virtual void PathEvent(std::string_view) {}
};
enum class NodeKind { Pose, Action, Custom, Finish };
// What a step's watchdog expiry means. Fail (default) fails the routine; Succeed (action and
// custom steps, project `onTimeout: "succeed"`) ends the step as done, for fixed windows such
// as "aim and shoot for 3.5 s". Pose steps always fail: running out of time is not arrival.
enum class TimeoutOutcome { Fail, Succeed };
struct AutoNode {
  std::string_view id; NodeKind kind; Goal goal; double timeout;
  std::string_view action; Constraints constraints;
  const std::string_view* during = nullptr; std::size_t duringCount = 0;
  // Pass-through anchor: the robot hands off here instead of stopping. `next` is
  // the pose node reached through this node's single "Always" transition, or -1.
  int next = -1; bool intermediate = false;
  TimeoutOutcome onTimeout = TimeoutOutcome::Fail;
};
struct AutoTransition { std::string_view id, from, to; bool (*guard)(const RobotHooks&); };
struct RoutineView {
  std::string_view id, initial; const AutoNode* nodes; std::size_t nodeCount;
  const AutoTransition* transitions; std::size_t transitionCount;
  // Signal keys the transition guards read (exporter 0.9.30+; empty in older headers).
  const std::string_view* signals = nullptr; std::size_t signalCount = 0;
};
// One exported routine: chooser label, graph, blue-origin start pose, and the path preview: the
// poses the editor's path-only playback drives through (about every 0.1 m), blue origin.
struct RoutineInfo {
  std::string_view name; RoutineView routine; Pose startPose;
  const Pose* preview = nullptr; std::size_t previewCount = 0;
};
// A robot action bound by ID (AutoRunner::Bind): `begin` when its step is entered (or when a
// path event with this key fires), `tick` every loop until it returns Succeeded or Failed
// (empty: Succeeded at once), `end(cancelled)` once when the step exits.
struct Action {
  std::function<void()> begin;
  std::function<ActionStatus()> tick;
  std::function<void(bool cancelled)> end;
};
// An action that does `fn` once and succeeds: Bind("superstructure.intake", Instant(...)).
inline Action Instant(std::function<void()> fn) { return Action{std::move(fn), {}, {}}; }
// One ID the routine uses that nothing is bound to (AutoRunner::Unbound). `kind` is "action"
// (action/custom step), "during", "event" (path event marker) or "signal" (guard input).
struct UnboundId { std::string_view kind, id, node; };
// Look up an exported routine by chooser label or routine ID; nullptr if absent.
template <class Routines>
const RoutineInfo* FindRoutine(const Routines& routines, std::string_view nameOrId) {
  for (const RoutineInfo& r : routines) if (r.name == nameOrId || r.routine.id == nameOrId) return &r;
  return nullptr;
}
enum class RunStatus { Idle, Running, Completed, Failed, Cancelled };
enum class DtOverrun { Clamp, Fail };
struct RunnerOptions {
  // Longest dt the follower integrates in one tick. A longer loop (first enabled
  // loop, GC, log flush) is integrated as maxDt with DtOverrun::Clamp, or fails the
  // routine with DtOverrun::Fail. Node watchdogs always count the real dt.
  double maxDt = PathFollower::kDefaultMaxDt;
  DtOverrun overrun = DtOverrun::Clamp;
};
class AutoRunner {
 public:
  // Robot callbacks through RobotHooks; IDs bound with Bind/BindSignal take precedence.
  AutoRunner(RoutineView routine, RobotHooks& hooks, RunnerOptions options = {})
      : AutoRunner(routine, &hooks, options) {}
  // Robot callbacks only through Bind/BindSignal. An unbound action fails its step; check
  // Unbound() at startup.
  explicit AutoRunner(RoutineView routine, RunnerOptions options = {}) : AutoRunner(routine, nullptr, options) {}
  AutoRunner(const AutoRunner&) = delete;
  AutoRunner& operator=(const AutoRunner&) = delete;

  // Bind an action, custom step or path event ID (allocates; call at startup). Rebinding an ID
  // replaces it. Returns *this for chaining.
  AutoRunner& Bind(std::string_view id, Action action) {
    for(auto& b:actions_) if(b.id==id) {b.action=std::move(action);return *this;}
    actions_.push_back({std::string{id},std::move(action)});
    return *this;
  }
  // Bind a guard input read by transitions (e.g. "superstructure.ready"). Once any signal is
  // bound, guards receive an internal RobotHooks (bound signals first, then your hooks'
  // Signal), so a hand-written guard must not downcast the RobotHooks it is given.
  AutoRunner& BindSignal(std::string_view key, std::function<SignalValue()> value) {
    for(auto& b:signals_) if(b.key==key) {b.value=std::move(value);return *this;}
    signals_.push_back({std::string{key},std::move(value)});
    return *this;
  }
  // Every action, during action, path event and guard signal the routine uses that has no
  // binding, each ID once (with the first node using it). Call at startup and fail loudly when
  // it is not empty. With RobotHooks, the list names the IDs the hooks must handle themselves.
  std::vector<UnboundId> Unbound() const {
    std::vector<UnboundId> missing;
    auto add=[&](std::string_view kind,std::string_view id,std::string_view node,bool bound){
      if(bound||id.empty()) return;
      for(const auto& m:missing) if(m.kind==kind&&m.id==id) return;
      missing.push_back({kind,id,node});
    };
    for(std::size_t i=0;i<routine_.nodeCount;i++) {
      const auto& n=routine_.nodes[i];
      if(n.kind==NodeKind::Action||n.kind==NodeKind::Custom) add("action",n.action,n.id,FindAction(n.action));
      for(std::size_t d=0;d<n.duringCount;d++) add("during",n.during[d],n.id,FindAction(n.during[d]));
      for(std::size_t m=0;n.kind==NodeKind::Pose&&n.goal.markers&&m<n.goal.markerCount;m++)
        if(n.goal.markers[m].kind==MarkerKind::Event) add("event",n.goal.markers[m].key,n.id,FindAction(n.goal.markers[m].key));
    }
    for(std::size_t i=0;routine_.signals&&i<routine_.signalCount;i++) add("signal",routine_.signals[i],{},FindSignal(routine_.signals[i]));
    return missing;
  }
  // Unbound() as text, one "<kind> <id> (node <node>)" per line; empty when all are bound.
  std::string UnboundReport() const {
    std::string text;
    for(const auto& m:Unbound()) {
      text.append(m.kind.data(),m.kind.size()).append(" ").append(m.id.data(),m.id.size());
      if(!m.node.empty()) text.append(" (node ").append(m.node.data(),m.node.size()).append(")");
      text.append("\n");
    }
    return text;
  }
  // Seconds for observer timestamps and for Tick(pose, measured) without a dt.
  void SetClock(Clock clock) {clock_=clock; lastTick_=Now();}
  // Report Enter/Guard/Transition/GoalProgress for this routine to `observer`. When the
  // observer has Describe(json) and RegisterGuard(transition, input) (WpiLogObserver,
  // NtStateObserver, CombinedObserver), the manifest is described and every transition
  // registered as "<routine>/<transition>" with input "condition". Allocates; call
  // during initialization. The observer must outlive the runner or be Detach()ed.
  template <class Observer>
  void Attach(Observer& observer, ManifestView manifest = {}, Clock clock = nullptr,
              std::string_view machine = kAutonomousMachine) {
    static_assert(std::is_base_of_v<StateObserver, Observer>, "Attach a mayhem::StateObserver");
    observer_=&observer; if(clock) SetClock(clock); machine_.assign(machine.data(),machine.size());
    qualified_.clear(); qualified_.reserve(routine_.transitionCount);
    for(std::size_t i=0;i<routine_.transitionCount;i++) qualified_.push_back(QualifiedTransitionId(routine_.id,routine_.transitions[i].id));
    if(!manifest.Empty()) detail::Describe(observer,manifest.Text());
    for(const auto& id:qualified_) detail::RegisterGuard(observer,id);
  }
  void Detach() {observer_=nullptr;}
  bool Start(Velocity measured = {}) {
    if(status_ == RunStatus::Running) Leave(true);
    current_ = Find(routine_.initial); elapsed_ = 0; follower_.Restart(measured);
    last_ = {}; lastCommand_ = {}; overruns_ = 0; lastTick_ = Now();
    if(!current_ || !std::isfinite(options_.maxDt) || options_.maxDt <= 0) {current_=nullptr;status_=RunStatus::Failed; return false;}
    status_=RunStatus::Running; Enter(); return true;
  }
  // dt == 0 (Execute in the same timestamp as Initialize) changes nothing and repeats the
  // last command. Negative or non-finite dt fails. See RunnerOptions for dt > maxDt.
  Velocity Tick(Pose pose, Velocity measured, double dt) {
    if(status_ != RunStatus::Running || !current_) return {};
    if(!std::isfinite(dt) || dt < 0) return Fail();
    if(dt == 0) return lastCommand_;
    if(dt > options_.maxDt) { ++overruns_; if(options_.overrun == DtOverrun::Fail) return Fail(); }
    lastCommand_ = Step(pose, measured, dt);
    return lastCommand_;
  }
  // Tick with dt from the clock (SetClock, or Attach with a clock): the time since Start() or the
  // previous clocked tick. Without a clock the routine fails, so a missing clock is not a stall.
  Velocity Tick(Pose pose, Velocity measured) {
    if(status_ != RunStatus::Running || !current_) return {};
    if(!clock_) return Fail();
    const double now=clock_(), dt=now-lastTick_;
    lastTick_=now;
    return Tick(pose, measured, dt);
  }
  void Cancel() {if(status_==RunStatus::Running) Leave(true);status_=RunStatus::Cancelled;}
  RunStatus Status() const {return status_;}
  std::string_view ActiveState() const {return current_?current_->id:std::string_view{};}
  const AutoNode* ActiveNode() const {return current_;}
  // The goal being driven to, or nullptr outside pose states. Coordinates are those of
  // the RoutineView (already alliance-transformed when built from AllianceRoutine).
  const Goal* ActiveGoal() const {return current_&&current_->kind==NodeKind::Pose?&current_->goal:nullptr;}
  // Follower output from the latest pose tick of the active state (errors, reached,
  // handoff, progress). Reset to defaults on every state entry.
  const ControlResult& LastResult() const {return last_;}
  // Real seconds in the active state (the watchdog clock).
  double StateTime() const {return elapsed_;}
  // Ticks since Start() whose dt exceeded RunnerOptions::maxDt.
  std::size_t Overruns() const {return overruns_;}
  const RoutineView& Routine() const {return routine_;}
  const RunnerOptions& Options() const {return options_;}
 private:
  // Routes RobotHooks calls to bound actions and signals first, then to the user's hooks.
  class Dispatch final : public RobotHooks {
   public:
    Dispatch(AutoRunner& runner, RobotHooks* user) : runner_(runner), user_(user) {}
    SignalValue Signal(std::string_view key) const override {
      if(const auto* s=runner_.FindSignal(key)) return s->value ? s->value() : SignalValue{};
      return user_ ? user_->Signal(key) : SignalValue{};
    }
    void BeginAction(std::string_view id) override {
      if(const auto* a=runner_.FindAction(id)) {if(a->begin) a->begin();}
      else if(user_) user_->BeginAction(id);
    }
    ActionStatus TickAction(std::string_view id) override {
      if(const auto* a=runner_.FindAction(id)) return a->tick ? a->tick() : ActionStatus::Succeeded;
      return user_ ? user_->TickAction(id) : ActionStatus::Failed;
    }
    void EndAction(std::string_view id, bool cancelled) override {
      if(const auto* a=runner_.FindAction(id)) {if(a->end) a->end(cancelled);}
      else if(user_) user_->EndAction(id,cancelled);
    }
    void StateEntered(std::string_view id) override {if(user_) user_->StateEntered(id);}
    void GuardEvaluated(std::string_view id, bool accepted) override {if(user_) user_->GuardEvaluated(id,accepted);}
    void Transitioned(std::string_view id, std::string_view from, std::string_view to) override {if(user_) user_->Transitioned(id,from,to);}
    // A bound path event runs its `begin` (an Instant's function).
    void PathEvent(std::string_view key) override {
      if(const auto* a=runner_.FindAction(key)) {if(a->begin) a->begin();}
      else if(user_) user_->PathEvent(key);
    }
    RobotHooks* User() const {return user_;}
   private:
    AutoRunner& runner_; RobotHooks* user_;
  };
  // Guards receive the robot's own hooks object (hand-written guards may downcast it) unless
  // signals are bound, in which case they read through the dispatcher.
  const RobotHooks& GuardHooks() const {
    if(dispatch_.User()&&signals_.empty()) return *dispatch_.User();
    return dispatch_;
  }
  struct ActionBinding {std::string id; Action action;};
  struct SignalBinding {std::string key; std::function<SignalValue()> value;};
  AutoRunner(RoutineView routine, RobotHooks* hooks, RunnerOptions options)
      : routine_(routine), dispatch_(*this, hooks), options_(options) {
    std::size_t maximum=0;
    for(std::size_t i=0;i<routine_.nodeCount;i++) maximum=std::max(maximum,routine_.nodes[i].duringCount);
    duringSucceeded_.reserve(maximum);
    follower_.SetMaxDt(options_.maxDt);
  }
  const Action* FindAction(std::string_view id) const {for(const auto& b:actions_) if(b.id==id) return &b.action;return nullptr;}
  const SignalBinding* FindSignal(std::string_view key) const {for(const auto& b:signals_) if(b.key==key) return &b;return nullptr;}
  Velocity Step(Pose pose, Velocity measured, double dt) {
    elapsed_ += dt;
    if(current_->kind == NodeKind::Finish) {status_=RunStatus::Completed;return {};}
    bool timedOut=false;
    if(!std::isfinite(current_->timeout) || elapsed_ >= current_->timeout) {
      // onTimeout Succeed: the action/custom step is done when its window ends.
      const bool action=current_->kind==NodeKind::Action||current_->kind==NodeKind::Custom;
      if(!action || current_->onTimeout!=TimeoutOutcome::Succeed || !std::isfinite(current_->timeout)) return Fail();
      timedOut=true;
    }
    for(std::size_t i=0;i<current_->duringCount;i++) {
      if(duringSucceeded_[i]) continue;
      const auto result=dispatch_.TickAction(current_->during[i]);
      if(result==ActionStatus::Failed) return Fail();
      duringSucceeded_[i]=result==ActionStatus::Succeeded;
    }
    Velocity command; bool ready=false;
    bool handedOff=false;
    if(current_->kind==NodeKind::Pose) {
      std::array<const Goal*,kMaxLookahead> anchors{}; std::array<double,kMaxLookahead> caps{}; std::size_t count=0;
      const AutoNode* walk=current_; bool final=true;
      while(walk && count<anchors.size()) {
        caps[count]=walk->constraints.maxVelocity;
        anchors[count++]=&walk->goal;
        if(!walk->intermediate) {final=true;break;}
        final=false;
        walk=walk->next>=0 && static_cast<std::size_t>(walk->next)<routine_.nodeCount ? &routine_.nodes[walk->next] : nullptr;
        if(walk==current_) break;
      }
      const auto result=follower_.Calculate(pose,measured,anchors.data(),count,final,current_->constraints,dt,caps.data());
      if(!result.valid) return Fail();
      last_=result; command=result.command;ready=result.reached;handedOff=result.handoff;
      if(observer_) observer_->GoalProgress(current_->id,result.translationError,result.rotationError,result.reached,Now());
      for(std::size_t i=result.eventsBegin;i<result.eventsEnd && i<current_->goal.markerCount;i++)
        if(current_->goal.markers[i].kind==MarkerKind::Event) dispatch_.PathEvent(current_->goal.markers[i].key);
    } else {
      const auto result=actionSucceeded_||timedOut ? ActionStatus::Succeeded : dispatch_.TickAction(current_->action);
      if(result==ActionStatus::Failed) return Fail(); actionSucceeded_=result==ActionStatus::Succeeded; ready=actionSucceeded_;
    }
    if(!ready) return command;
    for(std::size_t i=0;i<routine_.transitionCount;i++) {
      const auto& transition=routine_.transitions[i]; if(transition.from!=current_->id) continue;
      const bool accepted=transition.guard && transition.guard(GuardHooks()); dispatch_.GuardEvaluated(transition.id,accepted);
      if(observer_) observer_->Guard(machine_,Qualified(i),kGuardInput,accepted,Now());
      if(!accepted) continue;
      const auto* next=Find(transition.to); if(!next) return Fail();
      const auto previous=current_->id; Leave(false);current_=next;elapsed_=0;
      if(!(handedOff && next->kind==NodeKind::Pose)) follower_.Restart(command);
      dispatch_.Transitioned(transition.id,previous,next->id);
      if(observer_) observer_->Transition(machine_,Qualified(i),previous,next->id,Now());
      Enter();
      return current_->kind==NodeKind::Pose ? command : Velocity{};
    }
    return command;
  }
  const AutoNode* Find(std::string_view id) const {for(std::size_t i=0;i<routine_.nodeCount;i++) if(routine_.nodes[i].id==id) return &routine_.nodes[i];return nullptr;}
  std::string_view Qualified(std::size_t i) const {return i<qualified_.size()?std::string_view{qualified_[i]}:routine_.transitions[i].id;}
  double Now() const {return clock_?clock_():0;}
  void Enter() {
    actionSucceeded_=false; last_={};
    duringSucceeded_.assign(current_->duringCount,false);
    dispatch_.StateEntered(current_->id);
    if(observer_) observer_->Enter(machine_,current_->id,Now());
    if(current_->kind==NodeKind::Finish) {status_=RunStatus::Completed;return;}
    if(current_->kind==NodeKind::Action||current_->kind==NodeKind::Custom) dispatch_.BeginAction(current_->action);
    for(std::size_t i=0;i<current_->duringCount;i++) dispatch_.BeginAction(current_->during[i]);
  }
  void Leave(bool cancelled) {
    if(!current_) return;
    if(current_->kind==NodeKind::Action||current_->kind==NodeKind::Custom) dispatch_.EndAction(current_->action,cancelled);
    for(std::size_t i=0;i<current_->duringCount;i++) dispatch_.EndAction(current_->during[i],cancelled);
  }
  Velocity Fail() {Leave(true);status_=RunStatus::Failed;lastCommand_={};return {};}
  static constexpr std::size_t kMaxLookahead=64;
  RoutineView routine_;Dispatch dispatch_;RunnerOptions options_;
  std::vector<ActionBinding> actions_;std::vector<SignalBinding> signals_;double lastTick_=0;PathFollower follower_;const AutoNode* current_=nullptr;
  std::vector<bool> duringSucceeded_;
  bool actionSucceeded_=false;
  double elapsed_=0;RunStatus status_=RunStatus::Idle;
  ControlResult last_;Velocity lastCommand_;std::size_t overruns_=0;
  StateObserver* observer_=nullptr;Clock clock_=nullptr;std::string machine_;std::vector<std::string> qualified_;
};
// Alliance transforms. Flip: rotate the field 180 degrees (opposite alliance).
// Mirror: reflect across the long centerline (left/right variant).
inline Pose FlipPose(Pose pose,double fieldLength,double fieldWidth) {return {fieldLength-pose.x,fieldWidth-pose.y,Wrap(pose.heading+3.14159265358979323846)};}
inline Pose MirrorPose(Pose pose,double fieldWidth) {return {pose.x,fieldWidth-pose.y,Wrap(-pose.heading)};}
} // namespace mayhem
