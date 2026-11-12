#pragma once
#include "mayhem/pose_controller.hpp"
#include "mayhem/signals.hpp"
#include "mayhem/state_observer.hpp"
#include "mayhem/version.hpp"
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
struct AutoNode {
  std::string_view id; NodeKind kind; Goal goal; double timeout;
  std::string_view action; Constraints constraints;
  const std::string_view* during = nullptr; std::size_t duringCount = 0;
  // Pass-through anchor: the robot hands off here instead of stopping. `next` is
  // the pose node reached through this node's single "Always" transition, or -1.
  int next = -1; bool intermediate = false;
};
struct AutoTransition { std::string_view id, from, to; bool (*guard)(const RobotHooks&); };
struct RoutineView {
  std::string_view id, initial; const AutoNode* nodes; std::size_t nodeCount;
  const AutoTransition* transitions; std::size_t transitionCount;
};
// One exported routine: chooser label, graph, and blue-origin start pose.
struct RoutineInfo { std::string_view name; RoutineView routine; Pose startPose; };
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
  AutoRunner(RoutineView routine, RobotHooks& hooks, RunnerOptions options = {})
      : routine_(routine), hooks_(hooks), options_(options) {
    std::size_t maximum=0;
    for(std::size_t i=0;i<routine_.nodeCount;i++) maximum=std::max(maximum,routine_.nodes[i].duringCount);
    duringSucceeded_.reserve(maximum);
    follower_.SetMaxDt(options_.maxDt);
  }
  // Report Enter/Guard/Transition/GoalProgress for this routine to `observer`. When the
  // observer has Describe(json) and RegisterGuard(transition, input) (WpiLogObserver,
  // NtStateObserver, CombinedObserver), the manifest is described and every transition
  // registered as "<routine>/<transition>" with input "condition". Allocates; call
  // during initialization. The observer must outlive the runner or be Detach()ed.
  template <class Observer>
  void Attach(Observer& observer, ManifestView manifest = {}, Clock clock = nullptr,
              std::string_view machine = kAutonomousMachine) {
    static_assert(std::is_base_of_v<StateObserver, Observer>, "Attach a mayhem::StateObserver");
    observer_=&observer; clock_=clock; machine_.assign(machine.data(),machine.size());
    qualified_.clear(); qualified_.reserve(routine_.transitionCount);
    for(std::size_t i=0;i<routine_.transitionCount;i++) qualified_.push_back(QualifiedTransitionId(routine_.id,routine_.transitions[i].id));
    if(!manifest.Empty()) detail::Describe(observer,manifest.Text());
    for(const auto& id:qualified_) detail::RegisterGuard(observer,id);
  }
  void Detach() {observer_=nullptr;}
  bool Start(Velocity measured = {}) {
    if(status_ == RunStatus::Running) Leave(true);
    current_ = Find(routine_.initial); elapsed_ = 0; follower_.Restart(measured);
    last_ = {}; lastCommand_ = {}; overruns_ = 0;
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
  Velocity Step(Pose pose, Velocity measured, double dt) {
    elapsed_ += dt;
    if(current_->kind == NodeKind::Finish) {status_=RunStatus::Completed;return {};}
    if(!std::isfinite(current_->timeout) || elapsed_ >= current_->timeout) return Fail();
    for(std::size_t i=0;i<current_->duringCount;i++) {
      if(duringSucceeded_[i]) continue;
      const auto result=hooks_.TickAction(current_->during[i]);
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
        if(current_->goal.markers[i].kind==MarkerKind::Event) hooks_.PathEvent(current_->goal.markers[i].key);
    } else {
      const auto result=actionSucceeded_ ? ActionStatus::Succeeded : hooks_.TickAction(current_->action);
      if(result==ActionStatus::Failed) return Fail(); actionSucceeded_=result==ActionStatus::Succeeded; ready=actionSucceeded_;
    }
    if(!ready) return command;
    for(std::size_t i=0;i<routine_.transitionCount;i++) {
      const auto& transition=routine_.transitions[i]; if(transition.from!=current_->id) continue;
      const bool accepted=transition.guard && transition.guard(hooks_); hooks_.GuardEvaluated(transition.id,accepted);
      if(observer_) observer_->Guard(machine_,Qualified(i),kGuardInput,accepted,Now());
      if(!accepted) continue;
      const auto* next=Find(transition.to); if(!next) return Fail();
      const auto previous=current_->id; Leave(false);current_=next;elapsed_=0;
      if(!(handedOff && next->kind==NodeKind::Pose)) follower_.Restart(command);
      hooks_.Transitioned(transition.id,previous,next->id);
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
    hooks_.StateEntered(current_->id);
    if(observer_) observer_->Enter(machine_,current_->id,Now());
    if(current_->kind==NodeKind::Finish) {status_=RunStatus::Completed;return;}
    if(current_->kind==NodeKind::Action||current_->kind==NodeKind::Custom) hooks_.BeginAction(current_->action);
    for(std::size_t i=0;i<current_->duringCount;i++) hooks_.BeginAction(current_->during[i]);
  }
  void Leave(bool cancelled) {
    if(!current_) return;
    if(current_->kind==NodeKind::Action||current_->kind==NodeKind::Custom) hooks_.EndAction(current_->action,cancelled);
    for(std::size_t i=0;i<current_->duringCount;i++) hooks_.EndAction(current_->during[i],cancelled);
  }
  Velocity Fail() {Leave(true);status_=RunStatus::Failed;lastCommand_={};return {};}
  static constexpr std::size_t kMaxLookahead=64;
  RoutineView routine_;RobotHooks& hooks_;RunnerOptions options_;PathFollower follower_;const AutoNode* current_=nullptr;
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
