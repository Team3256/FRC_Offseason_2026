#pragma once
#include "mayhem/pose_controller.hpp"
#include <initializer_list>
#include <optional>
#include <string_view>
#include <variant>
#include <vector>

namespace mayhem {
using SignalValue = std::variant<std::monostate, bool, double, std::string_view>;
enum class CompareOp { Eq, Gt, Lt };
inline std::optional<bool> Compare(SignalValue actual, CompareOp op, SignalValue wanted) {
  if (actual.index() == 0 || actual.index() != wanted.index()) return std::nullopt;
  if (std::holds_alternative<double>(actual) && (!std::isfinite(std::get<double>(actual)) || !std::isfinite(std::get<double>(wanted)))) return std::nullopt;
  if (op == CompareOp::Eq) return actual == wanted;
  if (!std::holds_alternative<double>(actual)) return std::nullopt;
  return op == CompareOp::Gt ? std::get<double>(actual) > std::get<double>(wanted) : std::get<double>(actual) < std::get<double>(wanted);
}
inline std::optional<bool> Not(std::optional<bool> value) { return value ? std::optional<bool>{!*value} : std::nullopt; }
inline std::optional<bool> All(std::initializer_list<std::optional<bool>> values) {
  bool missing = false; for(auto v:values) {if(v && !*v) return false; if(!v) missing=true;} return missing ? std::nullopt : std::optional<bool>{true};
}
inline std::optional<bool> Any(std::initializer_list<std::optional<bool>> values) {
  bool missing = false; for(auto v:values) {if(v && *v) return true; if(!v) missing=true;} return missing ? std::nullopt : std::optional<bool>{false};
}
enum class ActionStatus { Running, Succeeded, Failed };
// Optional integration layer. Robot code owns scheduling, mechanisms, custom logic,
// and all signal acquisition. The controller and observer can be used independently.
class RobotHooks {
 public:
  virtual ~RobotHooks() = default;
  virtual SignalValue Signal(std::string_view) const { return {}; }
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
enum class RunStatus { Idle, Running, Completed, Failed, Cancelled };
class AutoRunner {
 public:
  AutoRunner(RoutineView routine, RobotHooks& hooks) : routine_(routine), hooks_(hooks) {
    std::size_t maximum=0;
    for(std::size_t i=0;i<routine_.nodeCount;i++) maximum=std::max(maximum,routine_.nodes[i].duringCount);
    duringSucceeded_.reserve(maximum);
  }
  bool Start(Velocity measured = {}) {
    if(status_ == RunStatus::Running) Leave(true);
    current_ = Find(routine_.initial); elapsed_ = 0; follower_.Restart(measured);
    if(!current_) {status_=RunStatus::Failed; return false;}
    status_=RunStatus::Running; Enter(); return true;
  }
  Velocity Tick(Pose pose, Velocity measured, double dt) {
    if(status_ != RunStatus::Running || !current_) return {};
    if(!std::isfinite(dt) || dt <= 0 || dt > .1) return Fail();
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
      std::array<const Goal*,kMaxLookahead> anchors{}; std::size_t count=0;
      const AutoNode* walk=current_; bool final=true;
      while(walk && count<anchors.size()) {
        anchors[count++]=&walk->goal;
        if(!walk->intermediate) {final=true;break;}
        final=false;
        walk=walk->next>=0 && static_cast<std::size_t>(walk->next)<routine_.nodeCount ? &routine_.nodes[walk->next] : nullptr;
        if(walk==current_) break;
      }
      const auto result=follower_.Calculate(pose,measured,anchors.data(),count,final,current_->constraints,dt);
      if(!result.valid) return Fail(); command=result.command;ready=result.reached;handedOff=result.handoff;
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
      if(!accepted) continue;
      const auto* next=Find(transition.to); if(!next) return Fail();
      const auto previous=current_->id; Leave(false);current_=next;elapsed_=0;
      if(!(handedOff && next->kind==NodeKind::Pose)) follower_.Restart(command);
      hooks_.Transitioned(transition.id,previous,next->id);Enter();
      return current_->kind==NodeKind::Pose ? command : Velocity{};
    }
    return command;
  }
  void Cancel() {if(status_==RunStatus::Running) Leave(true);status_=RunStatus::Cancelled;}
  RunStatus Status() const {return status_;}
  std::string_view ActiveState() const {return current_?current_->id:std::string_view{};}
 private:
  const AutoNode* Find(std::string_view id) const {for(std::size_t i=0;i<routine_.nodeCount;i++) if(routine_.nodes[i].id==id) return &routine_.nodes[i];return nullptr;}
  void Enter() {
    actionSucceeded_=false;
    duringSucceeded_.assign(current_->duringCount,false);
    hooks_.StateEntered(current_->id);
    if(current_->kind==NodeKind::Finish) {status_=RunStatus::Completed;return;}
    if(current_->kind==NodeKind::Action||current_->kind==NodeKind::Custom) hooks_.BeginAction(current_->action);
    for(std::size_t i=0;i<current_->duringCount;i++) hooks_.BeginAction(current_->during[i]);
  }
  void Leave(bool cancelled) {
    if(!current_) return;
    if(current_->kind==NodeKind::Action||current_->kind==NodeKind::Custom) hooks_.EndAction(current_->action,cancelled);
    for(std::size_t i=0;i<current_->duringCount;i++) hooks_.EndAction(current_->during[i],cancelled);
  }
  Velocity Fail() {Leave(true);status_=RunStatus::Failed;return {};}
  static constexpr std::size_t kMaxLookahead=64;
  RoutineView routine_;RobotHooks& hooks_;PathFollower follower_;const AutoNode* current_=nullptr;
  std::vector<bool> duringSucceeded_;
  bool actionSucceeded_=false;
  double elapsed_=0;RunStatus status_=RunStatus::Idle;
};
// Alliance transforms. Flip: rotate the field 180 degrees (opposite alliance).
// Mirror: reflect across the long centerline (left/right variant).
inline Pose FlipPose(Pose pose,double fieldLength,double fieldWidth) {return {fieldLength-pose.x,fieldWidth-pose.y,Wrap(pose.heading+3.14159265358979323846)};}
inline Pose MirrorPose(Pose pose,double fieldWidth) {return {pose.x,fieldWidth-pose.y,Wrap(-pose.heading)};}
} // namespace mayhem
