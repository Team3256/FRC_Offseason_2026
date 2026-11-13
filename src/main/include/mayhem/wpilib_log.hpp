#pragma once
// WPILOG state observer. Needs only wpi/DataLog.h (wpiutil): no command framework.
#include "mayhem/state_observer.hpp"
#include <wpi/DataLog.h>
#include <cstdint>
#include <string>
#include <variant>
#include <vector>

namespace mayhem {
// One observer per state machine. Register guards during robot initialization;
// steady-state guard and goal observations do not allocate or touch the network.
class WpiLogObserver final : public StateObserver {
 public:
  WpiLogObserver(wpi::log::DataLog& log,std::string machine)
    : log_(log),machine_(std::move(machine)),
      state_(log,"Mayhem/States/"+machine_+"/state"),
      transition_(log,"Mayhem/States/"+machine_+"/transition"),
      from_(log,"Mayhem/States/"+machine_+"/from"),to_(log,"Mayhem/States/"+machine_+"/to"),
      schema_(log,"Mayhem/Schema/"+machine_),goal_(log,"Mayhem/Goals/"+machine_+"/id"),
      translationError_(log,"Mayhem/Goals/"+machine_+"/translationError"),
      rotationError_(log,"Mayhem/Goals/"+machine_+"/rotationError"),
      reached_(log,"Mayhem/Goals/"+machine_+"/reached") {}
  void Describe(std::string_view json,double timestamp=0){schema_.Append(json,Micros(timestamp));}
  void RegisterGuard(std::string transition,std::string input){
    const auto path="Mayhem/States/"+machine_+"/guards/"+transition+"/"+input;
    guards_.push_back({std::move(transition),std::move(input),wpi::log::BooleanLogEntry{log_,path}});
  }
  // One typed entry per declared input, at the manifest `channel` (or
  // Mayhem/States/<machine>/inputs/<key>). Called by StateMachine::Attach with InputLogging::On.
  void RegisterInput(std::string_view machine,const MachineInput& input) override {
    if(machine!=machine_)return;
    const std::string path=input.channel.empty()?"Mayhem/States/"+machine_+"/inputs/"+std::string{input.key}:std::string{input.channel};
    InputEntry entry{std::string{input.key},input.type,{},{},{}};
    if(input.type==InputType::Boolean)entry.boolean=wpi::log::BooleanLogEntry{log_,path};
    else if(input.type==InputType::Number)entry.number=wpi::log::DoubleLogEntry{log_,path};
    else entry.text=wpi::log::StringLogEntry{log_,path};
    inputs_.push_back(std::move(entry));
  }
  void Input(std::string_view machine,std::string_view key,const SignalValue& value,double timestamp) override {
    if(machine!=machine_)return;
    for(auto& input:inputs_){
      if(input.key!=key)continue;
      const auto t=Micros(timestamp);
      if(input.type==InputType::Boolean&&std::holds_alternative<bool>(value))input.boolean.Append(std::get<bool>(value),t);
      else if(input.type==InputType::Number&&std::holds_alternative<double>(value))input.number.Append(std::get<double>(value),t);
      else if(input.type==InputType::String&&std::holds_alternative<std::string_view>(value))input.text.Append(std::get<std::string_view>(value),t);
      return;
    }
  }
  void Enter(std::string_view machine,std::string_view state,double timestamp) override {if(machine==machine_)state_.Append(state,Micros(timestamp));}
  void Guard(std::string_view machine,std::string_view transition,std::string_view input,bool result,double timestamp) override {
    if(machine!=machine_)return;for(auto& guard:guards_)if(guard.transition==transition&&guard.input==input){guard.entry.Append(result,Micros(timestamp));return;}
  }
  void Transition(std::string_view machine,std::string_view transition,std::string_view from,std::string_view to,double timestamp) override {
    if(machine!=machine_)return;const auto t=Micros(timestamp);transition_.Append(transition,t);from_.Append(from,t);to_.Append(to,t);state_.Append(to,t);
  }
  void GoalProgress(std::string_view goal,double translationError,double rotationError,bool reached,double timestamp) override {
    const auto t=Micros(timestamp);goal_.Append(goal,t);translationError_.Append(translationError,t);rotationError_.Append(rotationError,t);reached_.Append(reached,t);
  }
 private:
  static int64_t Micros(double seconds){return static_cast<int64_t>(seconds*1e6);}
  struct GuardEntry{std::string transition,input;wpi::log::BooleanLogEntry entry;};
  struct InputEntry{std::string key;InputType type;wpi::log::BooleanLogEntry boolean;wpi::log::DoubleLogEntry number;wpi::log::StringLogEntry text;};
  wpi::log::DataLog& log_;std::string machine_;std::vector<GuardEntry> guards_;std::vector<InputEntry> inputs_;
  wpi::log::StringLogEntry state_,transition_,from_,to_,schema_,goal_;
  wpi::log::DoubleLogEntry translationError_,rotationError_;wpi::log::BooleanLogEntry reached_;
};
} // namespace mayhem
