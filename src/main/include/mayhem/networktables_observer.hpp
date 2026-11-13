#pragma once
// Optional WPILib 2026 NetworkTables publishing for live Mayhem inspection.
#include "mayhem/state_observer.hpp"
#include <networktables/NetworkTableInstance.h>
#include <networktables/StringTopic.h>
#include <networktables/BooleanTopic.h>
#include <networktables/DoubleTopic.h>
#include <string>
#include <variant>
#include <vector>
namespace mayhem {
class NtStateObserver final : public StateObserver {
 public:
  NtStateObserver(nt::NetworkTableInstance instance,std::string machine)
    : instance_(instance),machine_(std::move(machine)),
      state_(instance_.GetStringTopic("/Mayhem/States/"+machine_+"/state").Publish()),
      transition_(instance_.GetStringTopic("/Mayhem/States/"+machine_+"/transition").Publish()),
      schema_(instance_.GetStringTopic("/Mayhem/Schema/"+machine_).Publish()),
      goal_(instance_.GetStringTopic("/Mayhem/Goals/"+machine_+"/id").Publish()),
      translationError_(instance_.GetDoubleTopic("/Mayhem/Goals/"+machine_+"/translationError").Publish()),
      rotationError_(instance_.GetDoubleTopic("/Mayhem/Goals/"+machine_+"/rotationError").Publish()),
      reached_(instance_.GetBooleanTopic("/Mayhem/Goals/"+machine_+"/reached").Publish()) {}
  void Describe(std::string_view manifest){schema_.Set(manifest);}
  void RegisterGuard(std::string transition,std::string input){
    auto publisher=instance_.GetBooleanTopic("/Mayhem/States/"+machine_+"/guards/"+transition+"/"+input).Publish();
    guards_.push_back({std::move(transition),std::move(input),std::move(publisher)});
  }
  // Typed topic /Mayhem/States/<machine>/inputs/<key> per declared input (the live viewer reads
  // it when the manifest channel is not on NetworkTables). StateMachine::Attach with InputLogging::On.
  void RegisterInput(std::string_view machine,const MachineInput& input) override {
    if(machine!=machine_)return;
    const auto path="/Mayhem/States/"+machine_+"/inputs/"+std::string{input.key};
    InputEntry entry{std::string{input.key},input.type,{},{},{}};
    if(input.type==InputType::Boolean)entry.boolean=instance_.GetBooleanTopic(path).Publish();
    else if(input.type==InputType::Number)entry.number=instance_.GetDoubleTopic(path).Publish();
    else entry.text=instance_.GetStringTopic(path).Publish();
    inputs_.push_back(std::move(entry));
  }
  void Input(std::string_view machine,std::string_view key,const SignalValue& value,double) override {
    if(machine!=machine_)return;
    for(auto& input:inputs_){
      if(input.key!=key)continue;
      if(input.type==InputType::Boolean&&std::holds_alternative<bool>(value))input.boolean.Set(std::get<bool>(value));
      else if(input.type==InputType::Number&&std::holds_alternative<double>(value))input.number.Set(std::get<double>(value));
      else if(input.type==InputType::String&&std::holds_alternative<std::string_view>(value))input.text.Set(std::get<std::string_view>(value));
      return;
    }
  }
  void Enter(std::string_view machine,std::string_view state,double) override {if(machine==machine_)state_.Set(state);}
  void Guard(std::string_view machine,std::string_view transition,std::string_view input,bool result,double) override {
    if(machine!=machine_)return;for(auto& guard:guards_)if(guard.transition==transition&&guard.input==input){guard.publisher.Set(result);return;}
  }
  void Transition(std::string_view machine,std::string_view transition,std::string_view,std::string_view to,double) override {if(machine==machine_){transition_.Set(transition);state_.Set(to);}}
  void GoalProgress(std::string_view goal,double translationError,double rotationError,bool reached,double) override {
    goal_.Set(goal);translationError_.Set(translationError);rotationError_.Set(rotationError);reached_.Set(reached);
  }
 private:
  struct GuardEntry{std::string transition,input;nt::BooleanPublisher publisher;};
  struct InputEntry{std::string key;InputType type;nt::BooleanPublisher boolean;nt::DoublePublisher number;nt::StringPublisher text;};
  nt::NetworkTableInstance instance_;std::string machine_;std::vector<GuardEntry> guards_;std::vector<InputEntry> inputs_;
  nt::StringPublisher state_,transition_,schema_,goal_;nt::DoublePublisher translationError_,rotationError_;nt::BooleanPublisher reached_;
};
class CombinedObserver final : public StateObserver {
 public:
  CombinedObserver(StateObserver& first,StateObserver& second):first_(first),second_(second){}
  void Enter(std::string_view machine,std::string_view state,double timestamp) override {first_.Enter(machine,state,timestamp);second_.Enter(machine,state,timestamp);}
  void Guard(std::string_view machine,std::string_view transition,std::string_view input,bool result,double timestamp) override {first_.Guard(machine,transition,input,result,timestamp);second_.Guard(machine,transition,input,result,timestamp);}
  void Transition(std::string_view machine,std::string_view transition,std::string_view from,std::string_view to,double timestamp) override {first_.Transition(machine,transition,from,to,timestamp);second_.Transition(machine,transition,from,to,timestamp);}
  void GoalProgress(std::string_view goal,double translationError,double rotationError,bool reached,double timestamp) override {first_.GoalProgress(goal,translationError,rotationError,reached,timestamp);second_.GoalProgress(goal,translationError,rotationError,reached,timestamp);}
  void RegisterInput(std::string_view machine,const MachineInput& input) override {first_.RegisterInput(machine,input);second_.RegisterInput(machine,input);}
  void Input(std::string_view machine,std::string_view key,const SignalValue& value,double timestamp) override {first_.Input(machine,key,value,timestamp);second_.Input(machine,key,value,timestamp);}
 private:StateObserver& first_;StateObserver& second_;
};
} // namespace mayhem
