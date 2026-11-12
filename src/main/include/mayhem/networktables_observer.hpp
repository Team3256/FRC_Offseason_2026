#pragma once
// Optional WPILib 2026 NetworkTables publishing for live Mayhem inspection.
#include "mayhem/state_observer.hpp"
#include <networktables/NetworkTableInstance.h>
#include <networktables/StringTopic.h>
#include <networktables/BooleanTopic.h>
#include <networktables/DoubleTopic.h>
#include <string>
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
  nt::NetworkTableInstance instance_;std::string machine_;std::vector<GuardEntry> guards_;
  nt::StringPublisher state_,transition_,schema_,goal_;nt::DoublePublisher translationError_,rotationError_;nt::BooleanPublisher reached_;
};
class CombinedObserver final : public StateObserver {
 public:
  CombinedObserver(StateObserver& first,StateObserver& second):first_(first),second_(second){}
  void Enter(std::string_view machine,std::string_view state,double timestamp) override {first_.Enter(machine,state,timestamp);second_.Enter(machine,state,timestamp);}
  void Guard(std::string_view machine,std::string_view transition,std::string_view input,bool result,double timestamp) override {first_.Guard(machine,transition,input,result,timestamp);second_.Guard(machine,transition,input,result,timestamp);}
  void Transition(std::string_view machine,std::string_view transition,std::string_view from,std::string_view to,double timestamp) override {first_.Transition(machine,transition,from,to,timestamp);second_.Transition(machine,transition,from,to,timestamp);}
  void GoalProgress(std::string_view goal,double translationError,double rotationError,bool reached,double timestamp) override {first_.GoalProgress(goal,translationError,rotationError,reached,timestamp);second_.GoalProgress(goal,translationError,rotationError,reached,timestamp);}
 private:StateObserver& first_;StateObserver& second_;
};
} // namespace mayhem
