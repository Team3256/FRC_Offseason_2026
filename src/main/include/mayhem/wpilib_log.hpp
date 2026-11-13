#pragma once
// WPILOG state observer. Needs only wpi/DataLog.h (wpiutil): no command framework.
#include "mayhem/state_observer.hpp"
#include <wpi/DataLog.h>
#include <cstdint>
#include <string>
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
  wpi::log::DataLog& log_;std::string machine_;std::vector<GuardEntry> guards_;
  wpi::log::StringLogEntry state_,transition_,from_,to_,schema_,goal_;
  wpi::log::DoubleLogEntry translationError_,rotationError_;wpi::log::BooleanLogEntry reached_;
};
} // namespace mayhem
