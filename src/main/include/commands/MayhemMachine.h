#pragma once
#include <functional>
#include "commands/MayhemObserver.h"
#include "mayhem/auto_runner.hpp"

/** Sensor-driven mechanism graph. State dwell time uses the FPGA clock.
 * Timeouts are explicit graph transitions to FAULT, not simulated success.
 */
class MayhemMachine final : private mayhem::RobotHooks {
 public:
  using Signals = std::function<mayhem::SignalValue(std::string_view)>;
  MayhemMachine(std::string_view id, mayhem::RoutineView graph, std::string_view schema,
                Signals signals)
      : m_id(id),
        m_signals(std::move(signals)),
        m_observer(std::string{id}),
        m_stateTimeLog(std::string{"Mayhem/States/"} + std::string{id} + "/inputs/stateTime"),
        m_runner(graph, static_cast<mayhem::RobotHooks&>(*this)) {
    m_observer.Describe(schema);
    for (std::size_t i = 0; i < graph.transitionCount; ++i)
      m_observer.RegisterGuard(std::string{graph.transitions[i].id}, "condition");
    Reset();
  }
  void Tick() {
    m_stateTimeLog.Set(StateTime());
    m_runner.Tick({}, {}, .02);
  }
  void Reset() { m_runner.Start(); }
  std::string_view State() const { return m_runner.ActiveState(); }
  double StateTime() const { return frc::Timer::GetFPGATimestamp().value() - m_entered; }

 private:
  mayhem::SignalValue Signal(std::string_view key) const override {
    if (key == "stateTime")
      return StateTime();
    return m_signals(key);
  }
  mayhem::ActionStatus TickAction(std::string_view) override {
    return mayhem::ActionStatus::Succeeded;
  }
  void StateEntered(std::string_view state) override {
    m_entered = frc::Timer::GetFPGATimestamp().value();
    m_observer.Enter(m_id, state, m_entered);
  }
  void GuardEvaluated(std::string_view edge, bool accepted) override {
    m_observer.Guard(m_id, edge, "condition", accepted, frc::Timer::GetFPGATimestamp().value());
  }
  void Transitioned(std::string_view edge, std::string_view from, std::string_view to) override {
    m_observer.Transition(m_id, edge, from, to, frc::Timer::GetFPGATimestamp().value());
  }
  std::string_view m_id;
  Signals m_signals;
  MayhemObserver m_observer;
  logging::Output<double> m_stateTimeLog;
  mayhem::AutoRunner m_runner;
  double m_entered = 0;
};
