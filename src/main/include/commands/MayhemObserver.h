#pragma once
#include <memory>
#include "logging/Logger.h"
#include "mayhem/networktables_observer.hpp"
#include "mayhem/wpilib.hpp"

// Use the robot's existing DataLog (including USB/fallback policy) and NT4 server.
class MayhemObserver final : public mayhem::StateObserver {
 public:
  explicit MayhemObserver(std::string machine)
      : m_nt(nt::NetworkTableInstance::GetDefault(), machine) {
    if (auto* log = logging::Logger::GetLog())
      m_log = std::make_unique<mayhem::WpiLogObserver>(*log, machine);
  }
  void Describe(std::string_view json) {
    m_nt.Describe(json);
    if (m_log)
      m_log->Describe(json);
  }
  void RegisterGuard(std::string transition, std::string input) {
    m_nt.RegisterGuard(transition, input);
    if (m_log)
      m_log->RegisterGuard(transition, input);
  }
  void Enter(std::string_view m, std::string_view s, double t) override {
    m_nt.Enter(m, s, t);
    if (m_log)
      m_log->Enter(m, s, t);
  }
  void Guard(std::string_view m, std::string_view e, std::string_view i, bool b,
             double t) override {
    m_nt.Guard(m, e, i, b, t);
    if (m_log)
      m_log->Guard(m, e, i, b, t);
  }
  void Transition(std::string_view m, std::string_view e, std::string_view f, std::string_view s,
                  double t) override {
    m_nt.Transition(m, e, f, s, t);
    if (m_log)
      m_log->Transition(m, e, f, s, t);
  }
  void GoalProgress(std::string_view g, double x, double h, bool b, double t) override {
    m_nt.GoalProgress(g, x, h, b, t);
    if (m_log)
      m_log->GoalProgress(g, x, h, b, t);
  }

 private:
  mayhem::NtStateObserver m_nt;
  std::unique_ptr<mayhem::WpiLogObserver> m_log;
};
