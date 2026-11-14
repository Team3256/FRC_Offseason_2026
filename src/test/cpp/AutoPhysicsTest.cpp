// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

// Every autonomous routine on the full REBUILT field in the physics simulation (sim::PhysicsWorld):
// the 20 s autonomous period on stepped time, checked for what would go wrong on a real field
// (hitting a trench arm or a wall, beaching on a bump, tipping, browning out, never scoring).
//
// Monte Carlo: with MAYHEM_SIM_SEED set, MonteCarlo.Run runs one auto (MAYHEM_SIM_AUTO, default
// the first chooser entry; MAYHEM_SIM_RED=1 for red) on a randomly perturbed robot and field and
// writes MAYHEM_SIM_OUT (JSON) and a .wpilog to open in AdvantageScope. tools/sim/montecarlo.sh
// runs hundreds of these in parallel and summarizes how they fail.

#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <numbers>
#include <sstream>
#include <string>
#include <vector>

#include <frc/DataLogManager.h>
#include <gtest/gtest.h>
#include <networktables/NetworkTableInstance.h>

#include "RobotHarness.h"
#include "commands/MayhemProject.h"
#include "logging/Logger.h"
#include "mayhem/sim/rng.hpp"
#include "sim/PhysicsWorld.h"

namespace {

struct AutoResult {
  std::string status;       // Mayhem/Auto/Status at the end ("Completed", "Failed", ...)
  std::string failedNode;   // the graph state when it failed
  double completedAt = -1;  // s into autonomous
  sim::Metrics metrics;
  double endX = 0, endY = 0;
};

/** Runs {@code name} for the whole 20 s autonomous period on the full field. */
AutoResult RunAuto(const std::string& name, bool red) {
  RobotHarness h{red, /*field=*/true};
  h.Step(5);
  h.C().GetAutoChooser().Select(name);
  h.Step(3);
  h.Autonomous();
  auto status =
      nt::NetworkTableInstance::GetDefault().GetTable("AdvantageKit/RealOutputs/Mayhem/Auto");
  auto graph = nt::NetworkTableInstance::GetDefault().GetTable("Mayhem/States/Autonomous");
  AutoResult r;
  std::string node;
  for (int i = 0; i < 1000; i++) {  // 20 s
    h.Step();
    r.status = status->GetString("Status", "");
    const auto now = graph->GetString("state", "");
    if (r.status != "Failed") {
      node = now;
    }
    if (r.status == "Completed" && r.completedAt < 0) {
      r.completedAt = (i + 1) * 0.02;
    }
  }
  if (r.status == "Failed") {
    r.failedNode = node;
  }
  const auto pose = sim::PhysicsWorld::Get().TruePose();
  r.endX = pose.X().value();
  r.endY = pose.Y().value();
  r.metrics = sim::PhysicsWorld::Get().GetMetrics();
  h.Disable();
  h.Step(2);
  return r;
}

/** The first problem a real field would have shown, or "" if the run was clean. */
/** The first problem a real field would have shown (in time order), or "" if the run was clean. */
std::string FirstFailure(const AutoResult& r) {
  const auto& m = r.metrics;
  if (m.tippedOver) {
    return "tipped over";
  }
  // Hits and beaching, earliest first: later trouble usually follows from the first.
  if (!m.hits.empty()) {
    const std::string& first = m.hits.front();
    if (first.rfind("beached", 0) == 0) {
      return "beached (high-centred)";
    }
    if (first.rfind("perimeter", 0) == 0) {
      return "hit the field wall";
    }
    if (first.rfind("trench_arm", 0) == 0) {
      return "hit a trench arm (robot taller than 22.25 in)";
    }
    return "hit " + first.substr(0, first.find(' '));
  }
  if (r.status == "Failed") {
    return "auto failed at " + r.failedNode;
  }
  if (r.completedAt < 0) {
    return "did not finish in 20 s (status " + r.status + ")";
  }
  if (m.brownedOut) {
    return "browned out";
  }
  if (m.shotsFired == 0) {
    return "never shot";
  }
  if (m.scoredOwnHub == 0) {
    return "no FUEL scored";
  }
  return "";
}

std::string Describe(const AutoResult& r) {
  const auto& m = r.metrics;
  std::ostringstream s;
  s << "status " << r.status << " (done at " << r.completedAt << " s), shots " << m.shotsFired
    << ", scored " << m.scoredOwnHub << " (+" << m.scoredOtherHub << " other hub), intaked "
    << m.intaked << ", wall hits " << m.wallHits << ", trench arm hits " << m.trenchArmHits
    << ", other hits " << m.obstacleHits << ", beached " << m.beachedSeconds << " s, max tilt "
    << m.maxTiltDeg << " deg, min battery " << m.minBatteryVolts << " V, end (" << r.endX << ", "
    << r.endY << ")";
  for (const auto& hit : m.hits) {
    s << "\n  hit " << hit;
  }
  return s.str();
}

class AutoPhysicsTest : public ::testing::TestWithParam<std::tuple<std::string, bool>> {};

TEST_P(AutoPhysicsTest, RunsCleanOnTheField) {
  const auto& [name, red] = GetParam();
  sim::PhysicsWorld::SetNextPerturbation({});
  const auto r = RunAuto(name, red);
  EXPECT_EQ(FirstFailure(r), "") << name << (red ? " red: " : " blue: ") << Describe(r);
}

std::vector<std::tuple<std::string, bool>> AllAutos() {
  std::vector<std::tuple<std::string, bool>> out;
  for (const auto& r : mayhem_generated::routines) {
    out.emplace_back(std::string{r.name}, false);
    out.emplace_back(std::string{r.name}, true);
  }
  return out;
}

INSTANTIATE_TEST_SUITE_P(EveryAuto, AutoPhysicsTest, ::testing::ValuesIn(AllAutos()),
                         [](const auto& info) {
                           std::string n = std::get<0>(info.param);
                           std::string out;
                           for (char c : n) {
                             if (std::isalnum(static_cast<unsigned char>(c))) {
                               out += c;
                             }
                           }
                           return out + (std::get<1>(info.param) ? "Red" : "Blue");
                         });

}  // namespace

TEST(MonteCarlo, Run) {
  const char* seedText = std::getenv("MAYHEM_SIM_SEED");
  if (seedText == nullptr) {
    GTEST_SKIP() << "set MAYHEM_SIM_SEED (tools/sim/montecarlo.sh does)";
  }
  const auto seed = mayhem::sim::Rng::SeedFromEnv(1);
  mayhem::sim::Rng rng{seed};
  const char* autoName = std::getenv("MAYHEM_SIM_AUTO");
  const std::string name = autoName ? autoName : std::string{mayhem_generated::routines[0].name};
  const bool red = std::getenv("MAYHEM_SIM_RED") != nullptr;

  // Robot and field variation a real match sees.
  sim::Perturbation p;
  p.massScale = 1 + rng.Normal(0, 0.03);
  p.muScale = 1 + rng.Normal(0, 0.08);
  p.batteryResistanceScale = 1 + std::abs(rng.Normal(0, 0.4));
  p.batteryVoltageOffset = rng.Uniform(-0.5, 0.2);
  p.startX = rng.Normal(0, 0.04);
  p.startY = rng.Normal(0, 0.04);
  p.startYaw = rng.Normal(0, 2.0 * std::numbers::pi / 180);
  p.visionNoise = 0.02 * (1 + std::abs(rng.Normal(0, 0.5)));
  p.shotSpeedScale = 1 + rng.Normal(0, 0.02);
  sim::PhysicsWorld::SetNextPerturbation(p);

  // A log per run, to open the bad ones in AdvantageScope.
  std::string logPath;
  if (const char* out = std::getenv("MAYHEM_SIM_OUT")) {
    const auto dir = std::filesystem::path{out}.parent_path() / ("log_" + std::to_string(seed));
    std::filesystem::create_directories(dir);
    logging::Logger::Start(dir.string());
  }
  const auto r = RunAuto(name, red);
  sim::PhysicsWorld::SetNextPerturbation({});
  if (auto* log = logging::Logger::GetLog()) {
    log->Flush();
    for (const auto& f : std::filesystem::directory_iterator{frc::DataLogManager::GetLogDir()}) {
      if (f.path().extension() == ".wpilog") {
        logPath = f.path().string();
      }
    }
  }
  const std::string failure = FirstFailure(r);
  const auto& m = r.metrics;
  if (const char* out = std::getenv("MAYHEM_SIM_OUT")) {
    std::ofstream json{out};
    auto escape = [](std::string s) {
      std::string e;
      for (char c : s) {
        e += (c == '"' || c == '\\') ? std::string{'\\', c} : std::string{c};
      }
      return e;
    };
    json << "{\"passed\": " << (failure.empty() ? "true" : "false") << ", \"failure\": \""
         << escape(failure) << "\", \"log\": \"" << escape(logPath) << "\", \"metrics\": {"
         << "\"autoSeconds\": " << r.completedAt << ", \"shots\": " << m.shotsFired
         << ", \"scored\": " << m.scoredOwnHub << ", \"intaked\": " << m.intaked
         << ", \"wallHits\": " << m.wallHits << ", \"trenchArmHits\": " << m.trenchArmHits
         << ", \"beachedSeconds\": " << m.beachedSeconds << ", \"maxTiltDeg\": " << m.maxTiltDeg
         << ", \"minBatteryVolts\": " << m.minBatteryVolts << ", \"startX\": " << p.startX
         << ", \"startY\": " << p.startY << ", \"mu\": " << p.muScale
         << ", \"mass\": " << p.massScale << "}}\n";
  }
  std::printf("[montecarlo] seed %llu %s %s: %s\n%s\n", static_cast<unsigned long long>(seed),
              name.c_str(), red ? "red" : "blue", failure.empty() ? "clean" : failure.c_str(),
              Describe(r).c_str());
  // One run per process: exit here. (Tearing down the data log and NetworkTables loggers at
  // static destruction can abort with "mutex lock failed", which would mark the run failed.)
  std::fflush(nullptr);
  std::_Exit(0);
}
