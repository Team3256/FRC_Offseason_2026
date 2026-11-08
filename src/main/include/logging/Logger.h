// Copyright (c) 2025 FRC 3256
// https://github.com/Team3256
//
// Use of this source code is governed by a
// license that can be found in the LICENSE file at
// the root directory of this project.

#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <span>
#include <string>
#include <string_view>
#include <vector>

#include <networktables/BooleanTopic.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/DoubleTopic.h>
#include <networktables/IntegerArrayTopic.h>
#include <networktables/IntegerTopic.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/StringTopic.h>
#include <networktables/StructArrayTopic.h>
#include <networktables/StructTopic.h>
#include <wpi/DataLog.h>
#include <wpi/struct/Struct.h>

/**
 * Logging that replaces AdvantageKit (which has no C++ version).
 *
 * Every value goes to two places:
 *  - the on-robot .wpilog file (USB stick at /U/wpilogs, else /home/lvuser/wpilogs), written by
 *    WPILib's DataLog background writer. Keys match what AdvantageKit wrote ("/RealOutputs/..."
 *    for outputs, "/<Subsystem>/<field>" for IO inputs), so existing AdvantageScope layouts work.
 *  - NetworkTables under "/AdvantageKit/...", like AdvantageKit's NT4Publisher. Publishing is
 *    skipped on a real robot while the FMS is attached (the old NT4PublisherNoFMS behaviour).
 *
 * Loop-time notes: entries are created once (members of the classes that log), so a loop does no
 * string building, map lookups or allocations. DataLog "Update" only appends when a value changed.
 */
namespace logging {

class Logger {
 public:
  /**
   * Starts file logging into {@code dir} (empty = no file, NetworkTables only). Call once, before
   * anything creates log entries; entries made earlier only publish to NetworkTables.
   */
  static void Start(std::string_view dir);

  /** Call at the start of every robot loop. */
  static void Periodic();

  /** The data log, or nullptr when file logging is off. */
  static wpi::log::DataLog* GetLog();

  /** True when values should also be published to NetworkTables this loop. */
  static bool NtActive() { return s_ntActive; }

  /** Forces NetworkTables publishing on/off (tests). */
  static void SetNtActive(bool active) { s_ntActive = active; }

  /** Writes a message to the log's "messages" entry and stdout. */
  static void Message(std::string_view message);

  static std::string LogKey(std::string_view key);
  static std::string NtKey(std::string_view key);
  static nt::NetworkTableInstance Nt() { return nt::NetworkTableInstance::GetDefault(); }

 private:
  static inline bool s_ntActive = true;
};

namespace detail {
template <typename T>
struct Traits;

template <>
struct Traits<double> {
  using Entry = wpi::log::DoubleLogEntry;
  using Pub = nt::DoublePublisher;
  static Pub Publish(std::string_view k) { return Logger::Nt().GetDoubleTopic(k).Publish(); }
};
template <>
struct Traits<bool> {
  using Entry = wpi::log::BooleanLogEntry;
  using Pub = nt::BooleanPublisher;
  static Pub Publish(std::string_view k) { return Logger::Nt().GetBooleanTopic(k).Publish(); }
};
template <>
struct Traits<int64_t> {
  using Entry = wpi::log::IntegerLogEntry;
  using Pub = nt::IntegerPublisher;
  static Pub Publish(std::string_view k) { return Logger::Nt().GetIntegerTopic(k).Publish(); }
};
template <>
struct Traits<std::string> {
  using Entry = wpi::log::StringLogEntry;
  using Pub = nt::StringPublisher;
  static Pub Publish(std::string_view k) { return Logger::Nt().GetStringTopic(k).Publish(); }
};
template <>
struct Traits<std::vector<double>> {
  using Entry = wpi::log::DoubleArrayLogEntry;
  using Pub = nt::DoubleArrayPublisher;
  static Pub Publish(std::string_view k) { return Logger::Nt().GetDoubleArrayTopic(k).Publish(); }
};
template <>
struct Traits<std::vector<int64_t>> {
  using Entry = wpi::log::IntegerArrayLogEntry;
  using Pub = nt::IntegerArrayPublisher;
  static Pub Publish(std::string_view k) { return Logger::Nt().GetIntegerArrayTopic(k).Publish(); }
};
template <wpi::StructSerializable T>
struct Traits<T> {
  using Entry = wpi::log::StructLogEntry<T>;
  using Pub = nt::StructPublisher<T>;
  static Pub Publish(std::string_view k) { return Logger::Nt().GetStructTopic<T>(k).Publish(); }
};
template <wpi::StructSerializable T>
struct Traits<std::vector<T>> {
  using Entry = wpi::log::StructArrayLogEntry<T>;
  using Pub = nt::StructArrayPublisher<T>;
  static Pub Publish(std::string_view k) {
    return Logger::Nt().GetStructArrayTopic<T>(k).Publish();
  }
};
}  // namespace detail

/**
 * One logged value. {@code T} is double, bool, int64_t, std::string, std::vector<double>,
 * std::vector<int64_t>, a WPILib struct type (Pose2d, ChassisSpeeds, ...) or a std::vector of one.
 * Vector types are set from a span, strings from a string_view.
 */
template <typename T>
class LogValue {
  using Tr = detail::Traits<T>;

 public:
  /** @param key full key, e.g. "RealOutputs/Shooter/reqVelocity" or "Shooter/shooterMotorVoltages"
   */
  explicit LogValue(std::string_view key) : m_pub{Tr::Publish(Logger::NtKey(key))} {
    if (auto* log = Logger::GetLog()) {
      m_entry.emplace(*log, Logger::LogKey(key));
    }
  }

  template <typename U>
  void Set(const U& value) {
    if (m_entry) {
      m_entry->Update(value);
    }
    if (Logger::NtActive()) {
      m_pub.Set(value);
    }
  }

 private:
  std::optional<typename Tr::Entry> m_entry;
  typename Tr::Pub m_pub;
};

/** An output value ("RealOutputs/<key>"), the equivalent of Logger.recordOutput. */
template <typename T>
class Output : public LogValue<T> {
 public:
  explicit Output(std::string_view key) : LogValue<T>(std::string{"RealOutputs/"}.append(key)) {}
};

/**
 * Logs a subsystem's IO inputs (the equivalent of Logger.processInputs). Register each field once
 * with a pointer to it; Publish() then logs the current values under "<prefix>/<name>".
 */
class InputsLogger {
 public:
  explicit InputsLogger(std::string_view prefix) : m_prefix{prefix} {}

  void Add(std::string_view name, const double* value);
  void Add(std::string_view name, const bool* value);
  void Add(std::string_view name, std::span<const double> values);

  void Publish();

 private:
  std::string Key(std::string_view name) const { return m_prefix + "/" + std::string{name}; }

  std::string m_prefix;
  std::vector<std::pair<const double*, std::unique_ptr<LogValue<double>>>> m_doubles;
  std::vector<std::pair<const bool*, std::unique_ptr<LogValue<bool>>>> m_bools;
  std::vector<std::pair<std::span<const double>, std::unique_ptr<LogValue<std::vector<double>>>>>
      m_arrays;
};

}  // namespace logging
