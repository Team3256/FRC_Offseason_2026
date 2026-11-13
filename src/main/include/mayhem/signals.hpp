#pragma once
// Typed, fail-closed guard inputs shared by AutoRunner and StateMachine. C++17.
#include <cmath>
#include <cstddef>
#include <initializer_list>
#include <optional>
#include <string>
#include <string_view>
#include <variant>

namespace mayhem {
using SignalValue = std::variant<std::monostate, bool, double, std::string_view>;
enum class CompareOp { Eq, Gt, Lt };
// Missing (monostate), mismatched, or non-finite inputs return nullopt, and a
// guard treats nullopt as false, including under Not.
inline std::optional<bool> Compare(SignalValue actual, CompareOp op, SignalValue wanted) {
  if (actual.index() == 0 || actual.index() != wanted.index()) return std::nullopt;
  if (std::holds_alternative<double>(actual) && (!std::isfinite(std::get<double>(actual)) || !std::isfinite(std::get<double>(wanted)))) return std::nullopt;
  if (op == CompareOp::Eq) return actual == wanted;
  if (!std::holds_alternative<double>(actual)) return std::nullopt;
  return op == CompareOp::Gt ? std::get<double>(actual) > std::get<double>(wanted) : std::get<double>(actual) < std::get<double>(wanted);
}
// Typed comparisons for generated typed machines (guards read an Inputs struct directly). Same
// rules as Compare: Gt/Lt only on numbers, a non-finite number is missing (nullopt). A separate
// name, so existing Compare(SignalValue...) calls keep their overload.
inline std::optional<bool> CompareTyped(bool actual, CompareOp op, bool wanted) {
  if (op != CompareOp::Eq) return std::nullopt;
  return actual == wanted;
}
inline std::optional<bool> CompareTyped(double actual, CompareOp op, double wanted) {
  if (!std::isfinite(actual) || !std::isfinite(wanted)) return std::nullopt;
  return op == CompareOp::Eq ? actual == wanted : op == CompareOp::Gt ? actual > wanted : actual < wanted;
}
inline std::optional<bool> CompareTyped(std::string_view actual, CompareOp op, std::string_view wanted) {
  if (op != CompareOp::Eq) return std::nullopt;
  return actual == wanted;
}
inline std::optional<bool> Not(std::optional<bool> value) { return value ? std::optional<bool>{!*value} : std::nullopt; }
inline std::optional<bool> All(std::initializer_list<std::optional<bool>> values) {
  bool missing = false; for(auto v:values) {if(v && !*v) return false; if(!v) missing=true;} return missing ? std::nullopt : std::optional<bool>{true};
}
inline std::optional<bool> Any(std::initializer_list<std::optional<bool>> values) {
  bool missing = false; for(auto v:values) {if(v && *v) return true; if(!v) missing=true;} return missing ? std::nullopt : std::optional<bool>{false};
}
// Anything generated guards can read. String views must stay valid while guards run.
class SignalSource {
 public:
  virtual ~SignalSource() = default;
  virtual SignalValue Signal(std::string_view) const { return {}; }
};
enum class InputType { Boolean, Number, String };
// One declared manifest input, exported with each MachineView: what observers log (to `channel`,
// or Mayhem/States/<machine>/inputs/<key> when empty) and which inputs are one-shot events
// (true only for the first step of a loop; StateMachine::Settle hides them on later steps and
// typed machines reset them to `initial` after that step).
struct MachineInput {
  std::string_view key, channel;
  InputType type = InputType::Boolean;
  bool event = false;
  SignalValue initial{};
};
// A manifest JSON split into literal-sized chunks (MSVC rejects literals over 64 KB).
struct ManifestView {
  const std::string_view* parts = nullptr; std::size_t count = 0;
  bool Empty() const { return count == 0; }
  // Allocates; call during initialization, not in the control loop.
  std::string Text() const {
    std::size_t size = 0; for (std::size_t i = 0; i < count; ++i) size += parts[i].size();
    std::string text; text.reserve(size);
    for (std::size_t i = 0; i < count; ++i) text.append(parts[i].data(), parts[i].size());
    return text;
  }
};
}  // namespace mayhem
