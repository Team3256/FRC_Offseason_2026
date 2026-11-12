#pragma once
#include <string_view>

/** Latched operator intent, separate from the physical mechanism state.
 * Autonomous requests replace the intent; operator shoot/intake/jitter requests combine.
 * The Mayhem graph resolves these inputs into the executable state.
 */
struct SuperstructureIntent {
  enum class Mode { Normal, Idle, Home, Outtake, Cancel };
  Mode mode = Mode::Normal;
  bool shoot = false, intake = false, jitter = false, manualRev = false;

  void Reset(Mode next = Mode::Normal) {
    mode = next;
    shoot = intake = jitter = manualRev = false;
  }
  bool Request(std::string_view state, bool exact = false) {
    if (state == "IDLE") {
      Reset(Mode::Idle);
      return true;
    }
    if (state == "HOME") {
      Reset(Mode::Home);
      return true;
    }
    if (state == "OUTTAKE") {
      Reset(Mode::Outtake);
      return true;
    }
    if (state == "CANCEL_ALL") {
      Reset(Mode::Cancel);
      return true;
    }
    if (state != "SHOOT" && state != "INTAKE" && state != "JITTER" && state != "REV" &&
        state != "SHOOT_AND_INTAKE" && state != "REV_AND_INTAKE" && state != "JITTER_AND_SHOOT")
      return false;
    if (exact || mode != Mode::Normal)
      Reset();
    mode = Mode::Normal;
    if (state == "SHOOT") {
      shoot = true;
      manualRev = false;
    }
    if (state == "INTAKE") {
      intake = true;
      jitter = false;
    }
    if (state == "JITTER") {
      jitter = true;
      intake = false;
    }
    if (state == "REV") {
      manualRev = true;
      shoot = false;
    }
    if (state == "SHOOT_AND_INTAKE") {
      shoot = intake = true;
      jitter = manualRev = false;
    }
    if (state == "REV_AND_INTAKE") {
      manualRev = intake = true;
      shoot = jitter = false;
    }
    if (state == "JITTER_AND_SHOOT") {
      shoot = jitter = true;
      intake = manualRev = false;
    }
    return true;
  }
};
