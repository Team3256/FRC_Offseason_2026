# frc2shim (desktop testing only)

A small, API-compatible re-implementation of the subset of WPILib's C++ command-based framework
(`wpilibNewCommands`) that this robot program uses. It exists only because the desktop CMake build
(`tools/desktop`) is meant to work on machines that cannot download artifacts from
frcmaven.wpi.edu.

* The robot itself (`./gradlew build`, `./gradlew deploy`, `./gradlew simulateNative`) **never**
  uses this code; GradleRIO links the official `wpilibNewCommands` library from
  `vendordeps/WPILibNewCommands.json`.
* Scheduler semantics follow WPILib 2026 (trigger edge state initialised at bind time, commands
  that don't run when disabled are not scheduled / are cancelled while disabled, requirement
  interruption, composition execution order, etc.) so that the simulation tests are meaningful.
* Only add to it what the robot code needs, and keep signatures identical to WPILib's.
