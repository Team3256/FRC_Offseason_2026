# FRC 3256 — 2026 Offseason Robot (C++)

Robot code for the 2026 offseason robot, in C++ (WPILib 2026, command-based, Phoenix 6, PhotonVision).
It is a port of the previous Java code with the same robot behaviour: same CAN IDs, gains, lookup
tables, button bindings, superstructure state machine and autonomous routines. See
[What changed from the Java code](#what-changed-from-the-java-code) for the few deliberate differences.

## Build, deploy, simulate

Use WPILib 2026 VS Code, or from a terminal:

```bash
./gradlew build            # roboRIO + desktop build, runs every test
./gradlew deploy           # deploy to the robot (team 3256, from .wpilib/wpilib_preferences.json)
./gradlew simulateNative   # run the robot program in the WPILib simulator (sim GUI + DS)
```

The first build needs internet access (GradleRIO downloads WPILib, the roboRIO toolchain and the
vendor libraries). On an `event*` branch, `deploy` also commits your working changes (as before).

### Without GradleRIO (desktop only)

`CMakeLists.txt` + `tools/desktop/` build and test the same code on Linux x86-64 without GradleRIO,
using WPILib's libraries as packaged by RobotPy (PyPI) and Phoenix 6 from CTRE's maven. The robot is
never built this way; see `tools/desktop/frc2shim/README.md`.

```bash
./tools/desktop/fetch_deps.sh
cmake -S . -B build-desktop && cmake --build build-desktop -j4
ctest --test-dir build-desktop --output-on-failure      # or ./build-desktop/robot-tests
./build-desktop/robot-sim                               # headless simulation of the robot program
```

## Layout

```
src/main/include, src/main/cpp
  Robot, RobotContainer        robot loop, bindings, auto chooser
  subsystems/                  Shooter, ShooterPivot (hood), LinearSlide, IntakeRollers, Feeder,
                               Indexer (each: Constants, IO interface, TalonFX IO, sim IO),
                               swerve/ (Tuner X drivetrain, trench assist), vision/, sotm/ (shot
                               calculator), Superstructure (state machine)
  rollers/                     shared two-motor leader/follower IO for intake, feeder and indexer
  mayhem/                      C++ port of MayhemLib (trajectories, follower, bump recovery, autos)
  commands/MayhemAutos         the autonomous routines
  logging/                     logging (replaces AdvantageKit), loop timing
  utils/, sim/                 Phoenix helpers, DisableSubsystem, Mechanism2d, sim battery
src/main/deploy                AprilTag layout, Mayhem (.mtraj) and Choreo paths
src/test/cpp                   tests (GoogleTest), see below
tools/golden                   generates golden data from the original Java implementation
tools/desktop                  desktop CMake dependencies + frc2 shim
```

## Controls (driver, port 0)

| Input | Action |
| --- | --- |
| Left stick / right stick X | field-centric drive / rotate (squared response) |
| Left bumper (hold) | slow drive (30%) |
| Right bumper (hold) | super slow drive (15%) |
| A | SHOOT (and, while held, aim at the target) |
| X | INTAKE |
| Y | JITTER |
| B | HOME |
| Left trigger | OUTTAKE |
| D-pad up / down | shot speed multiplier +/- 0.02 |
| D-pad right | re-zero field-centric heading |

Driving into your alliance zone (blue x < 4 m, red x > 12.5 m) revs the shooter for the hub;
elsewhere shots feed the near corner of your half. The hood drops automatically near a trench.
Trench assist is available but off by default (`TrenchAssist/Enabled` on the dashboard).

## Dashboard extras

These only add information to the dashboard (SmartDashboard / Elastic); none of them change what
the robot does or how it is driven.

* **Alerts** (the `Alerts` widget): any TalonFX, CANcoder or the Pigeon 2 that stops talking on CAN
  for 0.5 s (error, named by mechanism and CAN ID), a motor over 70 C, a config that failed to apply
  at boot, the driver controller unplugged, a brownout since boot, a resting battery under 12.0 V
  while disabled, CAN bus utilization over 90%, robot loops over 20 ms, and no auto selected.
  Devices are checked at 5 Hz.
* **Auto preview** (`Field Visualize`): while disabled, the selected auto's paths (`AutoPath`) and
  start pose (`AutoStart`) for your alliance. `RealOutputs/Auto/StartPoseErrorMeters`,
  `.../StartPoseErrorDegrees` and `.../AtStartPose` (within 0.15 m and 5 degrees) help line the
  robot up when vision sees tags. Hidden once the robot is enabled.
* **Shot info** (`Field Visualize`): the current shot target (`ShotTarget`) and the shoot-on-the-move
  lookahead pose (`ShotLookahead`). `RealOutputs/Superstructure/ReadyToShoot` (with `/Flywheel`,
  `/Pivot` and `/Aimed`) lights up when the flywheel is spun up and the hood is in place.

## Autonomous

Pick a routine on the dashboard (**Mayhem Auto Chooser**) while disabled. The routine and its paths
are built when you pick it, so nothing loads at the start of auto.

| Chooser entry | Paths (deploy/mayhem) |
| --- | --- |
| Depot Trench Sweep Depot | DepotTrenchSweepDepotPt, DepotTrenchSweepDepotPt2 |
| Outpost Trench Sweep | OutpostTrenchSweepBumpPt1 |
| Outpost Trench Sweep x2 | OutpostTrenchSweepBumpPt1, OutpostTrenchSweepBumpPt2 |
| Depot Trench Sweep | DepotTrenchSweepBumpPt1 |
| Depot Trench Sweep x2 | DepotTrenchSweepBumpPt1, DepotTrenchSweepBump2xPt2 |

Paths are flipped for the red alliance. Bump recovery, time dilation and telemetry (`/Mayhem` in
NetworkTables) work as in MayhemLib.

## Logging

`logging/Logger.h` replaces AdvantageKit (Java only) and keeps its key layout, so existing
AdvantageScope layouts keep working:

* **Log file**: `.wpilog` on the USB stick (`/U/wpilogs`), or `/home/lvuser/wpilogs` without one;
  `logs/` in simulation. It holds outputs (`/RealOutputs/...`), IO inputs (`/Shooter/...`,
  `/LinearSlide/...`, ...), Driver Station and joystick data, dashboard values (`NT:/SmartDashboard/`),
  swerve and Mayhem telemetry, console output, and per-subsystem loop timing
  (`/RealOutputs/LoggedTracer/<name>MS`, `/RealOutputs/LoopTimeMS`). Phoenix's SignalLogger (hoot)
  runs as before.
* **NetworkTables**: the same values under `/AdvantageKit/...`, off while the FMS is attached
  (the old NT4PublisherNoFMS behaviour).

AdvantageKit's log replay mode has no equivalent; it was turned off (`kAdvkitUseReplayLogs = false`).

## Loop time

What was done to keep the 20 ms loop cheap:

* All status signals are refreshed in one batched call per loop (`phoenix_util::RefreshAll`), and
  signals are read without per-call refresh.
* The drivetrain state (a copy under a lock in Phoenix C++) is read once per loop and cached;
  the Java code fetched it around 20 times per loop (every trigger condition).
* Log entries are created once; logging a value is an append (skipped when unchanged) plus an NT set.
  No string building, map lookups or allocations per loop.
* Lookup tables are compile-time arrays; trajectory sampling and drive output don't allocate.
* Swerve telemetry runs on Phoenix's odometry thread, not the robot loop.
* CAN bus: every mechanism TalonFX (shooter, hood, slide, rollers) only sends the signals the code
  reads at 50 Hz; everything else drops to 4 Hz (`OptimizeBusUtilization`).

In the desktop simulation, which also runs every mechanism's physics, a full teleop loop takes about
0.8 ms on average (p99 about 1.5 ms). `RobotSimTest.LoopTimeIsWellUnderBudget` checks this.

## Tests

`./gradlew build` (or `ctest`) runs:

* **GoldenTest**: checks that the C++ produces the same numbers as the original Java code.
  `tools/golden/generate.sh` ran the original Java (MayhemLib's jar and the robot's TrenchAssist,
  ShotCalculator and FieldConstants) and recorded the results in `src/test/resources/golden`. The
  test feeds the C++ the same inputs. It covers 4000 trench-assist cases, 2000 shot-calculator cases,
  path sampling and flipping, the bridge planner, and 50 closed-loop runs of every path, including
  shoves that trigger bump recovery and auto-unbeach. Every step's drive command, state and fired
  events must match.
* **SuperstructureLogicTest**: every state transition and mechanism command of the state machine,
  with ideal mechanisms and stepped time.
* **RobotSimTest / AutoSimTest**: the whole robot program with simulated hardware, driven through
  the simulated Driver Station and controller in real time. Covers each button, zone behaviour,
  driving, aiming, vision, and every auto on both alliances.
* **FeatureTest**: the dashboard extras above (alert logic, every device watched and healthy in
  simulation, auto preview on both alliances, shot readiness).
* **UnitTest**: utilities, config read-back verification, vision filtering, trajectory files, Mayhem
  routine and trigger semantics, and the logging round-trip.

## What changed from the Java code

Robot behaviour is the same. The deliberate differences:

* **Choreo chooser removed.** The Java code built a Choreo "auto chooser" and "Auto Visualizer" on
  the dashboard, but autonomous only ever ran the Mayhem chooser. The Choreo paths stay in
  `deploy/choreo` for the Choreo app; ChoreoLib, AdvantageKit and MayhemLib-java are no longer
  dependencies.
* **AdvantageKit replaced** by the logger above (same keys, no replay).
* **ShotCalculator.GetDistance()** before the first update now returns the distance from the current
  pose. Java would have thrown a NullPointerException.
* **Simulation only** (the real robot is unaffected):
  - Mechanism physics runs once per loop. The Java sim stepped it twice, because `simulationPeriodic`
    called `periodic`.
  - One battery model sums the Phoenix-simulated supply current of every mechanism motor, so current
    limits count. Java let each mechanism overwrite the battery voltage with its own uncapped draw.
  - The hood sim uses the configured 30:1 sensor-to-mechanism ratio. With the old 4:1 the simulated
    hood could not pass about 0.016 rotations, so INTAKE never got past waiting for the hood.
  - Vision is simulated (`VisionIOSim`) instead of reporting a disconnected camera.
* The MayhemLib port leaves out what this robot doesn't use: the Sleipnir bridge refiner and the
  fuel and bump sim injectors.
* WPILib's C++ scheduler doesn't promise to run subsystem `Periodic()` in registration order. None of
  the subsystems depend on each other's periodic.

## Before the first deploy

1. Build once with internet access (`./gradlew build`) so GradleRIO caches everything, then check
   that the tests pass.
2. On the robot, check in the Driver Station log that every TalonFX config applied. A verification
   failure prints `Failed to verify config for talon [id]`.
   The dashboard's `Alerts` widget should show no errors (a red "is disconnected" or "Config
   failed" entry names the device and its CAN ID).
3. Plug in the USB stick and check that `/U/wpilogs` fills up. Open a log in AdvantageScope and
   confirm the usual keys are there.
4. In the pit, go through each button above with the robot on blocks. Then run each auto on the
   ground once per alliance color.
5. Check `RealOutputs/LoopTimeMS` and the `LoggedTracer` timings in the log.

### Mayhem Suite overhaul

The robot now executes `robot.mayhem.json` through the new Mayhem C++ graph runner
and pose follower. Regenerate with `tools/mayhem/export.sh` after edits.
See [the integration guide](docs/mayhem/README.md) for state ownership, log mapping,
telemetry, build instructions and migration limits.
