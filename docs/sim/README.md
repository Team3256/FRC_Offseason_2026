# Physics simulation

The robot program runs in simulation on **Mayhem's physics engine** (`mayhem::sim`, vendored in
`src/main/include/mayhem/sim/`, source in [Mayhem](https://github.com/ryanabraham1/Mayhem) under
`lib-cpp/include/mayhem/sim/`). Everything is the real robot code: `RobotContainer`,
`Superstructure`, the Mayhem auto runner, Phoenix control requests. Only the motors, sensors,
drivetrain and field are simulated, and nothing depends on the wall clock. A 20 s autonomous runs in
about 1 s, the whole test suite in about 30 s, and a run with a given seed repeats bit for bit.

## What is simulated

| | How | Where |
|---|---|---|
| Every mechanism TalonFX | Each loop the sim reads what the robot code last sent the real `TalonFX` (`GetAppliedControl()`) and runs it on Mayhem's TalonFX emulator, configured from the **same** `TalonFXConfiguration` (slot gains, Motion Magic, current limits, ratios, FOC). Followers follow their leader. | `sim/PhysicsWorld.cpp`, `*IOSim.cpp` |
| Mechanisms | Flywheel (shooter), arm (hood, gravity), elevator (intake slide on its 12° rack), rollers (intake, feeder, indexer), with hard stops | `sim/PhysicsWorld.cpp` |
| Drivetrain | 6-DOF chassis on four spring-loaded wheel contacts with tire slip (friction circle, Stribeck), load transfer, bellypan and bumper contacts. Phoenix's swerve request math (deadbands, discretize, desaturate, optimize, cosine compensation, coupling) runs every 4 ms like Phoenix's odometry thread; odometry is a WPILib pose estimator fed by the simulated modules, gyro and camera. | `sim/DrivetrainSim.cpp`, `Drivetrain.cpp` |
| Battery | Internal + wiring resistance, bus filtered by wiring and capacitance, roboRIO brownout at 4.75 V (outputs off), disabled robot = all motors neutral | `sim/PhysicsWorld.cpp` |
| 2026 REBUILT field | Walls, hubs (body + funnel), the four bumps (real profile: 6.5 in ridge, 15° ramps), trench arms (22.25 in clearance, checked against the robot's height: hopper on the slide, hood angle), trench posts, towers, depots | `sim/Rebuilt2026.cpp` |
| FUEL | All 408 staged balls (positions from AdvantageScope's field model), rolling, ball–ball and ball–robot contacts, intake (only when the slide is out and the rollers spin), hopper (8 preloaded, 40 max), feeding, shots with drag and backspin, hub scoring, scored FUEL coming back out | `sim/PhysicsWorld.cpp` |
| Camera | `VisionIOSim` sees the robot's **true** pose plus noise; the estimate fuses it like on the robot | `VisionIOSim.cpp` |

The shot model is fitted to the robot's own hub lookup tables at startup (they are tuned on the real
robot, so they score): this robot lobs into the hub (exit ≈ 0.38 × flywheel surface speed, launch
≈ 81°→76° as the hood opens), and the fit reproduces the table to 0.1 m.

## Running it

```bash
./gradlew test                       # or run the test binary directly:
B=build/install/frcUserProgramTest/osxuniversal/release
$B/frcUserProgramTest --gtest_filter='EveryAuto/*'      # every auto, both alliances, full field
ROBOT_SIM_REALTIME=1 $B/frcUserProgramTest --gtest_filter=...   # wall-clock pace, to watch live
./gradlew simulateNative             # the robot program in real time, with the sim GUI
```

macOS can kill (`exit 137`/SIGKILL) a test binary that was relinked in place. Re-sign it with
`codesign --force -s - $B/lib/frcUserProgramTest` (`tools/sim/montecarlo.sh` does this).

**`AutoPhysicsTest`** runs every auto for the whole 20 s on the full field and fails on the first
problem a real field would show, in time order: tipping, hitting a wall / trench arm / post /
hub, beaching, the auto failing or not finishing, browning out, never shooting or scoring. The
message lists every hit with time, pose and speed.

**Monte Carlo** (`tools/sim/montecarlo.sh`): hundreds of runs of one auto, each on a randomly
perturbed robot (mass ±3 %, tread friction ±8 %, battery resistance and charge, start pose ±4 cm /
±2°, camera noise, shot speed ±2 %), summarized by how they fail:

```bash
tools/sim/montecarlo.sh "Depot Trench Sweep" --runs 200 --jobs 8
tools/sim/montecarlo.sh "Outpost Trench Sweep x2" --red --runs 100
MAYHEM_SIM_AUTO="Depot Trench Sweep" MAYHEM_SIM_SEED=41 $B/frcUserProgramTest --gtest_filter=MonteCarlo.Run   # one seed again
```

Each run writes a `.wpilog` (`sim-runs/<auto>/log_<seed>/`); the summary lists the worst ones.

## AdvantageScope

1. Build the robot model from the CAD (once, or after the CAD changes):
   ```bash
   uv run tools/sim/cad-to-advantagescope.py ~/Downloads/00.Main.glb tools/sim/robot-cad.json \
     --out ~/Library/Application\ Support/AdvantageScope/userAssets
   python3 tools/sim/trim_hopper.py ~/Library/Application\ Support/AdvantageScope/userAssets/Robot_32562026/model_0.glb
   ```
   It writes `Robot_32562026` (≈10 MB, robot frame, three articulated components) and a preview PNG
   next to it; `trim_hopper.py` cuts the CAD's old 0.61 m hopper down to the real robot's 0.555 m
   (just under the trench). `tools/sim/robot-cad.json` says which parts move: the intake slide (Onshape planar
   mate, 0.309 m along the 12° rack), the hood (pivot frame about the main shooter roller axle), the
   kicker bar (Onshape revolute, folded 72° until the first deploy).
2. Open a sim log (or connect to `simulateNative`) and add a **3D Field** tab, field "2026 Field":
   - **Robot**: `RealOutputs/Sim/RobotPose` (true pose, with pitch/roll on the bumps), model
     "3256 2026", components `RealOutputs/Sim/Components`.
   - **Ghost**: the pose estimate, `NT:/DriveState/Pose` (shows odometry and vision error).
   - **Game pieces**: `RealOutputs/Sim/Fuel` and `RealOutputs/Sim/FlyingFuel` as "Fuel".
3. Line graphs worth having: `Sim/BatteryVolts`, `Sim/TotalCurrentAmps`, `Sim/FlywheelRps`,
   `Sim/WheelNormalForces`, `Sim/WheelSlip`, `Sim/TiltDegrees`, `Sim/RobotHeightMeters`,
   `Sim/HopperCount`, `Sim/ShotsFired`, `Sim/ScoredOwnHub`, `Sim/Contacts`.

## Tuning off the sim

Gains, Motion Magic, current limits and ratios live in the robot's own constants files, and the sim
uses exactly those, so tuning them in sim tunes the robot. The sim's physical parameters are in
`src/main/include/sim/SimConstants.h`. Mass, CoM, inertias and the mechanism geometry come from
Onshape; the ones marked **measure** are estimates and are what to measure first:

- tread friction (`kMuStatic`, `kMuKinetic`): drive into a wall, or watch acceleration in a log;
- flywheel inertia (`kFlywheelInertia`), battery resistance, feed rate, intake capture geometry.

Compare a real log with a sim run of the same auto (same keys) to see what to change.

## What the sim has found so far

- `Depot Trench Sweep`, `Depot Trench Sweep Depot`, `Outpost Trench Sweep`: clean on both
  alliances; Monte Carlo 80/80 for Depot Trench Sweep (median 24 FUEL scored of 33 shot).
- All four `x2` autos: on the return trip along the side wall the robot rotates 145–170° with its
  centre 0.49–0.57 m from the wall, and its corners sweep out to 0.575 m, so it scrapes the wall at
  ~3 m/s (`perimeter/ymin|ymax`, t ≈ 10–11 s). `Depot Trench Sweep x2` then clips the trench post
  corner (t ≈ 14–15 s) and can end up pinned against it. The old `minY > 0.32` check let the
  bumper overlap the wall by 9 cm.
- Battery: spinning the 4-Kraken shooter up while the intake rollers start and the robot drives
  pulls ~400 A; the 5th-percentile minimum is ~4.9 V, just above the 4.75 V brownout threshold.

## Limits

- The trench/hood/hopper height and the shot model are calibrated from the code's tables and the
  CAD, not measured. Ball–ball and ball–robot contacts are simplified (spheres, a robot box).
- Phoenix device health (connected, configs) still comes from Phoenix's own simulation, which runs
  on the wall clock (`FeatureSimTest` uses `RobotHarness::StepWallClock` for it).
- Reusable parts (engine, CAD exporter, Monte Carlo runner) belong in Mayhem; change them there and
  vendor (`node scripts/mayhem.mjs vendor <robot> --snapshot docs/mayhem/vendor-snapshot.json`).
