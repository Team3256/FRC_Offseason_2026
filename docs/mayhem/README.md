# Mayhem Suite integration and state redesign

Open **robot.mayhem.json** in the current Mayhem desktop app. It owns the five
autonomous graphs, their pose goals/limits/events, and three mechanism graphs.
The original `project.mayhem` and `.mtraj` files remain as migration references
and legacy library regression fixtures. The chooser no longer executes them.

## Build and edit

```sh
python3 tools/mayhem/export.py
python3 tools/mayhem/export.py --check
./gradlew build
```

Commit the JSON and generated `src/main/include/commands/MayhemProject.h` together.
CI rejects stale generated output. Generation uses Python 3.9+ without packages,
Node, a solver, a sibling checkout, or a network connection. Graph edits take
effect on the robot after rebuilding/deploying; they do not command a live robot.
The generated code embeds the exact machine manifests used by that build.

Use the app's Constraints/Map view to tune routes. Robot defaults are overridden
by routine constraints, then generated limits, then manual node constraints.
Generated max speed is capped by the inherited path speed. A positive explicit
`goal.handoffRadius` overrides the effective constraint radius; zero inherits it.
The Python exporter implements the same precedence as the Mayhem editor.

## Driver intent and real mechanism progress

Driver controls stay A=shoot and aim assist while held, X=intake, Y=jitter,
B=home/ground intake, left trigger=outtake. Bumpers, D-pad, trench assist,
vision and shot lookahead are retained. A/X/Y requests combine intentionally:
shoot+intake and shoot+jitter remain supported. HOME, OUTTAKE, IDLE and CANCEL_ALL
clear old demands. Autonomous marker requests replace demands rather than
inheriting a stale shot request from a previous stage.

The three Mayhem machines have different responsibilities:

| Machine | Responsibility |
| --- | --- |
| `Superstructure` | Resolve operating intent with priority: invalid localization/cancel, home, outtake, shoot+jitter, shoot+intake, shoot, jitter, rev+intake, intake, rev, idle. |
| `IntakeSequence` | PARKED → CLEARING_HOOD → DEPLOYING → READY; READY ↔ JITTERING through safe redeployment. Clearance/deployment deadlines go to a latched FAULT. |
| `ShotCycle` | OFF → SPINNING_UP → AIMING → FEEDING. Lost speed/hood/slide readiness returns to preparation; lost aim returns to AIMING; trenches inhibit the shot. Preparation/aim deadlines go to FAULT. |

Buttons set intent; they are not mechanism stages. Select Intake or Shooting in
the app's State machines workspace to see the physical intermediate relationships.
Every transition uses actual sensor/intent values; waiting for a deadline never
pretends a blocked mechanism completed. No possession or shot counter is invented
because this robot does not expose that sensor.

A shot feeds only while enabled, shooting is requested, localization/shot distance
are finite, the flywheel has reached a positive request, the hood has reached its
request, aim is within 5 degrees, slide clearance is safe for the operation, and
the robot is outside the trench approach zone. Readiness must hold for 60 ms to
start feeding; any loss stops the indexer/feeder immediately. Outtake cancels the
flywheel and indexer instead of retaining a previous shot's commands.

The slide cannot deploy until the measured hood has cleared it. The intake graph
has a 2.5-second hood-clearance deadline and a 3-second measured-deployment deadline.
The shooting graph allows 4 seconds for preparation and 3 seconds to acquire aim;
a trench inhibition waits safely for clearance. Faults stop mechanism demand and
abort the selected auto with zero drive output. A new valid request clears a
latched phase fault; disable clears every demand, so re-enable cannot resume an
old shot or outtake. All continuous slide/shooter/hood commands stop IO on cancel.

Zone boundaries use a 30 cm hysteresis band. Entering a hub zone can pre-spin an
idle/intaking robot but cannot replace an existing shooting intent. Shot targets
are selected immediately at startup and continuously, including while disabled.
The shot multiplier is bounded to 0.7–1.3.

## Reworked autonomous routes

All five chooser labels/order and `Nothing` remain. Each routine uses deliberately
sparse poses, not a dense conversion of the old time-parameterized trajectory.
Positions and approach headings were checked against the old solved passes;
exact start/end poses replace mismatched unsolved waypoints. BLine's joint
speed/radius generation removes corner orbiting. The second sweeps explicitly
square up before their trench crossings, hold the crossing heading, and turn only
after the full robot clears the opening. Controlled final approaches settle
before the separate live aim/shoot action.

Intake and idle are progress events on pose chains, so markers no longer break a
continuous path into stop/start action nodes. Missing legacy `intake2`/`idle2`
bindings have been replaced with explicit collection/end markers. Historical
warnings are archived in `legacy-marker-warnings.json`.

`drivetrain.alignAndShoot` pre-spins and tracks the live lookahead. It requests
SHOOT only within 3 degrees and keeps the original 3.5-second shooting window.
It fails if aim and physical readiness were never acquired; the timed window is
not proof that any particular number of pieces was shot. Browser **Path only**
preview skips this robot action and is shorter than the actual routine.

The command owns the drivetrain for the entire auto, including completion/failure,
so a driver default command cannot take over during auto. Each enable resolves
blue-origin goals, aim points and rotation markers once for the alliance, resets
odometry once, and starts a fresh runner. CTRE output uses field-relative velocity,
BlueAlliance perspective and wheel-speed desaturation. The request belongs to the
drivetrain, avoiding dangling references when changing chooser selections.

## Logs and live inspection

Schemas publish under `/Mayhem/Schema/{Superstructure,IntakeSequence,ShotCycle,Autonomous}`.
State, transition and guard topics are under `/Mayhem/States/<machine>/...`.
Autonomous transition IDs are prefixed by routine ID. Goal observations are under
`/Mayhem/Goals/Autonomous/{id,translationError,rotationError,reached}` (meters/radians).
A reached goal can be a moving handoff; measured residual errors are logged.

The observers use the existing robot DataLog, retaining USB `/U/wpilogs`, roboRIO
fallback and simulation `ROBOT_LOG_DIR`. Existing IO/pose/readiness/alert/timing
channels remain. Added WPILOG channels, under `/RealOutputs/`:

| Channel | Meaning |
| --- | --- |
| `Superstructure/State`, `PrevState`, `StateTime` | Resolved operating mode and actual entry age |
| `Superstructure/Fault` | Latched mechanism fault reason; cleared by a new valid request |
| `Superstructure/FeedAllowed` | Physical feed gate before its rising debounce |
| `Mayhem/States/Superstructure/inputs/*` | Typed intent/sensor inputs shared by the mechanism graphs |
| `Mayhem/States/{IntakeSequence,ShotCycle}/inputs/stateTime` | FPGA-time phase dwell used by watchdog guards |
| `Mayhem/Auto/{Status,Name,Target,FieldSpeeds,DeltaSeconds,ShotAimAcquired}` | Runner status, chooser label, anchor, drive output, loop dt and aim result |

These outputs also appear on NT4 under `/AdvantageKit/RealOutputs/`; state observer
topics use `/Mayhem/` directly. Manifests map every graph input/state to its actual
log channel. Import the `.wpilog` with the matching project to review decisions.
The live viewer subscribes only; it cannot operate the robot.

## Verification limits

The pose follower has no optimizer, collision avoidance, legacy force feedforward,
bridge recovery or automatic unbeaching. The preview is an ideal kinematic plant;
its curve and timing can differ from the compiled robot follower and real hardware.
Full-robot simulation complements it, but neither proves field clearance, actual
piece acquisition, shooting accuracy, or match readiness. Both x2 routines include
two 3.5-second shooting windows; verify the complete duration against the available
auto period. Do not use path-only preview time as a match-duration claim.

The C++ suite headers are vendored under `src/main/include/mayhem/*.hpp` with exact
hashes in `vendor-snapshot.json`; retain `BLine-Lib-LICENSE.txt` when updating.
Capitalized legacy headers remain only for chooser compatibility and regression
fixtures. No robot deployment has been performed by this integration.
