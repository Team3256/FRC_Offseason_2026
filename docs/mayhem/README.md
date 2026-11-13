# Mayhem integration

`robot.mayhem.json` is the single Mayhem project for this robot. Open it in the Mayhem desktop app
(Open project file) to see and edit:

- the five autonomous routines (chooser names and order come from the project), and
- the three superstructure state machines (`Superstructure`, `IntakeSequence`, `ShotCycle`).

`tools/mayhem/export.sh` runs Mayhem's own C++ exporter and writes
`src/main/include/commands/MayhemProject.h` (namespace `mayhem_generated`). Commit both files together.
CI runs `tools/mayhem/export.sh --check`: it validates the project, fails on a stale header, and
fails if the vendored headers differ from Mayhem. The script uses the Mayhem commit recorded in
`docs/mayhem/vendor-snapshot.json`. It clones that commit into `build/mayhem-tools`, or uses
`MAYHEM_DIR` if set, and needs Node 24+.

The Mayhem C++ headers are vendored in `src/main/include/mayhem/*.hpp` by
`node <Mayhem>/scripts/mayhem.mjs vendor . --snapshot docs/mayhem/vendor-snapshot.json`. The BLine-Lib
license is in `BLine-Lib-LICENSE.txt`. Switch to the `MayhemCpp.json` vendordep once Mayhem hosts it.

## Driver controls

| Input | Mode |
| --- | --- |
| A (hold) | Shooter: SHOOTING. Spins up, then feeds once the flywheel first reaches speed, until A is released. Also turns the robot to the shot. |
| X (press) | Intake: INTAKING on / off (STOWED) |
| Y (hold) | Intake: JITTERING, then back to INTAKING on release |
| B (press) | Intake: DEPLOYED (slide out, rollers off) on / off |
| Left trigger (hold) | Intake: OUTTAKING (rollers and feeder reverse; stops the shot), then STOWED |
| Bumpers, D-pad, trench assist | drive modes, shot speed, heading reset |

The shooter and the intake are independent: shooting while intaking, or while jittering, is just
both modes at once. Inside the alliance zone the shooter pre-spins (REV) whenever it is not
shooting.

## How the state machines fit together

- `ShooterMode`: OFF, REV, SHOOTING (what the shooter is doing).
- `IntakeMode`: STOWED, INTAKING, JITTERING, DEPLOYED, OUTTAKING (what the intake is doing).
- `IntakeSequence`: PARKED → CLEARING_HOOD → DEPLOYING → READY, with JITTERING. The slide only moves
  in these phases, and it waits for the hood to clear it.
- `ShotCycle`: OFF → SPINNING_UP → FEEDING (latched), plus INHIBITED under the trench, where the
  hood is forced flat. The indexer and feeder run in FEEDING.

The two mode machines say what is wanted; the two phase machines do the physical sequencing under
them. Slow phases raise dashboard alerts (hood, slide, flywheel) instead of cancelling anything.
Without a valid shot distance the flywheel stays off and an alert says why. Autos ask through
`superstructure.intake` / `superstructure.stow` events and the aim-and-shoot action; when an auto
ends, everything stops.

`tools/mayhem/phase_graphs.py robot.mayhem.json` regenerates the four graphs, plus every
transition's name and description and every input's category. Categories: operator = driver
request, sensor = measured, internal = derived in code. The robot runs all four with
`mayhem::StateMachine`.

## Autonomous

Each routine goes through the key waypoints of its original solved sweep (start, corners, pickup,
shot spot) from the `.mtraj` passes in `tools/mayhem/solved/`; the old paths are a reference, not a
path to trace exactly. Every point carries the solved heading and speed, so the robot crosses the
trench sideways and **leads with the intake down the centerline pickup**.
`tools/mayhem/rebuild_routes.py .` rebuilds the routines. Drivetrain limits come from
`tools/mayhem/robot_limits.py --write`; it uses the motor curve, gearing and slip current from
`TunerConstants.h` at 150 lb.

After each pass, `drivetrain.alignAndShoot` behaves like the original AlignAndShoot:
- the drivetrain's heading controller turns to the lookahead shot;
- SHOOT is requested once within 3°;
- after 3.5 s the routine continues. A missed shot never aborts the second pass.

The x2 second passes now intake. In the original code the `intake2` marker was missing from the
path, so pass 2 never intaked. Their waypoints are hand-placed (`MANUAL` in `rebuild_routes.py`),
with the solved pass only as the general direction. They turn square near the shot spot, clear of
the wall and the trench post, and then drive the trench opening's centre line. Depot x2 comes back
north at x ≈ 6 and crosses the bump square, well below the trench post.

The simulation checks every routine on both alliances:
- driving time per pass;
- the intake leading through the pickup (median heading error vs travel < 35°);
- wall clearance;
- the end pose.

Each pass drives within ~0.4 s of the old solved time (2.6-6.0 s per pass);
see the PR for the numbers.

## Logs

The observers write to the robot's existing DataLog (USB `/U/wpilogs`, roboRIO fallback, sim
`ROBOT_LOG_DIR`) and NT4:

| Channel | Meaning |
| --- | --- |
| `Mayhem/Schema/<machine>` | Machine manifest (states, transitions, inputs) |
| `Mayhem/States/<machine>/{state,transition,from,to}` | State entries and transitions |
| `Mayhem/States/<machine>/guards/<transition>/condition` | Guard results |
| `Mayhem/States/Superstructure/inputs/*` | Every signal the graphs read |
| `Mayhem/Goals/Autonomous/*` | Active goal, translation/rotation error, reached |
| `RealOutputs/Superstructure/{State,PrevState,StateTime,Fault,FeedAllowed}` | Mode and diagnostics |
| `RealOutputs/Mayhem/Auto/{Status,Name,Target,FieldSpeeds,DeltaSeconds,ShotAimAcquired}` | Auto runner |

Import a `.wpilog` together with this project in Mayhem's Logs view to replay decisions.
