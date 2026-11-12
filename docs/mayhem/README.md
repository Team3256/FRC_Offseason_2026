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

## Driver controls: same inputs, same outcomes

| Input | Outcome |
| --- | --- |
| A (tap) | SHOOT. Feeds as soon as the flywheel first reaches speed and keeps feeding until the request ends. Held A also turns to the shot (aim assist). Aim is logged but not required. |
| X | INTAKE (SHOOT_AND_INTAKE while shooting) |
| Y | JITTER (JITTER_AND_SHOOT while shooting) |
| B | HOME: rollers, shooter, indexer, feeder off; slide to ground intake; hood 0.05 |
| Left trigger | OUTTAKE |
| Bumpers, D-pad, trench assist | unchanged |

Deliberate improvements. None of them changes what a button press ends up doing:

- **The hood clears the slide whenever the slide moves through its travel.** This covers HOME,
  SHOOT/REV_AND_INTAKE and JITTER, not only INTAKE. The slide waits for the hood
  (`IntakeSequence: CLEARING_HOOD`).
- **Feeding pauses under the trench** (where the hood is forced flat) and resumes on exit
  (`ShotCycle: INHIBITED`).
- **Entering the alliance zone pre-spins** an idle or intaking robot, but no longer replaces an
  active SHOOT with REV.
- **The shot target is picked from the pose every loop**, including at startup.
- **Disabling clears old requests**, so A works on the first press of teleop.
- **Slow mechanisms raise dashboard alerts** (hood, slide, flywheel) instead of cancelling your request.

## How the state machines fit together

- `Superstructure`: driver/auto intent resolves to the operating mode. Priority: cancel, home,
  outtake, shoot+jitter, shoot+intake, shoot, jitter, rev+intake, intake, rev, idle.
- `IntakeSequence`: PARKED → CLEARING_HOOD → DEPLOYING → READY, with JITTERING. The slide commands
  run from these phases.
- `ShotCycle`: OFF → SPINNING_UP → FEEDING (latched), plus INHIBITED under the trench. The indexer
  and feeder run in FEEDING.

`tools/mayhem/phase_graphs.py robot.mayhem.json` regenerates the two phase graphs, plus every
transition's name and description and every input's category. Categories: operator = driver
request, sensor = measured, internal = derived in code. The robot runs all three with
`mayhem::StateMachine`.

## Autonomous

Each routine follows its original solved sweep, sampled every ~0.6 m from the `.mtraj` passes in
`tools/mayhem/solved/`. Every point carries the solved heading and speed, so the robot crosses the
trench sideways and **leads with the intake down the centerline pickup**.
`tools/mayhem/rebuild_routes.py .` rebuilds the routines. Drivetrain limits come from
`tools/mayhem/robot_limits.py --write`; it uses the motor curve, gearing and slip current from
`TunerConstants.h` at 150 lb.

After each pass, `drivetrain.alignAndShoot` behaves like the original AlignAndShoot:
- the drivetrain's heading controller turns to the lookahead shot;
- SHOOT is requested once within 3°;
- after 3.5 s the routine continues. A missed shot never aborts the second pass.

The x2 second passes now intake. In the original code the `intake2` marker was missing from the
path, so pass 2 never intaked.

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
