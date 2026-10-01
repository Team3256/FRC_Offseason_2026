# Max-fuel Mayhem autos

Five auto options built from what the best 2026 dumper robots ran. The goal is the most FUEL in the
HUB during the 20 s AUTO, and every option is meant to fit with the other two robots on an alliance.
Paths are in `src/main/deploy/mayhem/`, authored for blue (MayhemLib flips them for red). The routines
that run them are in `MaxFuelMayhemAutos.java`.

| # | Auto | Paths | Fuel plan | Lanes used |
|---|------|-------|-----------|------------|
| 1 | NZ Double (top) | `nzDoubleTopPt1`, `nzDoubleTopPt2` | preload + load from the top half, then a load from the lower half | top trench out, top bump back, then top bump out / bottom bump back |
| 2 | NZ Double (bottom) | `nzDoubleBottomPt1`, `nzDoubleBottomPt2` | mirror of 1 | bottom trench, bottom bump, then bottom bump out / top bump back |
| 3 | Depot + Trench Sweep | `depotPt1`, `depotTrenchSweepPt2` | preload + depot (32), then one full load from the top half | top trench out, top bump back |
| 4 | Depot + Bump Sweep | `depotPt1`, `depotBumpSweepPt2` | preload + depot (32), then one load over the top bump | top bump only, both trenches free |
| 5 | Depot Only | `depotPt1`, `depotDumpPt2` | preload + depot (32), never leaves the alliance zone | none |

Images of every path are in this folder (`1-nz-double-top.png` ... `5-depot-only.png`).

## What the best dumpers actually did

Mostly from 2910's public 2026 code ([FRCTeam2910/2026CompetitionRobot-Public](https://github.com/FRCTeam2910/2026CompetitionRobot-Public)),
the Einstein finalist and a drum-shooter dumper, plus the Chief Delphi auto threads and the Einstein
match results.

- **2910 `LEFT_NEUTRAL_ZONE`**: out the trench with the intake down, sweep the neutral zone, come back
  over the bump, **stop and dump the whole hopper** (`scoreStationary`, up to 4 s, after the intake
  retracts), then go out the trench again for a second, different sweep and dump again. Right side is
  the mirror. The dump is stationary, so the auto is two big loads and not a stream of shots.
- **2910 `DEPOT`**: start in the middle, drive the depot (right, middle, left, with 1 s waits), dump
  from a pose near the depot, then run out the trench to the neutral zone centre.
- **Coordination meta** ([Triple Neutral Zone Autos](https://www.chiefdelphi.com/t/triple-neutral-zone-autos-an-update/519765),
  [Middle auto meta](https://www.chiefdelphi.com/t/middle-auto-meta-strategy/518735)): the two strong
  robots each take the neutral zone twice, the first pass at a predictable spot and the second a wide
  sweep for scattered fuel. The third robot takes preload + depot (8 + 24 = 32), and the better version
  waits a second or two for everyone to clear the middle before one more pass. Most robots had left the
  midline by about 5 s and came back around 9 s.
- **4414** (a turret robot, not a dumper, so used only as a benchmark): picked one depot auto with a
  single bump crossing that they could run in every match whatever their partners did, and found
  two-swipe versions scored less because shooting time ran out ([thread](https://www.chiefdelphi.com/t/team-4414-hightide-2026-tech-binder-ripcurrent/519602?page=7)).
- **Einstein finals** ([FIRST event data](https://frc-events.firstinspires.org/2026/CMPTX/playoffs/16)): the
  winning alliance (4065 / 4414 / 1323) scored 111 to 201 AUTO fuel points per match across three robots
  (about 60 each), the runner-up (868 / 2910 / 2046) 106 to 154.
- Big dumpers win auto because they fill the hopper and unload fast ([Starving and Snaring the Big Dumper](https://www.chiefdelphi.com/t/starving-and-snaring-the-big-dumper/517179)).

## Design rules these paths follow

1. **Few, big loads.** Every trip out costs about 3 s of travel, so each trip fills the hopper and then
   dumps, as 2910 does.
2. **Reverse over the bump.** Our shooter is on the back (`ShotCalculator` aims the drive angle at the
   hub plus 180°). Driving back over the bump rear-first leaves the shooter already pointed at the hub, so
   the robot arrives ready to shoot. The last waypoint carries a *point at hub, flipped* constraint.
3. **Shoot where the hub is targeted.** `Superstructure` only targets the hub when the robot is at
   x < 4 m (blue), so every dump starts inside the alliance zone. The `shoot` marker sits 0.8 s before
   arrival so the flywheel is up when the robot stops.
4. **Enter the pile front first.** In the Mayhem fuel sim, entering from the side lets the bumper scatter
   fuel before the intake reaches it. All sweeps enter from above, intake leading.
5. **Stay in your half.** Autos 1 to 4 keep their first load in the top half (y > 4.3) so a partner can
   take the bottom half.
6. **Depot is the cheapest fuel.** 24 balls with no bump or trench crossing, so autos 3 to 5 start there.
   The 32 fuel are shot on the move leaving the depot (still inside the alliance zone), which saves a
   stationary dump.

## Picking an auto to fit your alliance partners

Each auto leaves a different footprint, so pick the one that stays out of what your partners run.
Times are on the auto clock, including the depot and dump waits in `MaxFuelMayhemAutos`.

| Auto | Start | Neutral zone (x > 5.3 m) | Crossings | Dump pose(s) | Uses depot |
|------|-------|--------------------------|-----------|--------------|------------|
| 1 NZ Double (top) | top trench (4.63, 7.39) | 0.4 - 7.2 s, 13.5 - 17.4 s | top trench 0 s, top bump 7.3 s and 13.1 s, bottom bump 17.4 s | (3.3, 5.65) then (3.3, 2.3) | no |
| 2 NZ Double (bottom) | bottom trench (4.63, 0.68) | 0.4 - 7.2 s, 13.5 - 17.4 s | bottom trench 0 s, bottom bump 7.3 s and 13.1 s, top bump 17.4 s | (3.3, 2.4) then (3.3, 5.77) | no |
| 3 Depot + Trench Sweep | alliance zone (3.55, 4.95) | 7.6 - 14.4 s | top trench 7.2 s, top bump 14.4 s | (3.3, 5.65) | yes |
| 4 Depot + Bump Sweep | alliance zone (3.55, 4.95) | 7.4 - 13.9 s | top bump 7.1 s and 13.9 s | (3.3, 5.65) | yes |
| 5 Depot Only | alliance zone (3.55, 4.95) | never | none | shoots while backing out, parks at (2.0, 4.6) | yes |

If your partners run:

- **Both a depot auto and a top-side neutral zone auto** (the common case): take **2**. Its first load
  is the bottom half, finished by 7 s, and it only reaches the top half at about 15 s, after the top-side
  robots have usually left.
- **A depot auto and a bottom-side neutral zone auto**: take **1**, the mirror of the above.
- **A depot auto only**: take **1** or **2**, whichever side your other partner is not on. Do not take 3 or 4,
  there is one depot.
- **No depot auto**: take **3** (more fuel, uses the top trench from 7 s) or **4** (keeps both trenches clear).
- **Neutral zone autos on both sides, or you want zero contact risk**: take **5**. It never leaves the
  alliance zone and parks at (2.0, 4.6), clear of every dump pose above, for 32 fuel.

Do not run two of these on the same alliance if they share a dump pose: **1, 3 and 4 all dump at
(3.3, 5.65)**, and 2's second dump is the mirror image of that spot. Autos 1 and 2 together also cross the
alliance zone to opposite bumps at the same time (13 to 17 s).

## What is modelled, and what is not

The Mayhem fuel sim was run on every path (rate 10 fuel/s, the project default). The sim has no depot
walls, so the 24 depot fuel is assumed instead of simulated. Assuming a 10 fuel/s shooter, a hopper that
holds about 50, and the 8 preload fuel, the totals work out to:

| Auto | Preload | Depot | Neutral zone loads (sim) | Total | Last shot |
|------|---------|-------|--------------------------|-------|-----------|
| 1 / 2 NZ Double | 8 | - | 35 + 11 | about 54 | about 18.5 s |
| 3 Depot + Trench Sweep | 8 | 24 | 37 | about 69 | about 19 s |
| 4 Depot + Bump Sweep | 8 | 24 | 31 | about 63 | about 18 s |
| 5 Depot Only | 8 | 24 | - | 32 | about 7.6 s |

For reference the Einstein winners averaged about 60 AUTO fuel points per robot.

Treat these as relative numbers, not predictions. They depend on three values this repo does not record:

- the **intake rate** (fuel/s into the hopper). The sim default is 10 and the loads scale with it, so a
  faster intake needs less time in the pile. Change the `Max velocity` constraint over the intake span of
  a path (the slow section in each sweep) to match.
- the **hopper capacity**. The loads are sized for about 40 fuel in the pile plus the preload.
- the **shooter rate**. `kFirstDumpSeconds` and `kDepotIntakeSeconds` in `MaxFuelMayhemAutos` are the
  waits that hold the robot still while the hopper empties and the depot fills. They are guesses for
  the same 10 fuel/s.

Not covered: shooting accuracy on the move (autos 3 to 5 shoot while driving out of the depot), scatter
from partners, and bump timing on real carpet. The sim is a rough 2D model, so a path that looks good
there still needs a practice-robot pass.

## Regenerating

Open `src/main/deploy/mayhem` as a project in the Mayhem app and press Generate after any edit.
`MayhemTrajectoriesTest` fails if a path is stale.
