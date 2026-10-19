> Current deployment: the eight edited Normal cycle/route variants are retained as complete paths, with the front-intake constraints. See [imported auto corrections](imported-auto-corrections.md). The historical import inventory below is retained for source reference.

# Team 581 comp-bot paths

Current edited project: eight Normal paths remain (four per side); the Special paths were deleted by the user. All eight now use the configured front intake footprint (0.30 m extension, full bumper width). `FirstCycle` paths constrain pose 2 through the end; all other paths constrain pose 1 through the end. Existing waypoints, non-intake constraints, and deletions are preserved. Intake spans are exported with the regenerated trajectories. The source-import inventory and times below describe the original import before those edits, rather than the current working set. Do not rerun the source importer to update these edited paths: it reconstructs the original source definitions.

Open `src/main/deploy/mayhem` in Mayhem. All 14 imported competition driving paths are in two folders:

- **581 - Top - Depot**: LeftNormal / LeftSpecial, seven paths.
- **581 - Bottom - HP Station**: RightNormal / RightSpecial, seven paths.

Folders use the routine's original starting side in Mayhem's **blue-alliance view**. Continuation and branch paths stay with their parent routine, even when they begin near the hub or midline. Team 581's original Left/Right naming is retained. Red preview rotates the physical positions to the other alliance.

| Path | Folder | Generated driving time |
| --- | --- | --- |
| `LeftNormalFirstCycle` | Top / Depot | 4.31 s |
| `LeftNormalSecondCycle` | Top / Depot | 7.01 s |
| `LeftNormalSecondCycleFar` | Top / Depot | 6.76 s |
| `LeftNormalThirdCycle` | Top / Depot | 6.84 s |
| `LeftSpecialIntakeAcrossMidline` | Top / Depot | 6.51 s |
| `LeftSpecialReturnAndShoot` | Top / Depot | 1.43 s |
| `LeftSpecialSecondIntake` | Top / Depot | 4.86 s |
| `RightNormalFirstCycle` | Bottom / HP Station | 4.31 s |
| `RightNormalSecondCycle` | Bottom / HP Station | 7.00 s |
| `RightNormalSecondCycleFar` | Bottom / HP Station | 6.76 s |
| `RightNormalThirdCycle` | Bottom / HP Station | 6.84 s |
| `RightSpecialIntakeAcrossMidline` | Bottom / HP Station | 4.79 s |
| `RightSpecialReturnAndShoot` | Bottom / HP Station | 1.43 s |
| `RightSpecialSecondIntake` | Bottom / HP Station | 3.34 s |

## Coverage and sequencing

Each side has the following paths:

- Normal: `FirstCycle`, `SecondCycle`, `SecondCycleFar`, and `ThirdCycle`. Run First -> Second -> Third, substituting SecondCycleFar for SecondCycle when selecting the far-lane branch. SecondCycleFar includes the shared departure from the shooting pose and switches to the far route at the preceding anchor of the source's decision window. In Team 581's runtime, the actual switch position depends on when the cluster-map decision occurs.
- Special: `IntakeAcrossMidline` -> `ReturnAndShoot` -> `SecondIntake`. The first path ends before the return over the bump, matching their segment boundary. Their source has a 3-second starting delay and a shooting dwell between the return and second intake; neither is part of these driving trajectories.

The active Team 581 chooser exposes LeftNormal and RightNormal. LeftSpecial and RightSpecial also exist in their comp-bot source and are included here as additional variants, although they are not registered in that chooser. No dumper-bot/beta-bot paths were imported. Diagnostic `TestAuto` / `IntegrationTest`, `DoNothingAuto`, repeated stationary shooting targets, and sensor-generated recovery maneuvers do not add competition driving routes and are excluded.

These are generated driving trajectories, not robot mechanism/state-machine implementations. They have not been registered in this robot's auto chooser. Shooting/intake requests, timing/delays, IMU crossing gates, collision offsets, un-beaching, and cluster-map decision logic still require robot code. Reference event markers have empty commands.

## Source and coordinates

Source: [team581/frc-2026](https://github.com/team581/frc-2026), pinned commit `5037ac428146c253d1648bcde6d95bbf15d0d2ef`, under `comp-bot/src/main/java/frc/robot/autos/auto_state_machines/`:

- [LeftNormalAuto.java](https://github.com/team581/frc-2026/blob/5037ac428146c253d1648bcde6d95bbf15d0d2ef/comp-bot/src/main/java/frc/robot/autos/auto_state_machines/LeftNormalAuto.java)
- [RightNormalAuto.java](https://github.com/team581/frc-2026/blob/5037ac428146c253d1648bcde6d95bbf15d0d2ef/comp-bot/src/main/java/frc/robot/autos/auto_state_machines/RightNormalAuto.java)
- [LeftSpecialAuto.java](https://github.com/team581/frc-2026/blob/5037ac428146c253d1648bcde6d95bbf15d0d2ef/comp-bot/src/main/java/frc/robot/autos/auto_state_machines/LeftSpecialAuto.java)
- [RightSpecialAuto.java](https://github.com/team581/frc-2026/blob/5037ac428146c253d1648bcde6d95bbf15d0d2ef/comp-bot/src/main/java/frc/robot/autos/auto_state_machines/RightSpecialAuto.java)

Team 581 authors red-alliance poses. Stored blue poses use their 180-degree rotation: `(x, y, heading) -> (16.541 - x, 8.069 - y, heading + pi)`, with heading normalized. The local welded AprilTag layout supplies tag 17/22 trench center Y coordinates (0.6444996 / 7.4247756 m). Their FieldUtil bump center distance from the wall is `(48.94 + 12 + 73/2) inches`; the Normal/Special offsets are evaluated separately from each source class.

The manifest `docs/team581-comp-path-manifest.json` maps every imported path to its source segments and lists the original red poses. Small source asymmetries, including the substantially different LeftSpecial/RightSpecial cross-field routes, are intentionally preserved rather than mirrored.

## Translation choices

- Every source target has a fixed position and heading. Trailblazer transition tolerances govern target switching, so they are not copied as Mayhem optimization tolerances. Curves and timing are newly generated using the existing robot/field configuration.
- Each trajectory begins at the routine's starting pose or its nominal predecessor endpoint. Endpoints stop; duplicate-position targets with different headings stop for the turn in place. Shooting dwell and switching velocities in the original imperative controller are not reproduced.
- Preserve segment velocity, acceleration, and angular velocity caps, plus tighter target-specific caps on the incoming range to that target. Mayhem additionally enforces this project's physical drivetrain limits. This Mayhem version has no explicit angular acceleration constraint, so source angular acceleration caps cannot be transferred directly.
- Supplier targets use their nominal no-collision poses. A supplier's bump-crossing request is not an extra path target; only its returned pose is imported.
- Event marker names remain associated with source anchors, with empty commands. Trailblazer exposes markers when a target becomes active; Mayhem fires them at the anchor time. Robot actions and exact source marker timing are not implemented.
- Existing field bump polygons supply rough-terrain zones with Mayhem defaults (speed fraction 0.7, feedback scale 0.3). These are crossing metadata, not an IMU controller. Intake deployment geometry is not added.

## Recreate and validate

With a clean source checkout at the pinned commit:

```sh
python3 tools/import_team581_comp_paths.py /path/to/team581-frc-2026
```

Then generate the imported paths in Mayhem. The importer recreates editable definitions and writes the source manifest; changed definitions need regeneration. It updates the two sidebar folders and leaves existing robot, field, commands, unrelated paths, and chooser code unchanged.

All 14 trajectories generated successfully and contain samples/recovery data. Validation checks original-to-blue pose conversion, every waypoint against generated samples, increasing timestamps, finite sample values, and the MayhemLib trajectory loader. Generated times above cover driving only.
