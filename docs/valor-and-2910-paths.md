# Valor 6800 and 2910 path ports

**The original 2910 bundled conversion below is superseded by [the movement corrections](imported-auto-corrections.md). See that document for current names, arrival regions, and front-intake heading conversions.**

These imports add editable driving paths to the existing Mayhem project, with new timing generated for this robot. Existing trajectories, robot configuration, field configuration, and auto chooser code are preserved.

## Folders and scope

- **6800 - Top - Depot** and **6800 - Bottom - HP Station**: ten legs per side. Shared legs are imported once per side rather than duplicated for every routine that uses them.
- **2910 - Top - Depot**: six sections, including depot collection/exit and the neutral-zone-to-depot continuation.
- **2910 - Bottom - HP Station**: three neutral-zone sections.

All new paths use the configured 0.30 m front intake extension. Initial paths exclude pose 1 from the intake range; continuation paths cover the full path. Field bump polygons export rough-terrain metadata with the existing Mayhem defaults. These are path/footprint constraints, not robot mechanism commands.

## Valor source and conversion

Source: [Valor 6800 robot repository](https://git.valor6800.com/valor6800/2026/robot), commit `2fc773e9ed0f9cebbfe0b90f18584c8185cd11fd`. Follower reference: the pinned Valkyrie submodule at `57adca50f78eb5671c816c2d2fc4db7db8863a8e`.

Ten Choreo trajectories are referenced by their autonomous classes: `safefirstleg`, `safesecondleg`, `safepsecondleg`, `aggrofirstleg`, `aggrosecondleg`, `optimizefirstleg`, `optimizesecondleg`, `pointfive`, `trenchfirstleg`, and `trenchsecondleg`. This includes the declared TrenchDoubleGrab/LineupMerc variants as well as the active chooser variants. Unreferenced experiments, diagnostic paths, and the unused PathPlanner files are excluded.

The conversion uses each trajectory's **snapshot**, which describes its generated trajectory, rather than potentially newer unsolved editor parameters. Coordinates are already blue-alliance coordinates. Depot/HP counterparts mirror Y around 8.0692 m and negate heading, matching their follower's same-alliance mirror. The TrenchDoubleGrab source starts on the opposite side from the other base trajectories; its naming is assigned from the actual geometry.

Choreo StopPoint, MaxVelocity, MaxAcceleration, MaxAngularVelocity, KeepInLane, KeepInRectangle, and KeepOutCircle constraints are converted. Circle keep-outs use circumscribed polygons. Mayhem applies keep-out polygons globally, so these are deliberately more conservative than the original scoped circles; source scope remains recorded in the manifest.

The destination intake requires local adaptations:

- Narrow lane tolerances are widened to at least **0.10 m**, or **0.30 m** for the Trench legs, so the extended footprint can pass the hub and trench posts.
- Most source anchors remain at their original coordinates. Where an anchor's extended footprint collides, a small circular position tolerance permits the solver to find clearance: **0.05 m** for one Poofs waypoint and **0.20/0.24 m** for specific Trench waypoints. The TrenchSecondLeg departure/return anchors move **0.04/0.22 m toward the wall**. Five additional clearance anchors, including three stops, route the robot around the post before turning and crossing. These are explicit destination-robot adaptations. Headings, obstacles, and intake coverage remain enabled.
- Every adaptation is listed in `third-party/valor6800/path-manifest.json`. The source snapshot is retained there for comparison.

Names are short, for example `DepotSafeFirstLeg`, `HPAggroSecondLeg`, and `DepotHalfLeg`. The HalfLeg's original nonzero terminal speed is not transferred as an exact velocity boundary; its endpoint remains free rather than forced to stop. Mayhem generates a new profile. Shooting dwell, intake delays, end-condition tolerances, and hub alignment remain robot-code work.

## 2910 source and conversion

Source: [FRCTeam2910/2026CompetitionRobot-Public](https://github.com/FRCTeam2910/2026CompetitionRobot-Public), commit `921c85bf725fbd9c488eb7d557e0e065640da095`.

The competition autos in `AutoFactory.java` currently drive to poses in `FieldConstants.java`. The stored PathPlanner passing/tuning files are not the routes used by those competition factories and are excluded. Ports cover LeftNeutralZone, RightNeutralZone, LeftNeutralZoneToDepot, and Depot. Shared neutral-zone sections are reused. The source's idle/test/homing/SysId programs are excluded.

Routes are split at stationary scoring boundaries, producing nine unique sections. Pose constants/getters are resolved from the blue-alliance source definitions using the welded field layout. The original left/right asymmetries and final third-intake target are retained. Per-leg speed caps transfer from the source's drive-to-point limits (4.572 / 3.3528 / 1.524 m/s).

Identical consecutive poses are consolidated. Same-position heading changes stop for the turn. The 0.01 m/s command raced against a 0.5-second timer is treated as a timed hold, rather than a route to traverse at 0.01 m/s; its subsequent normal-speed movement remains. Delays, shooting, and hold durations are not embedded in the driving time.

Several source targets at Y=7.7 m put this robot's bumper/intake too close to the wall. Their source drive-to-point commands already accept translation error. The port keeps those target centers visible and adds the minimum clearance radius plus 0.04 m, only when it fits inside the source's allowed tolerance. All other anchors remain exact. These adjustments and original calls are recorded in `third-party/frc2910/path-manifest.json`.

## Validation and source notices

All ports must be generated successfully before being copied into the robot project. Validation checks input hashes against the receiving project, finite/ordered samples, constrained waypoint positions/headings, intake deployment spans, and MayhemLib loading. Original source licenses are retained beside each team's manifest under `docs/third-party/`.

Reconstruction tools stage new files in a separate directory containing a copy of `project.mayhem`:

```sh
python tools/import_valor_paths.py /path/to/valor-checkout /path/to/staging-directory
python tools/import_2910_paths.py /path/to/2910-checkout /path/to/staging-directory
```

Use the Mayhem solver Python environment for the Valor converter's geometry dependency. Generate staged files before copying them into the project. These reconstruction scripts do not preserve edits made to the new imported definitions after import; do not rerun them over edited robot paths.
