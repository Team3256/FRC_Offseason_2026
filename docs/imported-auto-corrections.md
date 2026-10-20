# Imported autonomous route options

The deployed paths are complete cycle and route variations, not individual point-to-point commands. The original 581 edited Normal routes are retained. 2910 routes use corrected handoff geometry so large drive-to-point arrival tolerances do not force the robot through distant target centers.

## Available options

- **581 (8 paths):** `LeftNormalFirstCycle`, `LeftNormalSecondCycle`, `LeftNormalSecondCycleFar`, `LeftNormalThirdCycle`, and their `Right` counterparts. These preserve the user's edited coordinates, headings, constraints, markers, and waypoint deletions.
- **2910 (11 paths):** `DepotNZFirstCycle`, `DepotNZSecondCycle`, `DepotNZThirdIntake`; `HPNZFirstCycle`, `HPNZSecondCycle`, `HPNZThirdIntake`; `DepotNZAltFirstCycle`, `DepotNZAltSecondCycle`, `DepotNZToDepot`; and `DepotCollection`, `DepotExit`.
- **Valor (20 paths):** the existing leg names are retained.

Files remain in each team's Top / Depot or Bottom / HP Station folders. The 2910 `Alt` paths come from the NeutralZoneToDepot source routine, which has a different first-sweep threshold. Use the two Alt cycles followed by `DepotNZToDepot` for that variation. `DepotCollection` followed by `DepotExit` represents the source's separate depot routine.

`third-party/grouped-route-manifest.json` records the source commands and internal handoff poses for every 2910 option. Score, hold, wait, intake, and branch commands still belong in robot code; no autonomous chooser is installed by this import. Source shooting and hold durations are not included in the generated driving time.

## Geometry and timing

2910's source advances drive-to-point commands on distance tolerances, sometimes several meters before the target center. An internal reconstruction solves those arrival regions sequentially and carries each solved endpoint forward. The final route joins these handoff poses into a continuous multi-waypoint trajectory. Interior waypoints flow through instead of stopping at every command; route endpoints stop for scoring or completion. Headings rotate for our intake convention.

The point-to-point reconstruction is an internal aid only. Its 126 temporary movements are not deployed. `third-party/split-movement-manifest.json` retains their target/radius evidence; names in that internal manifest are not selectable production paths. `tools/group_imported_autos.py` generates the complete options from that reconstruction. The source controllers and our optimizer have different timing, so these are reconstructed routes rather than recordings of another team's hardware motion.

The 2910 depot-right target (0.65, 5.8) requires a 0.22 m arrival radius for our longer front intake at the corrected heading, exceeding the source's 0.1524 m default. The internal manifest flags this destination-robot wall-clearance adjustment. Other source arrival regions are preserved during reconstruction. Final multi-waypoint routes use the chosen clearance-valid handoff poses, not the distant target centers.

The eight edited 581 parent definitions are retained without generated output under `third-party/pre-split-definitions/`. The nine superseded 2910 bundle definitions there are historical references only and should not be copied back into deploy. Existing 581 variants and source transition behavior are documented in `team581-comp-bot-examples.md` and the source manifests.

## Intake direction and footprint

Our intake is on robot +X. Existing Valor headings rotate by pi radians, preserving every waypoint's X/Y coordinates and existing clearance anchors. 2910 headings also rotate by pi: their `SwerveSubsystem.seekingWheelchair` explicitly adds `Rotation2d.kPi` when computing intake-direction translation, and uses a negative-X seeking rotation offset. 581's `ClusterMap` points the robot toward front-facing clusters, so its headings are preserved.

The configured 0.30 m front intake extension remains enabled through imported continuation paths. As in the edited parent paths, first-cycle paths exclude their initial pose from the intake range. Field obstacles remain enabled and bump zones retain rough-terrain metadata.

The heading correction changes footprint clearance. `third-party/valor6800/heading-corrections.json` records additional circular waypoint tolerances required by the front intake. Existing tolerances are never tightened and coordinates remain unchanged.

## Reproduction and validation

`tools/split_target_autos.py` produces internal arrival-region reconstructions from the source checkouts and edited 581 parents. `tools/group_imported_autos.py` restores complete route options from the archived 581 definitions and the reconstruction stage. Both tools stage their output away from deploy.

`tools/rotate_valor_intake.py` stages the heading correction from the pre-correction workspace definitions. Do not rerun it over already-corrected paths or it would rotate them again. The raw Valor source importer also applies the front-intake heading conversion for fresh imports.

The installed Mayhem solver must generate successful outputs before publication. The Mayhem MCP `inspect_project` operation checks saved-output freshness, and `deploy_trajectories` publishes explicit path names locally. Validation checks finite/ordered samples, route endpoint continuity, preservation of edited 581 inputs and Valor coordinates, and MayhemLib trajectory loading. These are simulation and file checks, not hardware execution results.
