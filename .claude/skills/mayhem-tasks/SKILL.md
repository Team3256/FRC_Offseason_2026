---
name: mayhem-tasks
description: Work the team's Mayhem task queue for this robot repo - feedback people left in the Mayhem app (Tasks page) about how the robot behaves. Picks up new feedback, investigates it with the logs and the simulation, proposes a plain-English plan, and once a person approves it, fixes the robot code in a closed loop (code, sim, read the log, fix) and reports the change. Use when asked to check, work or run the Mayhem tasks, the task queue, or robot feedback, including from /loop.
---

# Work the Mayhem task queue

People on the team don't read code. They leave feedback in the Mayhem app and review what the robot *does*. Each run of this skill advances the queue by one step and stops; run it on a loop (`/loop 10m /mayhem-tasks`) to keep the queue moving.

## Where to run

In a checkout of the robot repo that has the Mayhem integration (`tools/mayhem/machines.json`; the `mayhem/suite-integration` branch, at `~/Downloads/FRC_Offseason_2026-suite-integration` on Ryan's Mac). Feedback lives in that checkout's `tools/mayhem/review/`. If the current folder has no `tools/mayhem/machines.json`, stop and say which folder to open instead.

## Tools

Use the Mayhem MCP tools (`mcp__mayhem__list_tasks`, `get_task`, `claim_task`, `propose_plan`, `report_progress`, `finish_task`, `release_task`, `run_scenarios`, `behavior_diff`, `log_states`, `log_anomalies`, `log_what_changed`, `log_table`, `analyze_log`). If they are not available, call the same tools from the shell, from a Mayhem checkout (`$MAYHEM_DIR`, default `../Mayhem` next to this repo):

```sh
node "${MAYHEM_DIR:-../Mayhem}/scripts/mayhem.mjs" tool list_tasks '{"repo": "."}'
```

`repo` is this repository's root. Your agent name is `claude` unless the user gave you one; use it for every task call.

## Each run: one step

1. `list_tasks` with `status: ["open", "claimed", "plan", "approved", "review", "done"]`. Handle the first that applies, then stop and report what you did in two or three sentences.
   - **A task you claimed went back to "claimed"** with `planDecision.approved: false`: read `planDecision.note`, revise, `propose_plan` again.
   - **A task approved for you** (`status: "approved"`, `agent` is you): implement it (below).
   - **A "done" task with commits whose branch has no pull request yet** (`gh pr list --head agent/<task id> --state all` is empty): open the pull request (below).
   - **A new task** (`status: "open"`): investigate and propose a plan (below). Skip thumbs-up items (status "noted") and anything outside robot code.
   - **Waiting on people** (only "plan" or "review" tasks are yours): say so and stop. Never approve or accept your own work.
2. If nothing to do, say "Queue empty" and stop.

## Investigate, then propose a plan (no code changes yet)

1. `get_task`. The `anchor` holds the context: `logPath` and `time` (a moment in a match log), `revision` (the commit the robot ran), `pose` (field spot), `routine` and `step` (an auto step), `machine`/`state`/`transition`, `scenario` (a sim proof run), `behavior` (a finding on the Changes page). Driver forms have `form` answers.
2. `claim_task`.
3. Reproduce before you explain:
   - From a log: `log_states` and `log_what_changed` around `anchor.time`, `log_table` of the suspects.
   - From the robot's behavior: `run_scenarios` with an inline scenario that does what the person described (steps like `"hold A 2"`, `"stick left 0 1"`, `"down LT"`; expectations like `"Shooter mode never SHOOTING"`). A failing scenario is the best evidence.
4. `propose_plan` in plain English, for drivers and mentors who don't read code: what will change on the robot, what will stay the same, and how they can check it (the scenario name, the button, the auto). Mention anything safety-related up front (current limits, CAN IDs, brake/coast, motor direction, gear ratios). One short paragraph plus up to three bullets. No code, no file names.
5. Stop. A person approves it on the Tasks page; a later run picks it up.

## Implement an approved task (closed loop)

1. Start a branch from the task's `base` (or `origin/main`): `git switch -c agent/<task id> <base>`. Never work on `main` or on a teammate's branch.
2. Change the code to do exactly what the approved plan says. If the plan turns out wrong, `release_task` with a note or propose a new plan instead of improvising.
   - Autos and state machines: edit `robot.mayhem.json` / `tools/mayhem/machines.json` with the Mayhem tools (`apply_machine_spec`, `import_mtraj_route`), then regenerate `MayhemProject.h` with `tools/mayhem/export.sh`. `behavior_diff` reports red if you forget.
   - Plain values: keep them as named constants in the subsystem's `*Constants.h` so people can see and edit them in the app.
3. Loop until it works: `run_scenarios {after: "WORKTREE", scenarios: [<your reproduction>], names: [...]}`, read the result's `timeline` (and `log_states` on its log when a state machine misbehaves), fix, repeat. A state that flickers back and forth is always a bug.
4. Add your reproduction to `tools/mayhem/scenarios.json` with its expectations, so it stays fixed.
5. Check nothing else moved: `run_scenarios {after: "WORKTREE", before: "<base>"}` (every button, every auto, every team scenario) and `behavior_diff {before: "<base>"}`. Any difference in a driver button's outcome that the plan didn't call for is a regression: fix it.
6. Commit with a short plain subject and the trailer `Mayhem-Task: <task id>`, push the branch, then `report_progress` with the commit and a one-line summary.
7. `finish_task` with a note naming the sim proof result. Green changes (auto paths only, every path finishes) are accepted automatically; everything else waits for a person in "Ready to check".

## Open the pull request (after a person accepts, or auto-accepted)

`gh pr create --base main --head agent/<task id>` titled with the task title. The body: the feedback (who, what, where), the approved plan, the sim proof result (scenarios added, buttons/autos unchanged), and the `behavior_diff` risk. People review behavior on the Mayhem Changes page, not the diff. Do not merge it yourself.

## Rules

- Never change behavior the plan didn't mention; same driver inputs must give the same outcomes.
- Never deploy to the robot.
- Never commit generated build files or `src/main/deploy/mayhem-build.json`.
- If the build or the sim toolchain fails, report it on the task with `release_task` and a note instead of guessing.
