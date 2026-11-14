#!/usr/bin/env python3
# Vendored from Mayhem v0.9.28 (scripts/sim/montecarlo.py, github.com/ryanabraham1/Mayhem); update it there.
"""Run many seeded copies of a robot simulation test and summarize them.

    python3 scripts/sim/montecarlo.py --cmd "./build/test/simAuto --auto Left4" --runs 200 --jobs 8 --out sim-runs

Each run gets MAYHEM_SIM_SEED=<seed> and MAYHEM_SIM_OUT=<out>/run_<seed>.json in its environment
(seeds start at --seed, default 1). The command should write that JSON:

    {"passed": true, "failure": "", "metrics": {"autoTime": 14.2, "scored": 6}, "log": "logs/run_7.wpilog"}

A run that exits non-zero or writes no JSON counts as failed with failure "exit <code>" or "no
result". The summary prints failure modes by frequency, per-metric percentiles (p5, p50, p95,
min, max) and the worst seeds' logs to open in AdvantageScope; --json writes it as JSON too.
No dependencies beyond Python 3.10.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import shlex
import subprocess
import sys
import time
from collections import Counter
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path


def run_one(cmd: list[str], seed: int, out_dir: Path, timeout: float, cwd: str | None) -> dict:
    result_path = out_dir / f"run_{seed}.json"
    result_path.unlink(missing_ok=True)
    env = dict(os.environ, MAYHEM_SIM_SEED=str(seed), MAYHEM_SIM_OUT=str(result_path))
    started = time.monotonic()
    try:
        proc = subprocess.run(cmd, env=env, cwd=cwd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=timeout)
        code, output = proc.returncode, proc.stdout.decode(errors="replace")
    except subprocess.TimeoutExpired as e:
        code, output = None, (e.stdout or b"").decode(errors="replace")
    elapsed = time.monotonic() - started
    (out_dir / f"run_{seed}.out").write_text(output)
    record: dict = {"seed": seed, "seconds": elapsed, "exit": code}
    try:
        data = json.loads(result_path.read_text())
    except (OSError, json.JSONDecodeError):
        data = None
    if data is None:
        record.update(passed=False, failure="timeout" if code is None else ("no result" if code == 0 else f"exit {code}"),
                      metrics={}, log=None)
    else:
        record.update(passed=bool(data.get("passed")) and code == 0, failure=str(data.get("failure") or ""),
                      metrics={k: v for k, v in (data.get("metrics") or {}).items() if isinstance(v, (int, float))},
                      log=data.get("log"))
        if code != 0 and not record["failure"]:
            record["failure"] = "timeout" if code is None else f"exit {code}"
    if not record["passed"] and not record["failure"]:
        record["failure"] = "failed"
    return record


def percentile(sorted_values: list[float], q: float) -> float:
    if not sorted_values:
        return math.nan
    k = (len(sorted_values) - 1) * q
    lo, hi = math.floor(k), math.ceil(k)
    return sorted_values[lo] + (sorted_values[hi] - sorted_values[lo]) * (k - lo)


def summarize(records: list[dict], worst_metric: str | None, worst_count: int, lower_is_worse: bool) -> dict:
    failures = Counter(r["failure"] for r in records if not r["passed"])
    metrics: dict[str, list[float]] = {}
    for r in records:
        for k, v in r["metrics"].items():
            if math.isfinite(v):
                metrics.setdefault(k, []).append(float(v))
    table = {}
    for k, values in sorted(metrics.items()):
        values.sort()
        table[k] = {"n": len(values), "p5": percentile(values, 0.05), "p50": percentile(values, 0.5),
                    "p95": percentile(values, 0.95), "min": values[0], "max": values[-1],
                    "mean": sum(values) / len(values)}
    # Worst seeds: failures first (by failure frequency), then by the chosen metric.
    def badness(r: dict):
        metric = r["metrics"].get(worst_metric, math.nan) if worst_metric else math.nan
        if not math.isfinite(metric):
            metric = math.inf if lower_is_worse else -math.inf
        return (r["passed"], metric if lower_is_worse else -metric, r["seed"])
    worst = [{"seed": r["seed"], "passed": r["passed"], "failure": r["failure"], "log": r["log"],
              **({worst_metric: r["metrics"].get(worst_metric)} if worst_metric else {})}
             for r in sorted(records, key=badness)[:worst_count]]
    return {"runs": len(records), "passed": sum(r["passed"] for r in records), "failures": dict(failures.most_common()),
            "metrics": table, "worst": worst}


def print_summary(s: dict) -> None:
    runs, passed = s["runs"], s["passed"]
    print(f"\n{passed}/{runs} passed ({100.0 * passed / max(runs, 1):.1f}%)")
    if s["failures"]:
        print("\nFailure modes:")
        for failure, n in s["failures"].items():
            print(f"  {n:5d}  {100.0 * n / runs:5.1f}%  {failure}")
    if s["metrics"]:
        width = max(len(k) for k in s["metrics"])
        print(f"\n{'metric':<{width}}  {'n':>5}  {'p5':>10}  {'p50':>10}  {'p95':>10}  {'min':>10}  {'max':>10}")
        for k, m in s["metrics"].items():
            print(f"{k:<{width}}  {m['n']:>5}  {m['p5']:>10.4g}  {m['p50']:>10.4g}  {m['p95']:>10.4g}  {m['min']:>10.4g}  {m['max']:>10.4g}")
    if s["worst"]:
        print("\nWorst seeds (open the logs in AdvantageScope; rerun one with MAYHEM_SIM_SEED=<seed>):")
        for w in s["worst"]:
            extra = "  ".join(f"{k}={v}" for k, v in w.items() if k not in {"seed", "passed", "failure", "log"})
            print(f"  seed {w['seed']:>6}  {'pass' if w['passed'] else 'FAIL'}  {w['failure'] or '':<24} {extra}  {w['log'] or '(no log)'}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--cmd", required=True, help="command to run (shell-quoted string)")
    parser.add_argument("--runs", type=int, default=100)
    parser.add_argument("--jobs", type=int, default=os.cpu_count() or 4)
    parser.add_argument("--seed", type=int, default=1, help="first seed")
    parser.add_argument("--out", type=Path, default=Path("sim-runs"))
    parser.add_argument("--timeout", type=float, default=600, help="seconds per run")
    parser.add_argument("--cwd", help="working directory for the command")
    parser.add_argument("--worst", type=int, default=5, help="how many worst seeds to list")
    parser.add_argument("--worst-metric", help="rank passing runs by this metric")
    parser.add_argument("--higher-is-worse", action="store_true", help="for --worst-metric (default: lower is worse)")
    parser.add_argument("--json", type=Path, help="also write the summary here")
    args = parser.parse_args()

    cmd = shlex.split(args.cmd)
    args.out.mkdir(parents=True, exist_ok=True)
    out_dir = args.out.resolve()
    seeds = list(range(args.seed, args.seed + args.runs))
    records: list[dict] = []
    started = time.monotonic()
    with ThreadPoolExecutor(max_workers=max(1, args.jobs)) as pool:
        futures = [pool.submit(run_one, cmd, seed, out_dir, args.timeout, args.cwd) for seed in seeds]
        for i, future in enumerate(as_completed(futures), 1):
            r = future.result()
            records.append(r)
            mark = "." if r["passed"] else "F"
            sys.stdout.write(mark)
            if i % 50 == 0:
                sys.stdout.write(f" {i}/{len(seeds)}\n")
            sys.stdout.flush()
    records.sort(key=lambda r: r["seed"])
    summary = summarize(records, args.worst_metric, args.worst, not args.higher_is_worse)
    summary["wallSeconds"] = time.monotonic() - started
    print_summary(summary)
    print(f"\n{len(records)} runs in {summary['wallSeconds']:.1f} s with {args.jobs} jobs; outputs in {out_dir}")
    if args.json:
        args.json.write_text(json.dumps({**summary, "records": records}, indent=2) + "\n")
    return 0 if summary["passed"] == summary["runs"] else 1


if __name__ == "__main__":
    sys.exit(main())
