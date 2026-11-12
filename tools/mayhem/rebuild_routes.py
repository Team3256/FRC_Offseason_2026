"""Rebuild robot.mayhem.json routines from the original solved .mtraj passes.

Each original waypoint becomes a pose step whose heading is the solved heading at that waypoint;
rotation markers inside each segment carry the solved heading profile, so the intake leads where it
did in the original paths. Segment speed caps come from the solved speed profile.
usage: routes.py <robot repo> [json overrides]
"""
import json, math, sys
from pathlib import Path
repo = Path(sys.argv[1]); P = repo / "robot.mayhem.json"
cfg = dict(speedScale=1.0, cornerRadius=0.3, spacing=0.6, minDt=0.15, lookahead=0.2, vfloor=0.8, vmax=4.8, finalCap=3.0, translationKp=4.5, capRule="end", finalTolerance=0.15)
cfg["vmax"] = json.loads(P.read_text())["robot"]["constraints"]["maxVelocity"]
if len(sys.argv) > 2: cfg.update(json.loads(sys.argv[2]))
W = lambda a: math.atan2(math.sin(a), math.cos(a))
ROUTINES = {
    "depot-sweep-depot": ["DepotTrenchSweepDepotPt", "DepotTrenchSweepDepotPt2"],
    "outpost-sweep": ["OutpostTrenchSweepBumpPt1"],
    "outpost-sweep-x2": ["OutpostTrenchSweepBumpPt1", "OutpostTrenchSweepBumpPt2"],
    "depot-sweep": ["DepotTrenchSweepBumpPt1"],
    "depot-sweep-x2": ["DepotTrenchSweepBumpPt1", "DepotTrenchSweepBump2xPt2"],
}
INTAKE = {"intake", "intake2"}
def load(name): return json.load(open(repo / f"tools/mayhem/solved/{name}.mtraj"))
def heading_at(samples, t):
    best = min(samples, key=lambda s: abs(s["t"] - t)); return W(best["heading"])
def pass_nodes(rid, pi, name, last_pass):
    d = load(name); s = d["output"]["samples"]; wt = d["output"]["waypointTimes"]
    events = {e["name"]: e["t"] for e in d["output"]["events"]}
    intake_t = next((events[k] for k in events if k in INTAKE), 0.0)
    T = s[-1]["t"]
    # Pick sample indices: every `spacing` meters of arc, plus each original waypoint time.
    keep, arc, last = [], 0.0, 0.0
    for a, b in zip(s, s[1:]):
        arc += math.hypot(b["x"] - a["x"], b["y"] - a["y"])
        if arc - last >= cfg["spacing"]: keep.append(b["t"]); last = arc
    keep += wt[1:]
    times = sorted(set(round(t, 3) for t in keep if t > 0.05))
    merged = []
    for t in times:
        if merged and t - merged[-1] < cfg["minDt"] and t != round(T, 3): continue
        if merged and t == round(T, 3) and t - merged[-1] < cfg["minDt"]: merged.pop()
        merged.append(t)
    if merged[-1] != round(T, 3): merged.append(round(T, 3))
    at = lambda t: min(s, key=lambda x: abs(x["t"] - t))
    speed = lambda x: math.hypot(x["vx"], x["vy"])
    nodes, prev_t = [], 0.0
    for i, t in enumerate(merged):
        x = at(t); final = i == len(merged) - 1
        window = [speed(y) for y in s if prev_t <= y["t"] <= t + cfg["lookahead"]]
        if cfg["capRule"] == "window":
            base = min(window[len(window)//3:] or [speed(x)])
        else:  # the solved speed where the segment ends: brake only as much as the solved path did
            base = speed(x)
        cap = min(cfg["vmax"], max(cfg["vfloor"], base * cfg["speedScale"]))
        if final: cap = cfg["finalCap"]
        markers = []
        if prev_t <= intake_t < t or (intake_t == 0 and i == 0):
            f = 0 if intake_t == 0 else (intake_t - prev_t) / max(t - prev_t, 1e-9)
            markers.append({"kind": "event", "t": round(f, 3), "key": "superstructure.setState.INTAKE"})
        if final:
            markers.append({"kind": "event", "t": 1, "key": "superstructure.setState.IDLE"})
        goal = {"x": round(x["x"], 3), "y": round(x["y"], 3), "heading": round(W(x["heading"]), 4),
                "translationTolerance": cfg["finalTolerance"] if final else 0.12, "rotationTolerance": 0.08 if final else 0.2,
                "velocityTolerance": 0.3, "settleTime": 0.04 if final else 0, "handoffRadius": 0,
                "headingMode": "fixed", "stop": final, "profiled": True}
        if markers: goal["markers"] = markers
        node = {"id": f"{rid}-pass{pi + 1}-p{i + 1}", "name": f"{name} @ {t:.2f}s", "kind": "pose", "goal": goal,
                "timeout": 5, "description": f"Sampled from the solved {name} pass at {t:.2f} s: position, heading (intake leading) and speed.",
                "constraints": {"maxVelocity": round(cap, 2)}}
        if not final: node["constraints"]["handoffRadius"] = cfg["cornerRadius"]
        nodes.append(node); prev_t = t
    return nodes
d = json.load(open(P))
for r in d["routines"]:
    passes = ROUTINES[r["id"]]
    nodes = []
    for pi, name in enumerate(passes):
        nodes += pass_nodes(r["id"], pi, name, pi == len(passes) - 1)
        nodes.append({"id": f"align-shoot-{pi}", "name": "Aim and shoot", "kind": "custom",
                      "action": "drivetrain.alignAndShoot", "timeout": 4,
                      "description": "Turn to the live shot solution, request SHOOT once within 3 degrees, and shoot for the original 3.5 second window."})
    nodes.append({"id": "done", "name": "Auto complete", "kind": "finish", "timeout": 1})
    first = load(passes[0])["waypoints"][0]
    r["start"] = {"x": first["x"], "y": first["y"], "heading": first["heading"]}
    r["nodes"] = nodes
    r.setdefault("constraints", {}).update({k: cfg[k] for k in ("translationKp", "rotationKp", "anticipateSpeedCaps") if k in cfg})
    r["initial"] = nodes[0]["id"]
    r["transitions"] = [{"id": f"t{i}", "from": a["id"], "to": b["id"], "condition": {"kind": "always"}}
                        for i, (a, b) in enumerate(zip(nodes, nodes[1:]))]
    ext = r.setdefault("extensions", {})
    ext["legacySources"] = passes
    ext["routeIntent"] = "Original solved sweep geometry: sideways through the trench, intake leading along the centerline pickup."
    ext.pop("constraintGeneration", None)
d["revision"] += 1
P.write_text(json.dumps(d, indent=2, ensure_ascii=False) + "\n")
