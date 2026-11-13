"""Rebuild robot.mayhem.json routines from the original solved .mtraj passes.

Each original waypoint (start, corners, pickup, shot spot) becomes a pose step whose heading is the
solved heading there; the solved passes are a reference for the route, not a path to trace exactly.
rotation markers inside each segment carry the solved heading profile, so the intake leads where it
did in the original paths. Segment speed caps come from the solved speed profile.
usage: routes.py <robot repo> [json overrides]
"""
import json, math, sys
from pathlib import Path
repo = Path(sys.argv[1]); P = repo / "robot.mayhem.json"
cfg = dict(speedScale=1.0, cornerRadius=0.3, spacing=None, pathTolerance=0.15, headingTolerance=10, minDt=0.15, lookahead=0.2, vfloor=0.8, vmax=4.8, finalCap=3.0, translationKp=4.5, capRule="max", finalTolerance=0.15)
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
# Waypoint headings (deg) set by hand on generated passes, by point number: the follower's heading
# lags the blend, so these lead the turn and keep the intake ahead through the centre-line pickup.
HEADINGS = {"DepotTrenchSweepBumpPt1": {2: -60, 3: -75, 6: -175, 7: 130, 8: 95, 9: 90}}
# Hand-placed second passes of the x2 autos (blue): (x, y, heading deg, max speed, options), where
# "rotate": t reaches the heading by fraction t of the segment and "intake": t fires the intake there.
# They turn square near the shot spot, at least 0.7 m from the wall and the trench post, then drive
# the trench opening's centre line squared; the solved pass is only the general direction.
MANUAL = {
    ("outpost-sweep-x2", 1): [
        (3.00, 1.35, 90, 3.0, {"rotate": 0.5}),  # square by here, clear of the wall and the post
        (3.75, 0.70, 90, 2.2, {}),  # trench centre line
        (6.27, 0.71, 90, 4.3, {}),
        (7.21, 1.30, 90, 4.0, {"intake": 0}),
        (7.63, 2.55, 107, 3.55, {}),
        (7.17, 3.82, 156, 3.0, {}),
        (6.60, 4.10, -140, 2.4, {}),
        (6.42, 3.45, -95, 3.0, {}),
        (6.00, 2.50, -90, 3.0, {}),  # square onto the bump
        (3.10, 2.51, -90, 3.5, {}),
    ],
    ("depot-sweep-x2", 1): [
        (3.00, 6.72, 0, 3.0, {"rotate": 0.5, "intake": 0}),  # square by here, clear of the wall and the post
        (3.75, 7.37, 0, 2.2, {}),  # trench centre line
        (5.80, 7.43, 0, 4.2, {}),
        (6.60, 7.10, -45, 4.1, {}),
        (7.22, 6.34, -90, 3.8, {}),
        (7.44, 5.40, -133, 3.9, {}),
        (7.27, 4.33, -164, 4.3, {}),
        (6.85, 3.40, 142, 2.8, {}),
        (6.10, 3.75, 100, 3.0, {}),
        (6.00, 5.10, 90, 3.0, {}),  # square onto the bump, well below the trench post
        (2.84, 5.63, 90, 3.5, {}),
    ],
}
def load(name): return json.load(open(repo / f"tools/mayhem/solved/{name}.mtraj"))
def heading_at(samples, t):
    best = min(samples, key=lambda s: abs(s["t"] - t)); return W(best["heading"])
def node(rid, pi, i, x, y, heading, cap, final, markers, description):
    goal = {"x": round(x, 3), "y": round(y, 3), "heading": round(W(heading), 4),
            "translationTolerance": cfg["finalTolerance"] if final else 0.12, "rotationTolerance": 0.08 if final else 0.2,
            "velocityTolerance": 0.3, "settleTime": 0.04 if final else 0, "handoffRadius": 0,
            "headingMode": "fixed", "stop": final, "profiled": True}
    if markers: goal["markers"] = markers
    label = "end" if final else f"point {i + 1}"
    n = {"id": f"{rid}-pass{pi + 1}-p{i + 1}", "name": f"Pass {pi + 1} · {label}", "kind": "pose", "goal": goal,
         "timeout": 5, "description": description, "constraints": {"maxVelocity": round(cap, 2)}}
    if not final: n["constraints"]["handoffRadius"] = cfg["cornerRadius"]
    return n
def manual_nodes(rid, pi, name):
    pts = MANUAL[(rid, pi)]; nodes = []
    for i, (x, y, h, cap, opt) in enumerate(pts):
        final = i == len(pts) - 1
        markers = [{"kind": "rotation", "t": opt["rotate"], "heading": round(math.radians(h), 4)}] if "rotate" in opt else []
        if "intake" in opt: markers.append({"kind": "event", "t": opt["intake"], "key": "superstructure.intake"})
        if final: markers.append({"kind": "event", "t": 1, "key": "superstructure.stow"})
        markers.sort(key=lambda m: m["t"])
        nodes.append(node(rid, pi, i, x, y, math.radians(h), cap, final, markers,
                          f"Hand-placed waypoint, after the solved {name} pass: turn square clear of the wall and post, then the trench centre line."))
    return nodes
def pass_nodes(rid, pi, name, last_pass):
    if (rid, pi) in MANUAL: return manual_nodes(rid, pi, name)
    d = load(name); s = d["output"]["samples"]; wt = d["output"]["waypointTimes"]
    events = {e["name"]: e["t"] for e in d["output"]["events"]}
    intake_t = next((events[k] for k in events if k in INTAKE), 0.0)
    T = s[-1]["t"]
    # Each original waypoint time, plus the fewest points that keep the driven line within
    # `pathTolerance` m and the heading within `headingTolerance` deg of the solved pass (straight runs
    # need none), plus (if `spacing` is set) a point every `spacing` meters of arc.
    keep, arc, last = [], 0.0, 0.0
    def off(a, b, k):
        # How far sample k is from the straight line a-b, and from the heading interpolated along it.
        ax, ay, bx, by = s[a]["x"], s[a]["y"], s[b]["x"], s[b]["y"]
        L = math.hypot(bx - ax, by - ay)
        u = ((s[k]["x"] - ax) * (bx - ax) + (s[k]["y"] - ay) * (by - ay)) / (L * L) if L > 1e-9 else 0
        u = min(1, max(0, u))
        dist = math.hypot(s[k]["x"] - ax - u * (bx - ax), s[k]["y"] - ay - u * (by - ay))
        h = s[a]["heading"] + u * W(s[b]["heading"] - s[a]["heading"])
        return dist, abs(math.degrees(W(s[k]["heading"] - h)))
    def simplify(a, b):
        if b - a < 2: return
        errs = [(max(d / cfg["pathTolerance"], h / cfg["headingTolerance"]), k)
                for k in range(a + 1, b) for d, h in [off(a, b, k)]]
        worst, k = max(errs)
        if worst > 1:
            keep.append(s[k]["t"]); simplify(a, k); simplify(k, b)
    fixed = sorted({0, len(s) - 1} | {min(range(len(s)), key=lambda i: abs(s[i]["t"] - t)) for t in wt})
    for a, b in zip(fixed, fixed[1:]): simplify(a, b)
    for a, b in zip(s, s[1:]):
        arc += math.hypot(b["x"] - a["x"], b["y"] - a["y"])
        if cfg["spacing"] and arc - last >= cfg["spacing"]: keep.append(b["t"]); last = arc
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
        elif cfg["capRule"] == "max":  # the fastest the solved pass went on this segment
            base = max([speed(y) for y in s if prev_t < y["t"] <= t] or [speed(x)])
        else:  # the solved speed where the segment ends: brake only as much as the solved path did
            base = speed(x)
        cap = min(cfg["vmax"], max(cfg["vfloor"], base * cfg["speedScale"]))
        if final: cap = cfg["finalCap"]
        markers = []
        if prev_t <= intake_t < t or (intake_t == 0 and i == 0):
            f = 0 if intake_t == 0 else (intake_t - prev_t) / max(t - prev_t, 1e-9)
            markers.append({"kind": "event", "t": round(f, 3), "key": "superstructure.intake"})
        if final:
            markers.append({"kind": "event", "t": 1, "key": "superstructure.stow"})
        heading = W(x["heading"])
        if i + 1 in HEADINGS.get(name, {}): heading = math.radians(HEADINGS[name][i + 1])
        nodes.append(node(rid, pi, i, x["x"], x["y"], heading, cap, final, markers,
                          f"Waypoint from the solved {name} pass at {t:.2f} s: position, heading (intake leading) and speed."))
        prev_t = t
    return nodes
d = json.load(open(P))
for r in d["routines"]:
    if r["id"] not in ROUTINES: continue  # drawn in Mayhem, not generated
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
