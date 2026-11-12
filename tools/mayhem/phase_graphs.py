import json, sys
p = sys.argv[1]
d = json.load(open(p))
SRC = "src/main/cpp/subsystems/Superstructure.cpp"
def sig(k, v): return {"kind": "signal", "key": k, "op": "eq", "value": v}
def gt(k, v): return {"kind": "signal", "key": k, "op": "gt", "value": v}
def all_(*c): return {"kind": "all", "conditions": list(c)}
def any_(*c): return {"kind": "any", "conditions": list(c)}
def edge(f, t, c, name, description=None):
    e = {"id": f"{f}-to-{t}", "from": f, "to": t, "condition": c, "name": name}
    if description: e["description"] = description
    return e
def state(i, n): return {"id": i, "name": n, "source": SRC}

intake_states = [
    state("PARKED", "Parked / no deployment demand"),
    state("CLEARING_HOOD", "Raise hood clear of the slide"),
    state("DEPLOYING", "Deploy slide to intake"),
    state("READY", "Intake deployed"),
    state("JITTERING", "Jitter slide to settle pieces"),
]
stop = any_(sig("deploy", False), sig("enabled", False))
intake_edges = [edge(s, "PARKED", stop, "Intake not wanted", "Not intaking, jittering or homing, or disabled.")
                for s in ("CLEARING_HOOD", "DEPLOYING", "READY", "JITTERING")] + [
    edge("PARKED", "CLEARING_HOOD", all_(sig("enabled", True), sig("deploy", True), sig("hoodClear", False)),
         "Slide wanted, hood in the way", "The slide is inside the hood's travel, so the hood lifts first."),
    edge("PARKED", "JITTERING", all_(sig("enabled", True), sig("deploy", True), sig("jitterDemand", True)),
         "Jitter, hood clear"),
    edge("PARKED", "DEPLOYING", all_(sig("enabled", True), sig("deploy", True)), "Slide wanted, hood clear"),
    edge("CLEARING_HOOD", "JITTERING", all_(sig("hoodClear", True), sig("jitterDemand", True)), "Hood clear: jitter"),
    edge("CLEARING_HOOD", "DEPLOYING", sig("hoodClear", True), "Hood clear: deploy"),
    edge("DEPLOYING", "JITTERING", sig("jitterDemand", True), "Jitter requested"),
    edge("DEPLOYING", "READY", sig("slideDeployed", True), "Slide out", "Both slide encoders past 0.80."),
    edge("READY", "JITTERING", sig("jitterDemand", True), "Jitter requested"),
    edge("JITTERING", "CLEARING_HOOD", all_(sig("jitterDemand", False), sig("hoodClear", False)),
         "Jitter over, hood in the way"),
    edge("JITTERING", "DEPLOYING", sig("jitterDemand", False), "Jitter over: redeploy"),
]
shot_states = [
    state("OFF", "No shot requested"),
    state("SPINNING_UP", "Spin flywheel to the requested speed"),
    state("FEEDING", "Feed; latched until the shot request ends"),
    state("INHIBITED", "Paused under the trench (hood forced down)"),
]
off = any_(sig("shotDemand", False), sig("enabled", False))
shot_edges = [edge(s, "OFF", off, "Shot request ended", "Left SHOOT / SHOOT_AND_INTAKE / JITTER_AND_SHOOT, or disabled.")
              for s in ("SPINNING_UP", "FEEDING", "INHIBITED")] + [
    edge("OFF", "SPINNING_UP", all_(sig("enabled", True), sig("shotDemand", True)), "Shoot requested"),
    edge("SPINNING_UP", "INHIBITED", sig("trench", True), "Under the trench", "The hood is forced flat there."),
    edge("FEEDING", "INHIBITED", sig("trench", True), "Under the trench", "The hood is forced flat there."),
    edge("SPINNING_UP", "FEEDING", sig("flywheelReady", True), "Flywheel at speed",
         "Feeding then stays on until the request ends; the dip as each piece goes through is ignored."),
    edge("INHIBITED", "SPINNING_UP", sig("trench", False), "Out of the trench"),
]
# Where each input comes from (shown as a badge on the graph) and its display name.
INPUTS = {
    "shoot": ("operator", "Shoot requested (A / auto)"),
    "intake": ("operator", "Intake requested (X / auto)"),
    "jitter": ("operator", "Jitter requested (Y)"),
    "home": ("operator", "Home requested (B)"),
    "outtake": ("operator", "Outtake requested (left trigger)"),
    "cancel": ("internal", "Cancel all (auto ended or failed)"),
    "rev": ("internal", "Rev wanted (in alliance zone)", ["inAllianceZone"]),
    "enabled": ("internal", "Robot enabled"),
    "poseValid": ("sensor", "Pose and shot distance valid"),
    "inAllianceZone": ("sensor", "In alliance zone (pose)"),
    "trench": ("sensor", "Under a trench (pose)"),
    "flywheelReady": ("sensor", "Flywheel at speed"),
    "hoodClear": ("sensor", "Hood clear of the slide"),
    "slideDeployed": ("sensor", "Slide deployed"),
    "deploy": ("internal", "Slide wanted", ["intake", "jitter", "home"]),
    "jitterDemand": ("internal", "Jitter wanted", ["jitter"]),
    "shotDemand": ("internal", "Shot wanted", ["shoot"]),
}
SUPER_NAMES = {
    "CANCEL_ALL": ("Cancel / lost pose", "Highest priority: cancel requested, or the pose or shot distance is invalid."),
    "HOME": ("Home", None), "OUTTAKE": ("Outtake", None),
    "JITTER_AND_SHOOT": ("Shoot + jitter", None), "SHOOT_AND_INTAKE": ("Shoot + intake", None),
    "SHOOT": ("Shoot", None), "JITTER": ("Jitter", None),
    "REV_AND_INTAKE": ("Intake in zone", "Intaking inside the alliance zone pre-spins the flywheel."),
    "INTAKE": ("Intake", None), "REV": ("Rev", "In the alliance zone with nothing else requested."),
    "IDLE": ("Nothing requested", "No request, or the robot was disabled."),
}
def describe_inputs(m):
    keys = used(m["transitions"])
    out = []
    for k in sorted(keys, key=lambda k: list(INPUTS).index(k) if k in INPUTS else 99):
        meta = INPUTS.get(k, ("internal", k))
        i = {"key": k, "name": meta[1], "type": "boolean", "initial": False, "category": meta[0],
             "channel": f"/RealOutputs/Mayhem/States/Superstructure/inputs/{k}"}
        sources = [x for x in (meta[2] if len(meta) > 2 else []) if x in keys]
        if sources: i["derivedFrom"] = sources
        out.append(i)
    return out

def used(edges):
    keys = set()
    def walk(c):
        if c["kind"] == "signal": keys.add(c["key"])
        for x in c.get("conditions", []): walk(x)
        if "condition" in c: walk(c["condition"])
    for e in edges: walk(e["condition"])
    return keys
for m in d["extensions"]["machines"]:
    if m["id"] == "IntakeSequence":
        m["name"] = "Intake · clearance and deployment"
        m["states"], m["transitions"] = intake_states, intake_edges
    elif m["id"] == "ShotCycle":
        m["name"] = "Shooting · spin up and feed"
        m["states"], m["transitions"] = shot_states, shot_edges
    elif m["id"] == "Superstructure":
        for t in m["transitions"]:
            t["name"], desc = SUPER_NAMES[t["to"]]
            if desc: t["description"] = desc
    else:
        continue
    m["inputs"] = describe_inputs(m)
for sgn in d.get("signals", []):
    if sgn["key"] == "superstructure.ready": sgn["category"] = "sensor"
d["revision"] = d.get("revision", 0) + 1
json.dump(d, open(p, "w"), indent=2, ensure_ascii=False)
open(p, "a").write("\n")
