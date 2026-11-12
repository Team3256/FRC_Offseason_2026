import json, sys
p = sys.argv[1]
d = json.load(open(p))
SRC = "src/main/cpp/subsystems/Superstructure.cpp"
def sig(k, v): return {"kind": "signal", "key": k, "op": "eq", "value": v}
def gt(k, v): return {"kind": "signal", "key": k, "op": "gt", "value": v}
def all_(*c): return {"kind": "all", "conditions": list(c)}
def any_(*c): return {"kind": "any", "conditions": list(c)}
def edge(f, t, c): return {"id": f"{f}-to-{t}", "from": f, "to": t, "condition": c}
def state(i, n): return {"id": i, "name": n, "source": SRC}

intake_states = [
    state("PARKED", "Parked / no deployment demand"),
    state("CLEARING_HOOD", "Raise hood clear of the slide"),
    state("DEPLOYING", "Deploy slide to intake"),
    state("READY", "Intake deployed"),
    state("JITTERING", "Jitter slide to settle pieces"),
]
stop = any_(sig("deploy", False), sig("enabled", False))
intake_edges = [edge(s, "PARKED", stop) for s in ("CLEARING_HOOD", "DEPLOYING", "READY", "JITTERING")] + [
    edge("PARKED", "CLEARING_HOOD", all_(sig("enabled", True), sig("deploy", True), sig("hoodClear", False))),
    edge("PARKED", "JITTERING", all_(sig("enabled", True), sig("deploy", True), sig("jitterDemand", True))),
    edge("PARKED", "DEPLOYING", all_(sig("enabled", True), sig("deploy", True))),
    edge("CLEARING_HOOD", "JITTERING", all_(sig("hoodClear", True), sig("jitterDemand", True))),
    edge("CLEARING_HOOD", "DEPLOYING", sig("hoodClear", True)),
    edge("DEPLOYING", "JITTERING", sig("jitterDemand", True)),
    edge("DEPLOYING", "READY", sig("slideDeployed", True)),
    edge("READY", "JITTERING", sig("jitterDemand", True)),
    edge("JITTERING", "CLEARING_HOOD", all_(sig("jitterDemand", False), sig("hoodClear", False))),
    edge("JITTERING", "DEPLOYING", sig("jitterDemand", False)),
]
shot_states = [
    state("OFF", "No shot requested"),
    state("SPINNING_UP", "Spin flywheel to the requested speed"),
    state("FEEDING", "Feed; latched until the shot request ends"),
    state("INHIBITED", "Paused under the trench (hood forced down)"),
]
off = any_(sig("shotDemand", False), sig("enabled", False))
shot_edges = [edge(s, "OFF", off) for s in ("SPINNING_UP", "FEEDING", "INHIBITED")] + [
    edge("OFF", "SPINNING_UP", all_(sig("enabled", True), sig("shotDemand", True))),
    edge("SPINNING_UP", "INHIBITED", sig("trench", True)),
    edge("FEEDING", "INHIBITED", sig("trench", True)),
    edge("SPINNING_UP", "FEEDING", sig("flywheelReady", True)),
    edge("INHIBITED", "SPINNING_UP", sig("trench", False)),
]
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
    else:
        continue
    keys = used(m["transitions"])
    m["inputs"] = [i for i in m["inputs"] if i["key"] in keys]
    missing = keys - {i["key"] for i in m["inputs"]}
    for k in sorted(missing):
        m["inputs"].append({"key": k, "name": k, "type": "boolean", "initial": False,
                            "channel": f"/RealOutputs/Mayhem/States/Superstructure/inputs/{k}"})
d["revision"] = d.get("revision", 0) + 1
json.dump(d, open(p, "w"), indent=2, ensure_ascii=False)
open(p, "a").write("\n")
