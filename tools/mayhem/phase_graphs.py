import json, re, sys
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
intake_edges = [edge(s, "PARKED", stop, "Intake not wanted", "Intake mode is stowed or outtaking, or the robot is disabled.")
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
shot_edges = [edge(s, "OFF", off, "Shot request ended", "Shooter mode left Shooting, or the robot is disabled.")
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
    "shootHeld": ("operator", "A held (shoot)"),
    "intakePressed": ("operator", "X pressed (intake on / off)"),
    "jitterHeld": ("operator", "Y held (jitter)"),
    "deployPressed": ("operator", "B pressed (slide out, rollers off)"),
    "outtakeHeld": ("operator", "Left trigger held (outtake)"),
    "autoShooting": ("internal", "Auto is shooting"),
    "autoIntake": ("internal", "Auto starts intaking"),
    "autoStow": ("internal", "Auto stows the intake"),
    "stopAll": ("internal", "Stop everything (auto ended, or test mode)"),
    "enabled": ("internal", "Robot enabled"),
    "inAllianceZone": ("sensor", "In alliance zone (pose)"),
    "trench": ("sensor", "Under a trench (pose)"),
    "flywheelReady": ("sensor", "Flywheel at speed"),
    "hoodClear": ("sensor", "Hood clear of the slide"),
    "slideDeployed": ("sensor", "Slide deployed"),
    "deploy": ("internal", "Intake mode wants the slide out (X / Y / B)",
               [{"machine": "IntakeMode", "key": "intakePressed"}, {"machine": "IntakeMode", "key": "jitterHeld"},
                {"machine": "IntakeMode", "key": "deployPressed"}]),
    "jitterDemand": ("internal", "Intake mode is jittering (Y)", [{"machine": "IntakeMode", "key": "jitterHeld"}]),
    "shotDemand": ("internal", "Shooter mode is shooting (A / auto)",
                   [{"machine": "ShooterMode", "key": "shootHeld"}, {"machine": "ShooterMode", "key": "autoShooting"}]),
}
STRING_INPUTS = set()
def describe_inputs(m):
    keys = used(m["transitions"])
    out = []
    for k in sorted(keys, key=lambda k: list(INPUTS).index(k) if k in INPUTS else 99):
        meta = INPUTS.get(k, ("internal", k))
        i = {"key": k, "name": meta[1], "type": "string" if k in STRING_INPUTS else "boolean",
             "initial": "" if k in STRING_INPUTS else False, "category": meta[0],
             "channel": f"/RealOutputs/Mayhem/States/Superstructure/inputs/{k}"}
        sources = list(meta[2]) if len(meta) > 2 else []
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
on = lambda key: sig(key, True)
off_ = lambda key: sig(key, False)
def mode_machine(mid, name, desc, initial, states, edges):
    for e in edges:
        if e["from"] == "*": e["id"] = f"any-to-{e['to']}"
    return {"format": "mayhem-machine", "version": 1, "id": mid, "name": name, "description": desc,
            "initial": initial, "stateChannel": f"/RealOutputs/Superstructure/{mid}",
            "states": [{"id": i, "name": n, "description": dsc, "source": SRC} for i, n, dsc in states],
            "transitions": edges}
shoot_wanted = all_(any_(on("shootHeld"), on("autoShooting")), off_("outtakeHeld"))
shooter_mode = mode_machine("ShooterMode", "Shooter mode",
    "What the shooter is doing: off, pre-spinning in the alliance zone, or shooting while A is held (or the auto shoots).",
    "OFF", [("OFF", "Off", "Flywheel off."),
            ("REV", "Rev", "In the alliance zone: the flywheel pre-spins for the hub shot."),
            ("SHOOTING", "Shooting", "Spin up, then feed (ShotCycle). Hub shot in the alliance zone, corner feed elsewhere.")], [
    edge("*", "OFF", any_(off_("enabled"), on("stopAll")), "Disabled or stop all"),
    edge("OFF", "SHOOTING", shoot_wanted, "A held / auto"),
    edge("REV", "SHOOTING", shoot_wanted, "A held / auto"),
    edge("SHOOTING", "OFF", any_(on("outtakeHeld"), all_(off_("shootHeld"), off_("autoShooting"))),
         "A released / outtaking", "Inside the alliance zone it goes straight on to Rev."),
    edge("OFF", "REV", all_(on("enabled"), on("inAllianceZone")), "In alliance zone"),
    edge("REV", "OFF", off_("inAllianceZone"), "Left alliance zone"),
])
intake_mode = mode_machine("IntakeMode", "Intake mode",
    "What the intake is doing. X toggles intaking, Y (held) jitters, B toggles the slide out with the rollers off, the left trigger (held) outtakes.",
    "STOWED", [("STOWED", "Stowed", "Slide in, rollers off."),
               ("INTAKING", "Intaking", "Slide out (IntakeSequence), rollers in."),
               ("JITTERING", "Jittering", "Slide jitters to settle pieces, rollers in. Releasing Y goes back to intaking."),
               ("DEPLOYED", "Deployed", "Slide out, rollers off."),
               ("OUTTAKING", "Outtaking", "Rollers and feeder reverse; the shooter stops.")], [
    edge("*", "STOWED", any_(off_("enabled"), on("stopAll"), on("autoStow")), "Disabled, stop all or auto stows"),
    edge("*", "OUTTAKING", on("outtakeHeld"), "Left trigger held"),
    edge("OUTTAKING", "STOWED", off_("outtakeHeld"), "Left trigger released"),
    edge("STOWED", "INTAKING", any_(on("intakePressed"), on("autoIntake")), "X / auto"),
    edge("STOWED", "JITTERING", on("jitterHeld"), "Y held"),
    edge("STOWED", "DEPLOYED", on("deployPressed"), "B"),
    edge("INTAKING", "STOWED", on("intakePressed"), "X"),
    edge("INTAKING", "JITTERING", on("jitterHeld"), "Y held"),
    edge("INTAKING", "DEPLOYED", on("deployPressed"), "B"),
    edge("JITTERING", "INTAKING", off_("jitterHeld"), "Y released"),
    edge("DEPLOYED", "INTAKING", any_(on("intakePressed"), on("autoIntake")), "X / auto"),
    edge("DEPLOYED", "JITTERING", on("jitterHeld"), "Y held"),
    edge("DEPLOYED", "STOWED", on("deployPressed"), "B"),
])
machines = d["extensions"]["machines"]
machines[:] = [m for m in machines if m["id"] not in ("Superstructure", "ShooterMode", "IntakeMode")]
machines[:0] = [shooter_mode, intake_mode]
for m in machines:
    if m["id"] in ("ShooterMode", "IntakeMode"):
        pass
    elif m["id"] == "IntakeSequence":
        m["name"] = "Intake · hood clearance and slide"
        m["states"], m["transitions"] = intake_states, intake_edges
    elif m["id"] == "ShotCycle":
        m["name"] = "Shooter · spin up and feed"
        m["states"], m["transitions"] = shot_states, shot_edges
    else:
        continue
    m["inputs"] = describe_inputs(m)
for sgn in d.get("signals", []):
    if sgn["key"] == "superstructure.ready": sgn["category"] = "sensor"
d["revision"] = d.get("revision", 0) + 1
json.dump(d, open(p, "w"), indent=2, ensure_ascii=False)
open(p, "a").write("\n")
