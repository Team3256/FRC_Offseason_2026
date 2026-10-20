"""Recreate the comp-bot competition paths from a pinned Team 581 checkout.

Usage: python tools/import_team581_comp_paths.py /path/to/team581-frc-2026
Then generate the imported paths with Mayhem. No robot chooser changes are made.
"""

import ast
import json
import math
from pathlib import Path
import re
import subprocess
import sys

SOURCE_COMMIT = "5037ac428146c253d1648bcde6d95bbf15d0d2ef"
ROOT = Path(__file__).resolve().parents[1]
DEPLOY = ROOT / "src/main/deploy/mayhem"
TOP = "581 - Top - Depot"
BOTTOM = "581 - Bottom - HP Station"


def closing(text, opening):
    depth = 0
    for i in range(opening, len(text)):
        if text[i] == "(":
            depth += 1
        elif text[i] == ")":
            depth -= 1
            if depth == 0:
                return i
    raise ValueError("Unbalanced Java expression")


def arguments(text):
    depth = 0
    start = 0
    result = []
    for i, char in enumerate(text):
        if char in "({[":
            depth += 1
        elif char in ")}]":
            depth -= 1
        elif char == "," and depth == 0:
            result.append(text[start:i].strip())
            start = i + 1
    result.append(text[start:].strip())
    return result


def number(expression, constants):
    expression = expression.strip()
    if expression in constants:
        return constants[expression]
    match = re.fullmatch(r"Units\.(inchesToMeters|rotationsToRadians)\((.*)\)", expression, re.S)
    if match:
        return number(match[2], constants) * (0.0254 if match[1] == "inchesToMeters" else 2 * math.pi)
    for name, value in constants.items():
        if "." in name:
            expression = expression.replace(name, repr(value))
    # Evaluate only arithmetic, numeric literals, and named numeric constants.
    tree = ast.parse(expression, mode="eval")

    def visit(node):
        if isinstance(node, ast.Constant) and type(node.value) in (int, float):
            return node.value
        if isinstance(node, ast.Name) and node.id in constants:
            return constants[node.id]
        if isinstance(node, ast.UnaryOp) and isinstance(node.op, (ast.UAdd, ast.USub)):
            return visit(node.operand) * (-1 if isinstance(node.op, ast.USub) else 1)
        if isinstance(node, ast.BinOp):
            a, b = visit(node.left), visit(node.right)
            for kind, operation in [(ast.Add, lambda: a + b), (ast.Sub, lambda: a - b),
                                    (ast.Mult, lambda: a * b), (ast.Div, lambda: a / b)]:
                if isinstance(node.op, kind):
                    return operation()
        raise ValueError(f"Unsupported numeric expression: {expression}")

    return float(visit(tree.body))


def heading(text, constants):
    angles = {"Rotation2d.kZero": 0, "Rotation2d.k180deg": math.pi,
              "Rotation2d.kCCW_90deg": math.pi / 2, "Rotation2d.kCW_90deg": -math.pi / 2}
    if text in angles:
        return angles[text]
    match = re.fullmatch(r"Rotation2d.fromDegrees\((.*)\)", text, re.S)
    if match:
        return math.radians(number(match[1], constants))
    raise ValueError(f"Unsupported heading: {text}")


def pose(text, constants):
    matches = list(re.finditer(r"new Pose2d\(", text))
    # Suppliers may first request a sensor-controlled bump crossing. Only their
    # returned pose is a path target; it is the last Pose2d in these pinned sources.
    match = matches[-1]
    opening = match.end() - 1
    x, y, angle = arguments(text[opening + 1:closing(text, opening)])
    return [number(x, constants), number(y, constants), heading(angle, constants)]


def limits(text, constants):
    result = {}
    for method, names in [("Linear", ("maxVelocity", "maxAcceleration")),
                          ("Angular", ("maxAngularVelocity", None))]:
        match = re.search(r"\.with" + method + r"Constraints\(", text)
        if match:
            opening = match.end() - 1
            values = arguments(text[opening + 1:closing(text, opening)])
            for name, value in zip(names, values):
                if name:
                    result[name] = number(value, constants)
    return result


def load_auto(source, constants):
    text = source.read_text()
    constants = dict(constants)
    for name, value in re.findall(r"private static final double (\w+) = ([^;]+);", text):
        constants[name] = number(value, constants)
    start = pose(text[text.index("public Point getStartingPoint()"):].split("\n  }", 1)[0], constants)
    segments = {}
    for match in re.finditer(r"private final AutoSegment (\w+)\s*=\s*Trailblazer.segment\(", text):
        opening = match.end() - 1
        end = closing(text, opening)
        suffix = text[end + 1:text.index(";", end)]
        points = []
        for point in arguments(text[opening + 1:end]):
            marker = re.search(r"\.withMarker\(Markers\.(\w+)\)", point)
            transition = re.search(r"withTransitionTolerance\(\s*new PoseErrorTolerance\(([^)]*)\)", point)
            tolerance = [number(v, constants) for v in arguments(transition[1])] if transition else None
            points.append({"pose": pose(point, constants), "limits": limits(point, constants),
                           "marker": marker[1] if marker else None, "transitionTolerance": tolerance})
        segments[match[1]] = {"points": points, "limits": limits(suffix, constants)}
    return start, segments


def build(name, points, global_limits, folder, project):
    length, width = 16.541, 8.069
    waypoints, constraints, markers = [], [], []

    def cap(kind, value, first, last):
        constraints.append({"id": f"{kind}-{first}-{last}", "enabled": True,
                            "scope": {"kind": "range", "from": first, "to": last},
                            "data": {"type": kind, "value": value}})

    for i, point in enumerate(points):
        x, y, angle = point["pose"]
        angle += math.pi
        waypoint = {"id": f"w{i}", "x": length - x, "y": width - y,
                    "heading": math.atan2(math.sin(angle), math.cos(angle)),
                    "translationMode": "fixed", "headingMode": "fixed",
                    "headingTolerance": 0, "tolerance": {"kind": "none", "radius": 0.1, "dx": 0.1, "dy": 0.1},
                    "stop": i in (0, len(points) - 1), "split": False, "intervals": None, "poseRef": None}
        waypoints.append(waypoint)
        if i and point["pose"][:2] == points[i - 1]["pose"][:2]:
            # Consecutive equal translations encode a turn in place.
            waypoints[i - 1]["stop"] = True
            waypoint["stop"] = True
        for kind, value in point["limits"].items():
            if i and value != global_limits.get(kind):
                cap(kind, value, i - 1, i)
        if point["marker"]:
            markers.append({"id": f"marker-{i}", "name": point["marker"], "command": "",
                            "waypoint": i, "offset": 0, "endWaypoint": None, "endOffset": 0,
                            "recoveryPolicy": "fireAtJoin", "mustHit": False})
    for kind, value in global_limits.items():
        cap(kind, value, 0, len(points) - 1)
    for obstacle in project["field"]["obstacles"]:
        if obstacle["id"].startswith("bump-"):
            constraints.append({"id": "terrain-" + obstacle["id"], "enabled": True,
                                "scope": {"kind": "zone", "region": obstacle["points"]},
                                "data": {"type": "roughTerrain", "expectedSpeed": 0.7, "feedbackScale": 0.3}})
    trajectory = {"formatVersion": 1, "name": name, "waypoints": waypoints,
                  "constraints": constraints, "markers": markers,
                  "settings": {"targetDt": 0.08, "smoothing": 0.02, "candidates": 3,
                               "maxIterations": 1500, "timeLimit": 60},
                  "folder": folder, "output": None}
    path = DEPLOY / (name + ".mtraj")
    legacy = DEPLOY / ("team581Comp" + name + ".mtraj")
    if legacy.exists() and not path.exists():
        legacy.rename(path)
    if path.exists():
        old = json.loads(path.read_text())
        if all(old.get(key) == value for key, value in trajectory.items() if key not in ("folder", "output")):
            trajectory["output"] = old.get("output")
    path.write_text(json.dumps(trajectory, indent=1) + "\n")
    return {"name": name, "folder": folder, "redPoses": [p["pose"] for p in points]}


def main():
    checkout = Path(sys.argv[1]).resolve()
    commit = subprocess.check_output(["git", "-C", str(checkout), "rev-parse", "HEAD"], text=True).strip()
    if commit != SOURCE_COMMIT:
        raise ValueError(f"Expected pinned source {SOURCE_COMMIT}; got {commit}")
    project_path = DEPLOY / "project.mayhem"
    project = json.loads(project_path.read_text())
    layout = json.loads((ROOT / "src/main/deploy/apriltags/welded/2026-official.json").read_text())
    assert layout["field"] == {"length": 16.541, "width": 8.069}
    tags = {t["ID"]: t["pose"]["translation"] for t in layout["tags"]}
    bump = (48.94 + 12 + 73 / 2) * 0.0254
    constants = {"FieldUtil.RED_DEPOT_TRENCH_CENTER.getY()": tags[17]["y"],
                 "FieldUtil.RED_OUTPOST_TRENCH_CENTER.getY()": tags[22]["y"],
                 "FieldUtil.RED_DEPOT_BUMP_CENTER.getY()": bump,
                 "FieldUtil.RED_OUTPOST_BUMP_CENTER.getY()": 8.069 - bump,
                 "FieldUtil.RED_HUB_CENTER.getY()": 8.069 / 2}
    # Replace known FieldUtil numeric expressions before parsing arithmetic.
    # load_auto also evaluates static Java constants such as BUMP_OFFSET.
    source_dir = checkout / "comp-bot/src/main/java/frc/robot/autos/auto_state_machines"
    manifest = []
    for side, folder in [("Left", TOP), ("Right", BOTTOM)]:
        for routine in ("Normal", "Special"):
            source = source_dir / f"{side}{routine}Auto.java"
            start, segments = load_auto(source, constants)
            def emit(suffix, points, base, source_segments):
                entry = build(f"{side}{routine}{suffix}", points, base, folder, project)
                entry.update(source=str(source.relative_to(checkout)), segments=source_segments)
                manifest.append(entry)
            def anchor(value):
                return {"pose": value, "limits": {}, "marker": None}
            if routine == "Normal":
                first = segments["intakeFirstCycle"]
                second = segments["defaultIntakeSecondCycle"]
                third = segments["intakeThirdCycle"]
                far = segments["intakeSecondCycleFar"]
                shot = first["points"][-1]["pose"]
                emit("FirstCycle", [anchor(start)] + first["points"], first["limits"], ["intakeFirstCycle"])
                emit("SecondCycle", [anchor(shot)] + second["points"], second["limits"], ["defaultIntakeSecondCycle"])
                # Branch starts when MAKE_CLUSTER_MAP_DECISION becomes the active
                # target. Use the preceding anchor as the nominal switch location.
                decision = next(i for i, p in enumerate(second["points"]) if p["marker"] == "MAKE_CLUSTER_MAP_DECISION")
                prefix = second["points"][:decision]
                emit("SecondCycleFar", [anchor(shot)] + prefix + far["points"], second["limits"],
                     ["defaultIntakeSecondCycle (prefix)", "intakeSecondCycleFar"])
                emit("ThirdCycle", [anchor(shot)] + third["points"], third["limits"], ["intakeThirdCycle"])
            else:
                intake = segments["intakeAcrossMidline"]
                back = segments["driveBackAndShootOne"]
                second = segments["defaultSecondSegment"]
                emit("IntakeAcrossMidline", [anchor(start)] + intake["points"], intake["limits"], ["intakeAcrossMidline"])
                emit("ReturnAndShoot", [anchor(intake["points"][-1]["pose"])] + back["points"], back["limits"], ["driveBackAndShootOne"])
                emit("SecondIntake", [anchor(back["points"][-1]["pose"])] + second["points"], second["limits"], ["defaultSecondSegment"])
    project["folders"] = [f for f in project.get("folders", []) if f not in ("581 Comp Bot Examples", TOP, BOTTOM)] + [TOP, BOTTOM]
    project_path.write_text(json.dumps(project, indent=2) + "\n")
    (ROOT / "docs/team581-comp-path-manifest.json").write_text(json.dumps({"sourceCommit": commit, "paths": manifest}, indent=2) + "\n")
    print(f"Imported {len(manifest)} paths into {TOP!r} and {BOTTOM!r}")


if __name__ == "__main__":
    main()
