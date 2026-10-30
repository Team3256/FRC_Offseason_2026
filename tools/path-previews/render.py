#!/usr/bin/env python3
"""Render the PR's Mayhem trajectories without running the robot or desktop app."""
import argparse
import base64
import io
import hashlib
import json
import math
import os
from pathlib import Path
import subprocess
import sys
import tempfile

from PIL import Image
import imageio_ffmpeg

ROOT = "src/main/deploy/mayhem/"
FPS = 20
SIZE = (1920, 720)


def git(*args):
    return subprocess.check_output(["git", *args])


def read_at(ref, path):
    result = subprocess.run(["git", "show", f"{ref}:{path}"], capture_output=True)
    return json.loads(result.stdout) if result.returncode == 0 else None


def solve_file(project_path, trajectory_path, output_path):
    from mayhem_solver.drivetrain import build_drivetrain
    from mayhem_solver.models import Project, Trajectory
    from mayhem_solver.pipeline import input_hash, solve

    project = Project.model_validate_json(Path(project_path).read_text())
    trajectory = Trajectory.model_validate_json(Path(trajectory_path).read_text())
    if (trajectory.output and trajectory.output.stats.success
            and trajectory.output.input_hash == input_hash(project, trajectory)):
        output = trajectory.output
        source = "saved"
    else:
        result = solve(project, trajectory, parallel=False)
        if not result.success or result.output is None:
            raise RuntimeError("Trajectory could not be solved: " + "; ".join(
                str(issue) for issue in result.issues)[:1200])
        output = result.output
        source = "regenerated"
    data = output.model_dump(by_alias=True, mode="json")
    data["_maxSpeed"] = build_drivetrain(project.robot).max_speed
    data["_source"] = source
    Path(output_path).write_text(json.dumps(data))


def prepare(project, trajectory):
    if project is None:
        raise ValueError("Missing project.mayhem")
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory)
        for name, data in [("project.json", project), ("trajectory.json", trajectory)]:
            (root / name).write_text(json.dumps(data))
        result = subprocess.run(
            [sys.executable, str(Path(__file__).resolve()), "solve",
             str(root / "project.json"), str(root / "trajectory.json"), str(root / "output.json")],
            capture_output=True, text=True, timeout=180,
            env={**os.environ, "OPENBLAS_NUM_THREADS": "1", "OMP_NUM_THREADS": "1"},
        )
        if result.returncode:
            raise ValueError(result.stderr[-1500:] or "Solver failed")
        output = json.loads((root / "output.json").read_text())
    samples = output["samples"]
    if len(samples) < 2 or len(samples) > 100000:
        raise ValueError("Expected 2 to 100000 trajectory samples")
    previous = -1
    for sample in samples:
        if not all(math.isfinite(sample[k]) for k in ("t", "x", "y", "heading", "vx", "vy")):
            raise ValueError("Non-finite trajectory sample")
        if sample["t"] < previous or sample["t"] < 0:
            raise ValueError("Trajectory times must increase")
        previous = sample["t"]
    if not 0 < previous <= 600:
        raise ValueError("Trajectory duration must be between 0 and 600 seconds")
    return {"project": project, "output": output, "maxSpeed": output.pop("_maxSpeed"),
            "source": output.pop("_source"),
            "times": [s["t"] for s in samples]}


class MayhemScene:
    """Keep Mayhem's SVG renderer alive while encoding video frames."""

    def __init__(self, name, before, after):
        self.process = subprocess.Popen(
            ["node", str(Path(__file__).with_name("scene.cjs"))],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, text=True, bufsize=1,
        )
        try:
            self.send({"name": name, "before": before, "after": after})
            if self.process.stdout.readline().strip() != "ready":
                raise RuntimeError("Could not initialize Mayhem field renderer")
        except Exception:
            self.close()
            raise

    def send(self, data):
        self.process.stdin.write(json.dumps(data) + "\n")
        self.process.stdin.flush()

    def frame(self, time, speed):
        self.send({"t": time, "speed": speed})
        encoded = self.process.stdout.readline().strip()
        if not encoded:
            raise RuntimeError("Mayhem field renderer stopped unexpectedly")
        return Image.open(io.BytesIO(base64.b64decode(encoded))).convert("RGB")

    def close(self):
        self.process.stdin.close()
        try:
            self.process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait()
        self.process.stdout.close()


def render(name, before, after, destination):
    duration = max(d["times"][-1] for d in (before, after) if d)
    speed = max(1, math.ceil(duration / 45))
    count = math.ceil(duration / speed * FPS) + 1
    scene = MayhemScene(name, before, after)
    writer = imageio_ffmpeg.write_frames(str(destination.with_suffix(".mp4")), SIZE,
                                         fps=FPS, codec="libx264", quality=7,
                                         output_params=["-movflags", "+faststart"])
    writer.send(None)
    gif_frames = []
    try:
        for i in range(count + FPS):
            image = scene.frame(min(duration, i / FPS * speed), speed)
            writer.send(image.tobytes())
            if i % 2 == 0:
                gif_frames.append(image.resize((1280, 480)).quantize(colors=128))
        scene.frame(duration / 2, speed).save(destination.with_suffix(".png"))
    finally:
        writer.close()
        scene.close()
    gif_frames[0].save(destination.with_suffix(".gif"), save_all=True,
                       append_images=gif_frames[1:], duration=100, loop=0, optimize=False)


def preview(args):
    base = git("merge-base", args.base, args.head).decode().strip()
    # Treat renames as delete + add so both filenames are represented in review.
    changed = set(filter(None, git("diff", "--no-renames", "--name-only", "-z", base, args.head, "--", ROOT).decode().split("\0")))
    old_project = read_at(base, ROOT + "project.mayhem")
    new_project = read_at(args.head, ROOT + "project.mayhem")
    if ROOT + "project.mayhem" in changed:
        # Robot, field and pose-variable changes can affect unchanged paths too.
        for ref in (base, args.head):
            changed.update(filter(None, git("ls-tree", "-r", "--name-only", "-z", ref, "--", ROOT).decode().split("\0")))
    paths = sorted(p for p in changed if p.endswith(".mtraj"))
    destination = Path(args.output)
    destination.mkdir(parents=True, exist_ok=True)
    report = {"prNumber": getattr(args, "pr_number", None), "base": base, "head": git("rev-parse", args.head).decode().strip(), "paths": []}
    for path in paths:
        print(f"Previewing {path}", flush=True)
        entry = {"path": path, "name": Path(path).stem}
        old, new = read_at(base, path), read_at(args.head, path)
        if new is None:
            entry["status"] = "deleted"
            report["paths"].append(entry)
            continue
        before = after = None
        for side, project, trajectory in [("before", old_project, old), ("after", new_project, new)]:
            if trajectory is None:
                continue
            try:
                data = prepare(project, trajectory)
                if side == "before":
                    before = data
                else:
                    after = data
            except Exception as error:
                entry[side + "Error"] = str(error)[-1800:]
                print(f"{path} ({side}): {error}", file=sys.stderr, flush=True)
        if after:
            stem = hashlib.sha256(path.encode()).hexdigest()[:16]
            try:
                render(entry["name"], before, after, destination / stem)
                entry.update(status="changed" if old else "added", media=stem,
                             afterSource=after.get("source", "saved"),
                             afterDuration=after["times"][-1])
                if before:
                    entry["beforeDuration"] = before["times"][-1]
                    entry["beforeSource"] = before.get("source", "saved")
            except Exception as error:
                entry.update(status="failed", afterError=str(error)[-1800:])
                print(f"{path} (render): {error}", file=sys.stderr, flush=True)
        else:
            entry["status"] = "failed"
        report["paths"].append(entry)
        (destination / "manifest.json").write_text(json.dumps(report, indent=2))
    (destination / "manifest.json").write_text(json.dumps(report, indent=2))
    if any(p["status"] == "failed" for p in report["paths"]):
        raise SystemExit(1)


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "solve":
        solve_file(*sys.argv[2:])
    else:
        parser = argparse.ArgumentParser(description=__doc__)
        parser.add_argument("--base", required=True)
        parser.add_argument("--head", default="HEAD")
        parser.add_argument("--pr-number", type=int)
        parser.add_argument("--output", default="path-previews")
        preview(parser.parse_args())
