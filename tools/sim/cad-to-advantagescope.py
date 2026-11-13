# /// script
# requires-python = ">=3.10"
# dependencies = ["numpy>=1.26", "fast-simplification>=0.1.7", "pillow>=10"]
# ///
# Vendored from Mayhem v0.9.28 (scripts/sim/cad-to-advantagescope.py, github.com/ryanabraham1/Mayhem); update it there.
"""Turn a CAD GLB (e.g. an Onshape export) into an AdvantageScope custom robot asset.

    uv run scripts/sim/cad-to-advantagescope.py robot.glb config.json --out ~/advantagescope-assets

Run with --list first to print every part's node path (to write the regexes).

Writes <out>/Robot_<Name>/ with config.json, model.glb (the base) and model_<i>.glb per
articulated component, decimated to a total size budget, colours preserved, coordinates in the
robot frame (x forward, y left, z up, origin at the frame centre on the floor). Next to it:
Robot_<Name>.preview.png (side and top orthographic views, coloured by component) and
Robot_<Name>.joints.json (each component's joint, for robot code).

Config (JSON):
{
  "name": "3256 2026",
  "frame": {
    "forward": "+x",            # CAD axis that points robot-forward  (+x -x +y -y +z -z)
    "up": "+y",                 # CAD axis that points up (Onshape GLB exports are Y-up)
    "origin": "auto",           # CAD point (metres) that becomes the robot origin, or "auto":
    "originMatch": ["(?i)drive|swerve|frame|bumper"],  #  auto: x-y centre of these parts' box
    "floor": "auto"             # CAD height (along up) of the floor, or "auto": lowest point
  },
  "units": 1.0,                 # CAD units -> metres (Onshape GLB is already metres)
  "drop": ["(?i)screw|bolt|nut|washer|bearing|spacer|rivet|standoff"],
  "dropSmallerThan": 0.0,       # drop parts whose bounding-box diagonal is below this (m)
  "dropBelowFloor": -0.005,     # drop part instances whose top is below this height (m, robot frame)
  "dropOutside": {"min": [-0.6, -0.6, -0.05], "max": [0.6, 0.6, 1.5]},
                                # drop part instances whose box centre is outside this robot-frame
                                # box (strays); parts matched by a component are exempt
  "targetMB": 15,             # total budget for all .glb files; met by uniform vertex clustering
  "quadric": false,             # also run quadric decimation first (sharper, but can fold thin parts)
  "clip": {"maxZ": 1.2},        # optional: clamp the base model's vertices to this robot-frame z range
  "components": [
    {"name": "intake", "match": ["(?i)intake"],
     "joint": {"type": "revolute", "axis": [0, 1, 0], "origin": [0.30, 0, 0.20]},
     "clip": {"maxZ": 0.555}},  # optional: clamp this component's vertices (e.g. a part cut down after the CAD)
    {"name": "elevator", "match": ["(?i)carriage"],
     "joint": {"type": "prismatic", "axis": [0, 0, 1]}}
  ]
}

Run --list to see every part instance (node index, path, robot-frame box centre and z range,
whether it is dropped) to pick regexes and the dropOutside / dropBelowFloor limits. Regexes
match a part's full node path ("00.Main/0200_Intake_Assembly/occurrence of Roller/...")
so a subassembly name selects everything under it; the first matching component wins and
unmatched parts go into the base model. Joint axes and origins are in the robot frame (metres).

AdvantageScope applies a component's zeroedRotations/zeroedPosition to bring it to the robot
origin, then the logged pose. The meshes are written in the robot frame and zeroedPosition is
-origin, so robot code logs, per component in config order:
  revolute:  Pose3d(origin, Rotation3d(axis * angle))
  prismatic: Pose3d(origin + axis * distance, Rotation3d())   (origin defaults to 0)
and at zero joint position the component sits exactly where it is in CAD.
"""
from __future__ import annotations

import argparse
import json
import math
import re
import struct
from collections import Counter
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np

COMPONENT_TYPES = {5120: np.int8, 5121: np.uint8, 5122: np.int16, 5123: np.uint16, 5125: np.uint32, 5126: np.float32}
TYPE_SIZES = {"SCALAR": 1, "VEC2": 2, "VEC3": 3, "VEC4": 4, "MAT4": 16}
# Bytes per decimated triangle in the output: flat-shaded, unwelded (3 x (pos + normal)) + indices.
BYTES_PER_FACE = 3 * 24 + 12


# ----------------------------------------------------------------------------------------------
# GLB reading
def read_glb(path: Path) -> tuple[dict, memoryview]:
    data = path.read_bytes()
    magic, version, length = struct.unpack_from("<III", data, 0)
    if magic != 0x46546C67:
        sys.exit(f"{path} is not a binary glTF (.glb)")
    offset, gltf, binary = 12, None, memoryview(b"")
    while offset < length:
        chunk_len, chunk_type = struct.unpack_from("<II", data, offset)
        body = memoryview(data)[offset + 8 : offset + 8 + chunk_len]
        if chunk_type == 0x4E4F534A:
            gltf = json.loads(bytes(body))
        elif chunk_type == 0x004E4942:
            binary = body
        offset += 8 + chunk_len
    if gltf is None:
        sys.exit(f"{path} has no JSON chunk")
    return gltf, binary


def accessor_array(gltf: dict, binary: memoryview, index: int) -> np.ndarray:
    acc = gltf["accessors"][index]
    dtype = np.dtype(COMPONENT_TYPES[acc["componentType"]])
    width = TYPE_SIZES[acc["type"]]
    count = acc["count"]
    if "bufferView" not in acc:
        return np.zeros((count, width), dtype=dtype)
    view = gltf["bufferViews"][acc["bufferView"]]
    start = view.get("byteOffset", 0) + acc.get("byteOffset", 0)
    stride = view.get("byteStride", 0) or dtype.itemsize * width
    if stride == dtype.itemsize * width:
        arr = np.frombuffer(binary, dtype=dtype, count=count * width, offset=start).reshape(count, width)
    else:
        raw = np.frombuffer(binary, dtype=np.uint8, count=stride * (count - 1) + dtype.itemsize * width, offset=start)
        arr = np.lib.stride_tricks.as_strided(raw, shape=(count, dtype.itemsize * width), strides=(stride, 1))
        arr = np.ascontiguousarray(arr).view(dtype).reshape(count, width)
    if acc.get("normalized"):
        arr = arr.astype(np.float32) / np.iinfo(dtype).max
    return arr


def node_matrix(node: dict) -> np.ndarray:
    if "matrix" in node:
        return np.array(node["matrix"], dtype=np.float64).reshape(4, 4).T
    t = np.array(node.get("translation", [0, 0, 0]), dtype=np.float64)
    x, y, z, w = node.get("rotation", [0, 0, 0, 1])
    s = np.array(node.get("scale", [1, 1, 1]), dtype=np.float64)
    r = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    m = np.eye(4)
    m[:3, :3] = r * s
    m[:3, 3] = t
    return m


@dataclass
class Part:
    path: str
    vertices: np.ndarray  # (n, 3) float64, CAD frame
    faces: np.ndarray     # (m, 3) int64
    color: tuple          # RGBA 0..1
    instance: int = -1    # glTF node index: one node (instance) may be split into several colours


def load_parts(path: Path) -> list[Part]:
    gltf, binary = read_glb(path)
    nodes = gltf.get("nodes", [])
    materials = gltf.get("materials", [])
    mesh_cache: dict[int, list[tuple[np.ndarray, np.ndarray, tuple]]] = {}

    def mesh_primitives(mi: int):
        if mi in mesh_cache:
            return mesh_cache[mi]
        out = []
        for prim in gltf["meshes"][mi]["primitives"]:
            if prim.get("mode", 4) != 4 or "POSITION" not in prim["attributes"]:
                continue
            pos = accessor_array(gltf, binary, prim["attributes"]["POSITION"]).astype(np.float64)
            if "indices" in prim:
                idx = accessor_array(gltf, binary, prim["indices"]).reshape(-1).astype(np.int64)
            else:
                idx = np.arange(len(pos), dtype=np.int64)
            color = (0.7, 0.7, 0.7, 1.0)
            if "material" in prim:
                mat = materials[prim["material"]]
                color = tuple(mat.get("pbrMetallicRoughness", {}).get("baseColorFactor", color))
            out.append((pos, idx.reshape(-1, 3), color))
        mesh_cache[mi] = out
        return out

    parts: list[Part] = []
    scene = gltf.get("scenes", [{}])[gltf.get("scene", 0)]
    stack = [(n, np.eye(4), "") for n in reversed(scene.get("nodes", []))]
    while stack:
        index, parent, prefix = stack.pop()
        node = nodes[index]
        name = node.get("name", "") or f"node{index}"
        path_name = f"{prefix}/{name}" if prefix else name
        world = parent @ node_matrix(node)
        if "mesh" in node:
            # Merge this node's primitives per colour.
            by_color: dict[tuple, list] = {}
            for pos, faces, color in mesh_primitives(node["mesh"]):
                by_color.setdefault(color, []).append((pos, faces))
            for color, chunks in by_color.items():
                verts, tris, base = [], [], 0
                for pos, faces in chunks:
                    verts.append(pos)
                    tris.append(faces + base)
                    base += len(pos)
                v = np.concatenate(verts)
                v = v @ world[:3, :3].T + world[:3, 3]
                parts.append(Part(path_name, v, np.concatenate(tris), color, index))
        for child in reversed(node.get("children", [])):
            stack.append((child, world, path_name))
    return parts


# ----------------------------------------------------------------------------------------------
# Frame, grouping, decimation
AXES = {"+x": (0, 1), "-x": (0, -1), "+y": (1, 1), "-y": (1, -1), "+z": (2, 1), "-z": (2, -1)}


def axis_vector(name: str) -> np.ndarray:
    if name not in AXES:
        sys.exit(f"frame axis must be one of {sorted(AXES)}, got {name!r}")
    i, s = AXES[name]
    v = np.zeros(3)
    v[i] = s
    return v


def robot_rotation(frame: dict) -> np.ndarray:
    forward, up = axis_vector(frame.get("forward", "+x")), axis_vector(frame.get("up", "+y"))
    if abs(forward @ up) > 0.5:
        sys.exit("frame.forward and frame.up must be different axes")
    left = np.cross(up, forward)
    return np.stack([forward, left, up])  # rows: CAD -> robot


def weld(vertices: np.ndarray, faces: np.ndarray, tolerance: float = 1e-5) -> tuple[np.ndarray, np.ndarray]:
    keys = np.round(vertices / tolerance).astype(np.int64)
    _, first, inverse = np.unique(keys, axis=0, return_index=True, return_inverse=True)
    faces = inverse.reshape(-1)[faces]
    faces = faces[(faces[:, 0] != faces[:, 1]) & (faces[:, 1] != faces[:, 2]) & (faces[:, 0] != faces[:, 2])]
    return vertices[first], faces


def decimate(vertices: np.ndarray, faces: np.ndarray, target_faces: int) -> tuple[np.ndarray, np.ndarray]:
    if len(faces) <= target_faces or len(faces) < 16:
        return vertices, faces
    import fast_simplification

    reduction = 1.0 - max(target_faces, 4) / len(faces)
    v, f = fast_simplification.simplify(vertices.astype(np.float32), faces.astype(np.int32), target_reduction=reduction)
    return v.astype(np.float64), f.astype(np.int64)


def cluster(vertices: np.ndarray, faces: np.ndarray, cell: float) -> tuple[np.ndarray, np.ndarray]:
    """Vertex clustering on a uniform grid: topology-agnostic, always reaches a coarse enough target."""
    if len(faces) == 0:
        return vertices, faces
    keys = np.floor(vertices / cell).astype(np.int64)
    _, inverse, counts = np.unique(keys, axis=0, return_inverse=True, return_counts=True)
    inverse = inverse.reshape(-1)
    rep = np.zeros((len(counts), 3))
    np.add.at(rep, inverse, vertices)
    rep /= counts[:, None]
    f = inverse[faces]
    f = f[(f[:, 0] != f[:, 1]) & (f[:, 1] != f[:, 2]) & (f[:, 0] != f[:, 2])]
    if len(f):
        # Both sides of a thin plate collapse onto the same triangle: keep one (materials are double-sided).
        _, keep = np.unique(np.sort(f, axis=1), axis=0, return_index=True)
        f = f[np.sort(keep)]
    return rep, f


@dataclass
class Group:
    component: int  # -1 base
    color: tuple
    vertices: np.ndarray
    faces: np.ndarray
    out_vertices: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))
    out_faces: np.ndarray = field(default_factory=lambda: np.zeros((0, 3), dtype=np.int64))


# ----------------------------------------------------------------------------------------------
# GLB writing (flat-shaded triangles, one mesh per colour)
def write_glb(path: Path, groups: list[Group]) -> int:
    buffers: list[bytes] = []
    views, accessors, meshes, materials, nodes = [], [], [], [], []
    offset = 0

    def add_view(data: bytes, target: int) -> int:
        nonlocal offset
        pad = (-offset) % 4
        if pad:
            buffers.append(b"\0" * pad)
            offset += pad
        buffers.append(data)
        views.append({"buffer": 0, "byteOffset": offset, "byteLength": len(data), "target": target})
        offset += len(data)
        return len(views) - 1

    for g in groups:
        if len(g.out_faces) == 0:
            continue
        tri = g.out_vertices[g.out_faces].astype(np.float32)  # (m, 3, 3)
        normal = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
        length = np.linalg.norm(normal, axis=1, keepdims=True)
        normal = np.where(length > 0, normal / np.maximum(length, 1e-20), np.array([0, 0, 1], dtype=np.float32))
        positions = tri.reshape(-1, 3)
        normals = np.repeat(normal, 3, axis=0).astype(np.float32)
        count = len(positions)
        indices = np.arange(count, dtype=np.uint32 if count > 65535 else np.uint16)
        pv = add_view(positions.tobytes(), 34962)
        nv = add_view(normals.tobytes(), 34962)
        iv = add_view(indices.tobytes(), 34963)
        accessors.append({"bufferView": pv, "componentType": 5126, "count": count, "type": "VEC3",
                          "min": positions.min(axis=0).tolist(), "max": positions.max(axis=0).tolist()})
        accessors.append({"bufferView": nv, "componentType": 5126, "count": count, "type": "VEC3"})
        accessors.append({"bufferView": iv, "componentType": 5125 if count > 65535 else 5123, "count": count, "type": "SCALAR"})
        materials.append({"pbrMetallicRoughness": {"baseColorFactor": [float(c) for c in g.color],
                                                   "metallicFactor": 0.0, "roughnessFactor": 0.7},
                          "doubleSided": True,
                          **({"alphaMode": "BLEND"} if g.color[3] < 0.999 else {})})
        meshes.append({"primitives": [{"attributes": {"POSITION": len(accessors) - 3, "NORMAL": len(accessors) - 2},
                                       "indices": len(accessors) - 1, "material": len(materials) - 1}]})
        nodes.append({"mesh": len(meshes) - 1})
    binary = b"".join(buffers)
    binary += b"\0" * ((-len(binary)) % 4)
    gltf = {"asset": {"version": "2.0", "generator": "Mayhem cad-to-advantagescope"},
            "scene": 0, "scenes": [{"nodes": list(range(len(nodes)))}], "nodes": nodes, "meshes": meshes,
            "materials": materials, "accessors": accessors, "bufferViews": views,
            "buffers": [{"byteLength": len(binary)}] if binary else []}
    if not binary:
        gltf.pop("buffers")
        gltf.pop("bufferViews")
    js = json.dumps(gltf, separators=(",", ":")).encode()
    js += b" " * ((-len(js)) % 4)
    out = struct.pack("<III", 0x46546C67, 2, 12 + 8 + len(js) + (8 + len(binary) if binary else 0))
    out += struct.pack("<II", len(js), 0x4E4F534A) + js
    if binary:
        out += struct.pack("<II", len(binary), 0x004E4942) + binary
    path.write_bytes(out)
    return len(out)


# ----------------------------------------------------------------------------------------------
# Preview
PALETTE = [(230, 97, 1), (31, 120, 180), (51, 160, 44), (227, 26, 28), (106, 61, 154), (255, 187, 0), (0, 166, 166),
           (177, 89, 40), (247, 129, 191), (102, 102, 102)]


def preview(path: Path, groups: list[Group], components: list[dict], title: str) -> None:
    from PIL import Image, ImageDraw

    tris, colors = [], []
    for g in groups:
        if len(g.out_faces) == 0:
            continue
        tris.append(g.out_vertices[g.out_faces])
        base = (170, 170, 170) if g.component < 0 else PALETTE[g.component % len(PALETTE)]
        colors.append(np.tile(np.array(base, dtype=np.float64), (len(g.out_faces), 1)))
    if not tris:
        return
    tri = np.concatenate(tris)
    col = np.concatenate(colors)
    normal = np.cross(tri[:, 1] - tri[:, 0], tri[:, 2] - tri[:, 0])
    normal /= np.maximum(np.linalg.norm(normal, axis=1, keepdims=True), 1e-12)
    lo, hi = tri.reshape(-1, 3).min(axis=0), tri.reshape(-1, 3).max(axis=0)
    panel_w, panel_h, margin = 900, 620, 40
    # Side view: x right, z up, looking from -y (robot's right side) toward +y; depth = y.
    # Top view: x right, y up, looking down; depth = -z.
    views = [("side (from robot right)", 0, 2, 1, -1.0), ("top", 0, 1, 2, 1.0)]
    image = Image.new("RGB", (panel_w * 2, panel_h + 60), (250, 250, 250))
    draw = ImageDraw.Draw(image)
    for v, (label, ax_u, ax_v, ax_d, look) in enumerate(views):
        span = max(hi[ax_u] - lo[ax_u], hi[ax_v] - lo[ax_v], 1e-6)
        scale = min((panel_w - 2 * margin) / max(hi[ax_u] - lo[ax_u], 1e-6), (panel_h - 2 * margin) / max(hi[ax_v] - lo[ax_v], 1e-6))
        ox = v * panel_w + margin
        oy = panel_h - margin + 40

        def to_px(p):
            return ox + (p[..., ax_u] - lo[ax_u]) * scale, oy - (p[..., ax_v] - lo[ax_v]) * scale

        depth = tri[:, :, ax_d].mean(axis=1) * look
        order = np.argsort(depth)  # far first
        shade = 0.45 + 0.55 * np.abs(normal[:, ax_d])
        px, py = to_px(tri)
        for i in order:
            c = tuple(int(x) for x in np.clip(col[i] * shade[i], 0, 255))
            draw.polygon([(px[i, 0], py[i, 0]), (px[i, 1], py[i, 1]), (px[i, 2], py[i, 2])], fill=c)
        # Robot origin and joint markers.
        o = np.array([0.0, 0.0, 0.0])
        x0, y0 = to_px(o)
        draw.line([(x0 - 8, y0), (x0 + 8, y0)], fill=(0, 0, 0), width=2)
        draw.line([(x0, y0 - 8), (x0, y0 + 8)], fill=(0, 0, 0), width=2)
        for k, comp in enumerate(components):
            joint = comp.get("joint", {})
            jo = np.array(joint.get("origin", [0, 0, 0]), dtype=np.float64)
            axis = np.array(joint.get("axis", [0, 0, 1]), dtype=np.float64)
            jx, jy = to_px(jo)
            tip = jo + 0.15 * axis / max(np.linalg.norm(axis), 1e-9)
            tx, ty = to_px(tip)
            color = PALETTE[k % len(PALETTE)]
            draw.ellipse([jx - 5, jy - 5, jx + 5, jy + 5], outline=(0, 0, 0), fill=color)
            draw.line([(jx, jy), (tx, ty)], fill=(0, 0, 0), width=2)
        draw.text((v * panel_w + margin, 12), f"{title}: {label}   (+ = robot origin, dots = joint origins, lines = axes)",
                  fill=(0, 0, 0))
        _ = span
    legend_y = panel_h + 30
    x = margin
    draw.text((x, legend_y), "base", fill=(110, 110, 110))
    x += 50
    for k, comp in enumerate(components):
        draw.rectangle([x, legend_y, x + 12, legend_y + 12], fill=PALETTE[k % len(PALETTE)])
        draw.text((x + 16, legend_y), f"{k}: {comp['name']}", fill=(0, 0, 0))
        x += 30 + 8 * len(comp["name"]) + 20
    image.save(path)


# ----------------------------------------------------------------------------------------------
def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("glb", type=Path)
    parser.add_argument("config", type=Path)
    parser.add_argument("--out", type=Path, default=Path("."))
    parser.add_argument("--target-mb", type=float, help="override config targetMB")
    parser.add_argument("--list", action="store_true", help="print part paths and exit (to write regexes)")
    args = parser.parse_args()

    config = json.loads(args.config.read_text())
    name = config.get("name") or args.glb.stem
    target_bytes = (args.target_mb or config.get("targetMB", 15)) * 1e6
    units = float(config.get("units", 1.0))
    frame = config.get("frame", {})
    drop = [re.compile(p) for p in config.get("drop", [])]
    components = config.get("components", [])
    matchers = [[re.compile(p) for p in c.get("match", [])] for c in components]

    print(f"reading {args.glb} ...", flush=True)
    parts = load_parts(args.glb)
    rotation = robot_rotation(frame)
    for p in parts:
        p.vertices = (p.vertices * units) @ rotation.T
    total_in = sum(len(p.faces) for p in parts)
    small = float(config.get("dropSmallerThan", 0.0))

    def regex_dropped(p: Part) -> bool:
        if any(r.search(p.path) for r in drop):
            return True
        return small > 0 and np.linalg.norm(p.vertices.max(axis=0) - p.vertices.min(axis=0)) < small

    candidates = [p for p in parts if not regex_dropped(p)]
    if not candidates:
        sys.exit("every part was dropped; check the drop regexes")

    # Robot origin (in rotated CAD coordinates, before translation). The automatic floor is the
    # lowest point of the originMatch parts (the drivetrain's wheels) when given, so a stray part
    # below the floor cannot move it.
    origin = frame.get("origin", "auto")
    pattern = [re.compile(p) for p in frame.get("originMatch", [])]
    sel = [p for p in candidates if any(r.search(p.path) for r in pattern)] if pattern else candidates
    if not sel:
        print("originMatch matched nothing; using every kept part", file=sys.stderr)
        sel = candidates
    sv = np.concatenate([p.vertices for p in sel])
    if origin == "auto":
        centre = (sv.min(axis=0) + sv.max(axis=0)) / 2
    else:
        centre = (np.array(origin, dtype=np.float64) * units) @ rotation.T
    floor = frame.get("floor", "auto")
    floor_z = sv[:, 2].min() if floor == "auto" else AXES[frame.get("up", "+y")][1] * float(floor) * units
    shift = np.array([centre[0], centre[1], floor_z])
    for p in parts:
        p.vertices = p.vertices - shift

    # Per-instance boxes (an instance may be several colour parts) and the bounding-box filters.
    boxes: dict[int, tuple[np.ndarray, np.ndarray]] = {}
    for p in parts:
        lo, hi = p.vertices.min(axis=0), p.vertices.max(axis=0)
        if p.instance in boxes:
            lo, hi = np.minimum(lo, boxes[p.instance][0]), np.maximum(hi, boxes[p.instance][1])
        boxes[p.instance] = (lo, hi)
    in_component = lambda p: any(any(r.search(p.path) for r in ms) for ms in matchers)
    below = config.get("dropBelowFloor")
    outside = config.get("dropOutside")

    def box_reason(p: Part) -> str:
        lo, hi = boxes[p.instance]
        if below is not None and hi[2] < float(below):
            return "below floor"
        if outside and not in_component(p):
            c = (lo + hi) / 2
            if np.any(c < np.array(outside["min"], dtype=float)) or np.any(c > np.array(outside["max"], dtype=float)):
                return "outside"
        return ""

    if args.list:
        seen = set()
        print(f"{'node':>6}  {'centre x':>8} {'y':>7} {'z':>7}  {'z range':>15}  {'dropped':<11}  path")
        for p in parts:
            if p.instance in seen:
                continue
            seen.add(p.instance)
            lo, hi = boxes[p.instance]
            c = (lo + hi) / 2
            reason = "regex/size" if regex_dropped(p) else box_reason(p)
            print(f"{p.instance:>6}  {c[0]:8.3f} {c[1]:7.3f} {c[2]:7.3f}  {lo[2]:7.3f}..{hi[2]:<6.3f}  {reason or '-':<11}  {p.path}")
        return
    dropped_box = Counter(box_reason(p) for p in candidates)
    kept = [p for p in candidates if not box_reason(p)]
    if not kept:
        sys.exit("every part was dropped; check dropOutside / dropBelowFloor")
    print(f"{len(parts)} parts, {total_in:,} triangles; kept {len(kept)} parts, {sum(len(p.faces) for p in kept):,} triangles"
          f" (box filters dropped {dropped_box.get('outside', 0)} outside, {dropped_box.get('below floor', 0)} below the floor)")

    # Optional clip: clamp each part's robot-frame z to its component's range ("clip" on a
    # component, or top-level "clip" for the base), for parts modified after the CAD export.
    def clip_range(comp: int) -> tuple[float, float]:
        c = (components[comp].get("clip") if comp >= 0 else config.get("clip")) or {}
        return float(c.get("minZ", -math.inf)), float(c.get("maxZ", math.inf))

    clipped = 0
    for p in kept:
        comp = next((i for i, ms in enumerate(matchers) if any(r.search(p.path) for r in ms)), -1)
        lo, hi = clip_range(comp)
        if math.isfinite(lo) or math.isfinite(hi):
            before = p.vertices[:, 2].copy()
            p.vertices[:, 2] = np.clip(p.vertices[:, 2], lo, hi)
            clipped += int(np.any(before != p.vertices[:, 2]))
    if clipped:
        print(f"clip: {clipped} parts clamped to their z range")

    # Group by (component, colour), weld for decimation.
    grouped: dict[tuple, list[Part]] = {}
    counts = [0] * len(components)
    for p in kept:
        comp = next((i for i, ms in enumerate(matchers) if any(r.search(p.path) for r in ms)), -1)
        if comp >= 0:
            counts[comp] += 1
        grouped.setdefault((comp, tuple(round(c, 4) for c in p.color)), []).append(p)
    for i, c in enumerate(components):
        print(f"component {i} {c['name']}: {counts[i]} parts" + ("  (WARNING: nothing matched)" if counts[i] == 0 else ""))
    # Each part is welded and decimated on its own (welding different parts together lets edge
    # collapses drag geometry from one part across another), then merged per colour.
    groups: list[Group] = []
    group_parts: list[list[tuple[np.ndarray, np.ndarray]]] = []
    for (comp, color), ps in sorted(grouped.items(), key=lambda kv: (kv[0][0], kv[0][1])):
        welded = [weld(p.vertices, p.faces) for p in ps]
        group_parts.append(welded)
        groups.append(Group(comp, color, np.zeros((0, 3)), np.zeros((0, 3), dtype=np.int64)))
        groups[-1].faces = np.zeros((sum(len(f) for _, f in welded), 3), dtype=np.int64)  # count only
    total = sum(len(g.faces) for g in groups)

    def merge(pieces: list[tuple[np.ndarray, np.ndarray]]) -> tuple[np.ndarray, np.ndarray]:
        verts, faces, base = [], [], 0
        for v, f in pieces:
            verts.append(v)
            faces.append(f + base)
            base += len(v)
        return (np.concatenate(verts), np.concatenate(faces)) if verts else (np.zeros((0, 3)), np.zeros((0, 3), dtype=np.int64))

    out_dir = args.out / f"Robot_{re.sub(r'[^A-Za-z0-9_-]+', '', name.replace(' ', '')) or 'Model'}"
    out_dir.mkdir(parents=True, exist_ok=True)
    budget = target_bytes * 0.97 / BYTES_PER_FACE
    # 1) Optional quadric decimation per part ("quadric": true). Off by default: on CAD exports it
    #    stalls on small holed parts and folds long thin parts (bumpers, rails) into wedges.
    ratio = min(1.0, budget / max(total, 1))
    keep = max(ratio, 0.15)
    if config.get("quadric", False):
        quadric = [merge([decimate(v, f, max(12, int(len(f) * keep))) for v, f in pieces]) for pieces in group_parts]
    else:
        quadric = [merge(pieces) for pieces in group_parts]
    print(f"{'quadric decimation' if config.get('quadric') else 'welded'}: {sum(len(f) for _, f in quadric):,} triangles (budget {int(budget):,})")
    sizes: list[int] = []
    for attempt in range(4):
        # 2) If still over budget, uniform vertex clustering with the cell size found by bisection.
        if sum(len(f) for _, f in quadric) > budget:
            lo_cell, hi_cell = 1e-4, 0.2
            for _ in range(14):
                mid = math.sqrt(lo_cell * hi_cell)
                n = sum(len(cluster(v, f, mid)[1]) for v, f in quadric)
                lo_cell, hi_cell = (mid, hi_cell) if n > budget else (lo_cell, mid)
            print(f"vertex clustering at {hi_cell * 1000:.1f} mm cells")
            for g, (v, f) in zip(groups, quadric):
                g.out_vertices, g.out_faces = cluster(v, f, hi_cell)
        else:
            for g, (v, f) in zip(groups, quadric):
                g.out_vertices, g.out_faces = v, f
        sizes = [write_glb(out_dir / "model.glb", [g for g in groups if g.component < 0])]
        for i in range(len(components)):
            sizes.append(write_glb(out_dir / f"model_{i}.glb", [g for g in groups if g.component == i]))
        used = sum(sizes)
        print(f"pass {attempt + 1}: {sum(len(g.out_faces) for g in groups):,} triangles, {used / 1e6:.2f} MB")
        if used <= target_bytes:
            break
        budget *= target_bytes / used * 0.97

    cfg = {
        "name": name,
        "isFTC": False,
        "rotations": [],
        "position": [0, 0, 0],
        "cameras": [],
        "components": [
            {"zeroedRotations": [], "zeroedPosition": [0.0 - float(x) for x in c.get("joint", {}).get("origin", [0, 0, 0])]}
            for c in components
        ],
    }
    (out_dir / "config.json").write_text(json.dumps(cfg, indent=2) + "\n")
    joints = {
        "name": name,
        "note": "Log one Pose3d per component, in this order: revolute Pose3d(origin, Rotation3d(axis * angle)); "
                "prismatic Pose3d(origin + axis * distance, Rotation3d()). Robot frame, metres.",
        "components": [{"index": i, "name": c["name"], **c.get("joint", {})} for i, c in enumerate(components)],
    }
    (args.out / f"{out_dir.name}.joints.json").write_text(json.dumps(joints, indent=2) + "\n")
    preview(args.out / f"{out_dir.name}.preview.png", groups, components, name)
    lo, hi = np.concatenate([g.out_vertices for g in groups if len(g.out_vertices)]).min(axis=0), \
        np.concatenate([g.out_vertices for g in groups if len(g.out_vertices)]).max(axis=0)
    print(f"robot-frame bounds x {lo[0]:.3f}..{hi[0]:.3f}  y {lo[1]:.3f}..{hi[1]:.3f}  z {lo[2]:.3f}..{hi[2]:.3f} m")
    print(f"wrote {out_dir} ({sum(sizes) / 1e6:.2f} MB), {out_dir.name}.preview.png, {out_dir.name}.joints.json")


if __name__ == "__main__":
    main()
