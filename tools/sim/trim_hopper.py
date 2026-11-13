#!/usr/bin/env python3
"""Trim the exported intake slide model to the real hopper height.

The Onshape CAD (00.Main) still has the old hopper, whose polycarbonate reaches 0.61 m (24 in).
The real hopper was cut down to fit under the 22.25 in (0.565 m) trench opening, so after
cad-to-advantagescope.py, clamp the slide component's geometry to that height:

    python3 tools/sim/trim_hopper.py <assets>/Robot_32562026/model_0.glb

(robot frame, z up; the same height as kHopperTopStowedM in src/main/include/sim/SimConstants.h).
No dependencies.
"""
import json
import struct
import sys
from pathlib import Path

HOPPER_TOP_M = 0.555


def main() -> None:
    path = Path(sys.argv[1])
    top = float(sys.argv[2]) if len(sys.argv) > 2 else HOPPER_TOP_M
    data = bytearray(path.read_bytes())
    json_len = struct.unpack_from("<I", data, 12)[0]
    gltf = json.loads(data[20 : 20 + json_len])
    bin_offset = 20 + json_len + 8  # BIN chunk header
    clamped = 0
    positions = {p["attributes"]["POSITION"] for m in gltf["meshes"] for p in m["primitives"]}
    for index in positions:
        acc = gltf["accessors"][index]
        view = gltf["bufferViews"][acc["bufferView"]]
        start = bin_offset + view.get("byteOffset", 0) + acc.get("byteOffset", 0)
        stride = view.get("byteStride", 12)
        for i in range(acc["count"]):
            at = start + i * stride + 8  # z
            if struct.unpack_from("<f", data, at)[0] > top:
                struct.pack_into("<f", data, at, top)
                clamped += 1
        acc["max"][2] = min(acc["max"][2], top)
    # Rewrite the JSON chunk (same padded length or longer).
    text = json.dumps(gltf, separators=(",", ":")).encode()
    text += b" " * ((4 - len(text) % 4) % 4)
    out = data[:12] + struct.pack("<II", len(text), 0x4E4F534A) + text + data[20 + json_len :]
    struct.pack_into("<I", out, 8, len(out))
    path.write_bytes(out)
    print(f"{path}: {clamped} vertices clamped to z <= {top} m")


if __name__ == "__main__":
    main()
