#!/usr/bin/env python3
# /// script
# requires-python = ">=3.10"
# dependencies = ["numpy>=1.26", "pyfqmr>=0.2"]
# ///
"""Per-part simplification of a CAD GLB before cad-to-advantagescope.py.

    uv run tools/sim/simplify_parts.py 00.Main.coarse.glb 00.Main.light.glb 400 0.08 1500

Merges each part's faces (Onshape exports one primitive per face), then decimates only parts with
more than <keep> triangles to <frac> of their triangles (at most <cap>) with pyfqmr, keeping part
borders. Plates, the hopper and the bumpers (EXACT) keep Onshape's exact tessellation, and the
hopper's polycarbonate gets a clear material (the CAD colours it black). The exporter then only has
to cluster at ~1 mm, which keeps 1/8 in plates intact.
"""
import sys, json, struct, numpy as np, pyfqmr
src, dst, keep, frac, cap = sys.argv[1], sys.argv[2], int(sys.argv[3]), float(sys.argv[4]), int(sys.argv[5])
d = open(src, 'rb').read(); clen = struct.unpack('<I', d[12:16])[0]; j = json.loads(d[20:20+clen]); b = d[20+clen+8:]
CT = {5126: (np.float32, 4), 5125: (np.uint32, 4), 5123: (np.uint16, 2), 5121: (np.uint8, 1)}
def read(i):
    a = j['accessors'][i]; v = j['bufferViews'][a['bufferView']]; dt, sz = CT[a['componentType']]
    n = {'SCALAR': 1, 'VEC3': 3, 'VEC2': 2, 'VEC4': 4}[a['type']]
    o = v.get('byteOffset', 0) + a.get('byteOffset', 0); stride = v.get('byteStride', 0)
    if stride and stride != sz * n:
        raw = np.frombuffer(b, dtype=np.uint8, count=stride * a['count'], offset=o).reshape(-1, stride)[:, :sz*n]
        return raw.copy().view(dt).reshape(-1, n)
    arr = np.frombuffer(b, dtype=dt, count=a['count'] * n, offset=o)
    return arr.reshape(-1, n) if n > 1 else arr
out = bytearray(); views = []; accs = []
def add(arr, target=None, kind='pos'):
    arr = np.ascontiguousarray(arr); off = len(out); out.extend(arr.tobytes()); out.extend(b'\0' * ((4 - len(out) % 4) % 4))
    views.append({'buffer': 0, 'byteOffset': off, 'byteLength': arr.nbytes, **({'target': target} if target else {})})
    if kind == 'pos':
        accs.append({'bufferView': len(views)-1, 'componentType': 5126, 'count': len(arr), 'type': 'VEC3', 'min': arr.min(0).tolist(), 'max': arr.max(0).tolist()})
    else:
        accs.append({'bufferView': len(views)-1, 'componentType': 5125, 'count': arr.size, 'type': 'SCALAR'})
    return len(accs) - 1
import re
EXACT = re.compile(r'(?i)hopper|bumper|backing|0222_|0211_|0401_|0406_|0407_|0414_|0201_|0202_|0203_|0204_')
exact_meshes = {n['mesh'] for n in j['nodes'] if 'mesh' in n and EXACT.search(n.get('name') or '')}
# Clear polycarbonate: the CAD colours the hopper panels solid black.
POLYCARB = re.compile(r'(?i)0211_Hopper_Front|0222_Hopper_Side|^Hopper[ _](Back|Side)')
polycarb_meshes = {n['mesh'] for n in j['nodes'] if 'mesh' in n and POLYCARB.search(n.get('name') or '')}
j.setdefault('materials', []).append({'pbrMetallicRoughness': {'baseColorFactor': [0.82, 0.88, 0.95, 0.35], 'metallicFactor': 0.0, 'roughnessFactor': 0.2}, 'alphaMode': 'BLEND', 'doubleSided': True})
CLEAR = len(j['materials']) - 1
before = after = 0
for mi, m in enumerate(j['meshes']):
    groups = {}
    for p in m['primitives']:
        P = read(p['attributes']['POSITION']).astype(np.float64); I = read(p['indices']).astype(np.int64).reshape(-1, 3)
        g = groups.setdefault(p.get('material', -1), ([], []))
        off = sum(len(x) for x in g[0]); g[0].append(P); g[1].append(I + off)
    total = sum(sum(len(i) for i in g[1]) for g in groups.values()); before += total
    prims = []
    for mat, (Ps, Is) in groups.items():
        P = np.concatenate(Ps); I = np.concatenate(Is)
        # weld coincident vertices so faces join into one surface
        key = np.round(P / 1e-6).astype(np.int64)
        _, first, inv = np.unique(key, axis=0, return_index=True, return_inverse=True)
        P = P[first].astype(np.float32); I = inv.reshape(-1)[I].astype(np.int32)
        I = I[(I[:, 0] != I[:, 1]) & (I[:, 1] != I[:, 2]) & (I[:, 0] != I[:, 2])]
        n = len(I)
        if mi not in exact_meshes and total > keep and n > 60:
            target = int(min(cap * n / total, max(keep * n / total, n * frac)))
            sim = pyfqmr.Simplify(); sim.setMesh(P.astype(np.float64), I.astype(np.int64))
            sim.simplify_mesh(target_count=max(12, target), aggressiveness=5, preserve_border=True, verbose=False)
            P2, I2, _ = sim.getMesh()
            if len(I2) > 0: P, I = P2.astype(np.float32), I2.astype(np.int32)
        after += len(I)
        prim = {'attributes': {'POSITION': add(P, 34962, 'pos')}, 'indices': add(I.astype(np.uint32).reshape(-1), 34963, 'idx')}
        if mi in polycarb_meshes: prim['material'] = CLEAR
        elif mat >= 0: prim['material'] = mat
        prims.append(prim)
    m['primitives'] = prims
j['accessors'] = accs; j['bufferViews'] = views; j['buffers'] = [{'byteLength': len(out)}]
for k in ('textures', 'images', 'samplers'): j.pop(k, None)
for mat in j.get('materials', []):
    pbr = mat.get('pbrMetallicRoughness', {}); pbr.pop('baseColorTexture', None)
js = json.dumps(j, separators=(',', ':')).encode(); js += b' ' * ((4 - len(js) % 4) % 4)
blob = struct.pack('<III', 0x46546C67, 2, 28 + len(js) + len(out)) + struct.pack('<II', len(js), 0x4E4F534A) + js + struct.pack('<II', len(out), 0x004E4942) + bytes(out)
open(dst, 'wb').write(blob); print(f'unique-mesh triangles {before:,} -> {after:,}; {len(blob)/1e6:.1f} MB')
