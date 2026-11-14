#!/usr/bin/env python3
"""Export the robot assembly (00.Main) from Onshape as a coarsely tessellated GLB.

    ONSHAPE_ACCESS_KEY=... ONSHAPE_SECRET_KEY=... python3 tools/sim/onshape_export.py coarse 00.Main.coarse.glb

Onshape's own tessellation at the "coarse" resolution keeps every part's real shape with ~2M
triangles (the default download has ~5.7M), which tools/sim/simplify_parts.py then reduces per part.
No dependencies.
"""
import sys, hmac, hashlib, base64, random, string, json, urllib.request, email.utils, time
import os
# Onshape API keys (https://dev-portal.onshape.com/keys, read scope) from the environment.
acc, sec = os.environ['ONSHAPE_ACCESS_KEY'], os.environ['ONSHAPE_SECRET_KEY']
def call(method, path, query='', body=None, accept='application/json'):
    nonce = ''.join(random.choice(string.ascii_letters + string.digits) for _ in range(25))
    date = email.utils.formatdate(usegmt=True); ct = 'application/json'
    s = (method + '\n' + nonce + '\n' + date + '\n' + ct + '\n' + path + '\n' + query + '\n').lower()
    sig = base64.b64encode(hmac.new(sec.encode(), s.encode(), hashlib.sha256).digest()).decode()
    data = json.dumps(body).encode() if body is not None else None
    req = urllib.request.Request('https://cad.onshape.com' + path + ('?' + query if query else ''), data=data, method=method.upper(), headers={
        'Authorization': f'On {acc}:HmacSHA256:{sig}', 'On-Nonce': nonce, 'Date': date, 'Content-Type': ct, 'Accept': accept})
    try:
        with urllib.request.urlopen(req) as r: return r.status, r.read()
    except urllib.error.HTTPError as e: return e.code, e.read()
D, W, E = '2cb0d352e3910b9de977dd2f', '33bcd7f51c4f605560058582', '5cc3f3a16ff2cea0a5656d82'
res = sys.argv[1] if len(sys.argv) > 1 else 'coarse'
out_path = sys.argv[2] if len(sys.argv) > 2 else '00.Main.coarse.glb'
body = {"formatName": "GLTF", "storeInDocument": False, "resolution": res, "flattenAssemblies": False}
st, out = call('post', f'/api/v10/assemblies/d/{D}/w/{W}/e/{E}/translations', '', body)
print('start', st, out[:300])
if st != 200: sys.exit(1)
tid = json.loads(out)['id']
for i in range(180):
    time.sleep(5)
    st, out = call('get', f'/api/v10/translations/{tid}')
    j = json.loads(out); print(i, j.get('requestState'), j.get('failureReason'))
    if j.get('requestState') in ('DONE', 'FAILED'): break
fid = (j.get('resultExternalDataIds') or [None])[0]
st, data = call('get', f'/api/v10/documents/d/{D}/externaldata/{fid}', accept='application/octet-stream')
# Onshape returns glTF JSON with an embedded base64 buffer; write a binary .glb.
import base64, struct
g = json.loads(data)
buf = base64.b64decode(g['buffers'][0].pop('uri').split(',', 1)[1])
g['buffers'][0]['byteLength'] = len(buf)
js = json.dumps(g, separators=(',', ':')).encode(); js += b' ' * ((4 - len(js) % 4) % 4)
buf += b'\0' * ((4 - len(buf) % 4) % 4)
glb = (struct.pack('<III', 0x46546C67, 2, 28 + len(js) + len(buf)) + struct.pack('<II', len(js), 0x4E4F534A)
       + js + struct.pack('<II', len(buf), 0x004E4942) + buf)
open(out_path, 'wb').write(glb)
print('saved', out_path, f'{len(glb) / 1e6:.1f} MB')
