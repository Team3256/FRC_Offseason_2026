"""Stage a 180 degree intake-frame correction of the CURRENT edited Valor paths.

Run with Mayhem's solver Python environment for Shapely. Never operates on its
own staged output, so rerunning cannot accidentally rotate the paths twice.
"""
import copy
import json
import math
from pathlib import Path
import sys

from import_valor_paths import adapt_intake_clearance
from split_target_autos import DEPLOY, Solver


def main():
    stage=Path(sys.argv[1]);stage.mkdir(parents=True,exist_ok=True)
    project=json.loads((DEPLOY/'project.mayhem').read_text())
    (stage/'project.mayhem').write_text(json.dumps(project,indent=2)+'\n')
    manifest=[];solver=Solver()
    try:
        for path in sorted(DEPLOY.glob('*.mtraj')):
            trajectory=json.loads(path.read_text())
            if not (trajectory.get('folder') or '').startswith('6800 - '):continue
            original=copy.deepcopy(trajectory)
            for point in trajectory['waypoints']:
                point['heading']=(point['heading']+2*math.pi)%(2*math.pi)-math.pi
                if point['heading']==0:point['heading']=0.0
            adaptations=adapt_intake_clearance(trajectory,project,route_trench=False)
            trajectory['output']=None
            target=stage/path.name
            cached=json.loads(target.read_text()) if target.exists() else None
            if cached and cached.get('output') and {k:v for k,v in cached.items() if k!='output'}=={k:v for k,v in trajectory.items() if k!='output'}:
                trajectory['output']=cached['output']
            else:
                trajectory['output']=solver.solve(project,trajectory)
            assert [(p['x'],p['y']) for p in original['waypoints']]==[(p['x'],p['y']) for p in trajectory['waypoints']]
            target.write_text(json.dumps(trajectory,indent=1)+'\n')
            manifest.append({'name':trajectory['name'],'headingOffsetRadians':math.pi,'adaptations':adaptations})
            print('Rotated and generated',trajectory['name'],flush=True)
    finally:
        solver.close()
        (stage/'heading-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')


if __name__=='__main__':main()
