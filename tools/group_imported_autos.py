"""Stage complete cycle/variant options; point-to-point solves are internal aids.

Run after split_target_autos.py, with its staging directory as the second argument.
581 definitions come from the archived user-edited parents. 2910 uses the solved
arrival-region handoff poses, avoiding the original incorrect target-center loop.
"""
import copy
import json
from pathlib import Path
import sys

from split_target_autos import ROOT, DEPLOY, Solver, constraint, waypoint


def main():
    stage=Path(sys.argv[1]);source=Path(sys.argv[2]);stage.mkdir(parents=True,exist_ok=True)
    project=json.loads((DEPLOY/'project.mayhem').read_text())
    (stage/'project.mayhem').write_text(json.dumps(project,indent=2)+'\n')
    manifest=json.loads((source/'split-manifest.json').read_text())
    groups=[];solver=Solver()
    def solve(t):
        target=stage/(t['name']+'.mtraj')
        old=json.loads(target.read_text()) if target.exists() else None
        if old and old.get('output') and {k:v for k,v in old.items() if k!='output'}=={k:v for k,v in t.items() if k!='output'}:
            t['output']=old['output']
        else:t['output']=solver.solve(project,t)
        target.write_text(json.dumps(t,indent=1)+'\n')
        print('Generated complete route',t['name'],flush=True)
    try:
        for f in sorted((ROOT/'docs/third-party/pre-split-definitions').glob('*Normal*.mtraj')):
            t=json.loads(f.read_text());solve(t)
            groups.append({'name':t['name'],'team':581,'editedParent':f.name})
        names={
            'createLeftNeutralZoneAuto':['DepotNZFirstCycle','DepotNZSecondCycle','DepotNZThirdIntake'],
            'createRightNeutralZoneAuto':['HPNZFirstCycle','HPNZSecondCycle','HPNZThirdIntake'],
            'createLeftNeutralZoneToDepotAuto':['DepotNZAltFirstCycle','DepotNZAltSecondCycle','DepotNZToDepot'],
            'createDepotAuto':['DepotCollection','DepotExit']}
        for routine in manifest['routines']:
            if routine['team']!=2910:continue
            cycles=[];events=[]
            for entry in routine['sequence']:
                if entry['kind']=='score':
                    if any(e['kind']=='path' for e in events):cycles.append(events);events=[]
                else:events.append(entry)
            if any(e['kind']=='path' for e in events):cycles.append(events)
            assert len(cycles)==len(names[routine['sourceMethod']])
            for number,(name,events) in enumerate(zip(names[routine['sourceMethod']],cycles)):
                legs=[e for e in events if e['kind']=='path']
                folder='2910 - Bottom - HP Station' if name.startswith('HP') else '2910 - Top - Depot'
                t={'formatVersion':1,'name':name,'folder':folder,'waypoints':[],
                   'constraints':[],'markers':[],
                   'settings':{'targetDt':.08,'smoothing':.02,'candidates':2,'maxIterations':1500,'timeLimit':60},'output':None}
                t['waypoints']=[waypoint('start',legs[0]['start'])]
                for i,leg in enumerate(legs):
                    wp=waypoint(f'arrival-{i+1}',leg['endpoint']);wp['stop']=i==len(legs)-1
                    t['waypoints'].append(wp)
                    t['constraints'].append(constraint(f'speed-{i}',{'type':'maxVelocity','value':leg['speed']},i,i+1))
                t['constraints'] += [constraint('angular-rate',{'type':'maxAngularVelocity','value':5},0,len(legs)),
                    constraint('intake-extended',{'type':'intakeExtended'},1 if number==0 else 0,len(legs))]
                for obstacle in project['field']['obstacles']:
                    if obstacle['id'].startswith('bump-'):
                        t['constraints'].append({'id':'terrain-'+obstacle['id'],'enabled':True,
                            'scope':{'kind':'zone','region':obstacle['points']},
                            'data':{'type':'roughTerrain','expectedSpeed':.7,'feedbackScale':.3}})
                solve(t)
                groups.append({'name':name,'team':2910,'sourceMethod':routine['sourceMethod'],
                    'events':copy.deepcopy(events),'note':'continuous cycle; internal arrival poses are fixed handoff locations, intermediate stops removed'})
    finally:
        solver.close()
        (stage/'grouped-manifest.json').write_text(json.dumps(groups,indent=2)+'\n')


if __name__=='__main__':main()
