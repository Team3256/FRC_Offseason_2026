"""Stage INTERNAL target movements for arrival-region reconstruction.

Use group_imported_autos.py for final user-facing cycle/variant paths.

Usage: python tools/split_target_autos.py OUTPUT_DIRECTORY
Reads the current robot files, never regenerates edited 581 goals from source.
Uses the installed Mayhem solver's persistent stdio backend. Source checkouts
are supplied through --source2910 and --source581.
"""
import argparse
import copy
import json
import math
from pathlib import Path
import queue
import re
import subprocess
import sys
import threading

from import_2910_paths import Resolver, route_calls
from import_team581_comp_paths import load_auto

ROOT=Path(__file__).resolve().parents[1]
DEPLOY=ROOT/'src/main/deploy/mayhem'


def front_intake_pose(pose):
    """2910 collects toward robot -X; our deployed intake is robot +X."""
    return [pose[0],pose[1],(pose[2]+2*math.pi)%(2*math.pi)-math.pi]


class Solver:
    def __init__(self):
        self.process=subprocess.Popen(['/Applications/Mayhem.app/Contents/MacOS/mayhem-solver','serve'],
            stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=sys.stderr,text=True,bufsize=1)
        self.lines=queue.Queue();self.mid=0
        def pump():
            for line in self.process.stdout:
                try:self.lines.put(json.loads(line))
                except json.JSONDecodeError:pass
            self.lines.put(None)
        threading.Thread(target=pump,daemon=True).start()
        assert self.recv()['method']=='ready'
    def recv(self):
        value=self.lines.get(timeout=150)
        if value is None:raise RuntimeError('Mayhem solver exited')
        return value
    def call(self,method,params=None):
        self.mid+=1
        self.process.stdin.write(json.dumps({'id':self.mid,'method':method,'params':params or {}})+'\n')
        self.process.stdin.flush()
        while True:
            msg=self.recv()
            if msg.get('id')==self.mid:
                if 'error' in msg:raise RuntimeError(msg['error'])
                return msg['result']
    def solve(self,project,trajectory):
        self.call('solve',{'project':project,'trajectory':trajectory})
        while True:
            msg=self.recv()
            if msg.get('method')=='solveDone':
                result=msg['params']
                if not result['success']:raise RuntimeError(json.dumps(result.get('issues',[])))
                return result['output']
    def close(self):
        self.call('shutdown');self.process.wait(timeout=15)


def waypoint(identifier,pose,radius=0):
    return {'id':identifier,'x':pose[0],'y':pose[1],
        'heading':0.0 if pose[2]==0 else pose[2],
        'translationMode':'fixed','headingMode':'fixed','headingTolerance':0,
        'tolerance':{'kind':'circle' if radius else 'none','radius':radius or .1,'dx':.1,'dy':.1},
        'stop':True,'split':False,'intervals':None,'poseRef':None}


def base_path(name,start,target,radius,folder):
    return {'formatVersion':1,'name':name,'folder':folder,
        'waypoints':[waypoint('start',start),waypoint('target',target,radius)],
        'constraints':[],'markers':[],
        'settings':{'targetDt':.08,'smoothing':.02,'candidates':1,'maxIterations':1500,'timeLimit':45},
        'output':None}


def constraint(identifier,data,first=0,last=1):
    return {'id':identifier,'enabled':True,'scope':{'kind':'range','from':first,'to':last},'data':data}


def wall_clearance_radius(project,pose,radius):
    """A destination intake may require more clearance than a source robot."""
    bumper=project['robot']['bumper'];front=bumper['front']+project['robot']['intake']['extension']
    angle=pose[2]
    corners=[(x*math.cos(angle)-y*math.sin(angle),x*math.sin(angle)+y*math.cos(angle))
             for x,y in [(front,bumper['left']),(front,-bumper['right']),
                         (-bumper['back'],bumper['left']),(-bumper['back'],-bumper['right'])]]
    field=project['field'];margin=field['wallMargin']
    x=min(max(pose[0],margin-min(x for x,y in corners)),field['length']-margin-max(x for x,y in corners))
    y=min(max(pose[1],margin-min(y for x,y in corners)),field['width']-margin-max(y for x,y in corners))
    distance=math.hypot(x-pose[0],y-pose[1])
    return math.ceil((distance+.04)*100)/100 if distance>radius else radius


def generate(solver,project,trajectory,stage,entry):
    # Bound each movement near its target ray. Arrival regions, rather than the
    # target centers, determine where this command hands over to the next one.
    start,target=trajectory['waypoints']
    if math.hypot(target['x']-start['x'],target['y']-start['y'])>1e-4:
        trajectory['constraints'].append(constraint('target-ray',{'type':'straightLine','tolerance':.25}))
    cached=stage/(trajectory['name']+'.mtraj')
    old=json.loads(cached.read_text()) if cached.exists() else None
    same=old and old.get('output') and {k:v for k,v in old.items() if k!='output'}=={k:v for k,v in trajectory.items() if k!='output'}
    if same:
        trajectory['output']=old['output']
    else:
        try:
            trajectory['output']=solver.solve(project,trajectory)
        except RuntimeError as error:
            if not any(c['id']=='target-ray' for c in trajectory['constraints']):raise
            if 'passes through obstacle' in str(error):
                trajectory['constraints']=[c for c in trajectory['constraints'] if c['id']!='target-ray']
                entry['clearanceAdaptation']='direct target ray intersects field geometry; solve a local obstacle-avoiding movement'
            else:
                next(c for c in trajectory['constraints'] if c['id']=='target-ray')['data']['tolerance']=.5
                entry['clearanceAdaptation']='ray corridor widened from 0.25 to 0.50 m for footprint clearance'
            trajectory['output']=solver.solve(project,trajectory)
    endpoint=trajectory['output']['samples'][-1]
    end=[endpoint['x'],endpoint['y'],endpoint['heading']]
    entry.update(name=trajectory['name'],start=[start['x'],start['y'],start['heading']],
                 target=[target['x'],target['y'],target['heading']],arrivalRadius=target['tolerance']['radius']
                 if target['tolerance']['kind']=='circle' else 0,endpoint=end)
    (stage/(trajectory['name']+'.mtraj')).write_text(json.dumps(trajectory,indent=1)+'\n')
    return end


def make_2910(solver,project,checkout,stage,manifest):
    base=checkout/'src/main/java/org/frc2910/robot'
    resolver=Resolver((base/'constants/FieldConstants.java').read_text())
    factory=(base/'autos/AutoFactory.java').read_text()
    methods={'createLeftNeutralZoneAuto':'DepotNZ','createRightNeutralZoneAuto':'HPNZ',
             'createLeftNeutralZoneToDepotAuto':'DepotNZDepot','createDepotAuto':'Depot'}
    for method,prefix in methods.items():
        match=re.search(r'Pair<Pose2d, Command> '+method+r'\(\)\s*\{',factory)
        body=re.split(r'\n    (?:Pair<Pose2d, Command>|Command) \w+\(',factory[match.end():],1)[0]
        current=front_intake_pose(resolver.resolve(re.search(r'FieldConstants\.\w+\(alliance\)',body)[0]))
        sequence=[];source_command=0
        for call in route_calls(body,resolver):
            if call is None:
                sequence.append({'kind':'score','note':'stationary scoring command; no driving path'})
                continue
            source_command+=1
            call['pose']=front_intake_pose(call['pose'])
            if call['hold']:
                sequence.append({'kind':'timedHold','sourceCommand':source_command,'duration':.5,
                                 'sourceCall':call['sourceCall'],'target':call['pose']})
                continue
            distance=math.hypot(current[0]-call['pose'][0],current[1]-call['pose'][1])
            if distance<=call['tolerance']+1e-5:
                sequence.append({'kind':'alreadySatisfied','sourceCommand':source_command,
                                 'sourceCall':call['sourceCall'],'position':current})
                continue
            number=sum(e.get('kind')=='path' for e in sequence)+1
            name=f'{prefix}-{number:02}'
            folder='2910 - Bottom - HP Station' if prefix=='HPNZ' else '2910 - Top - Depot'
            radius=wall_clearance_radius(project,call['pose'],call['tolerance'])
            trajectory=base_path(name,current,call['pose'],radius,folder)
            trajectory['constraints'] += [constraint('speed',{'type':'maxVelocity','value':call['speed']}),
                constraint('angular-rate',{'type':'maxAngularVelocity','value':5.0}),
                constraint('intake-extended',{'type':'intakeExtended'},1 if number==1 else 0)]
            for obstacle in project['field']['obstacles']:
                if obstacle['id'].startswith('bump-'):
                    trajectory['constraints'].append({'id':'terrain-'+obstacle['id'],'enabled':True,
                        'scope':{'kind':'zone','region':obstacle['points']},
                        'data':{'type':'roughTerrain','expectedSpeed':.7,'feedbackScale':.3}})
            entry={'kind':'path','team':2910,'routine':method,'sourceCommand':source_command,
                   'sourceCall':call['sourceCall'],'speed':call['speed'],
                   'sourceArrivalRadius':call['tolerance']}
            if radius!=call['tolerance']:
                entry['wallClearanceAdaptation']='arrival radius enlarged to clear the field wall with our front intake'
            current=generate(solver,project,trajectory,stage,entry)
            sequence.append(entry)
            print('2910',name,'arrival radius',call['tolerance'],flush=True)
        manifest['routines'].append({'team':2910,'sourceMethod':method,'sequence':sequence})


def make_581(solver,project,checkout,stage,manifest):
    layout=json.loads((ROOT/'src/main/deploy/apriltags/welded/2026-official.json').read_text())
    tags={t['ID']:t['pose']['translation'] for t in layout['tags']};bump=(48.94+12+73/2)*.0254
    constants={'FieldUtil.RED_DEPOT_TRENCH_CENTER.getY()':tags[17]['y'],
        'FieldUtil.RED_OUTPOST_TRENCH_CENTER.getY()':tags[22]['y'],
        'FieldUtil.RED_DEPOT_BUMP_CENTER.getY()':bump,
        'FieldUtil.RED_OUTPOST_BUMP_CENTER.getY()':8.069-bump,'FieldUtil.RED_HUB_CENTER.getY()':8.069/2}
    for parent in sorted(DEPLOY.glob('*Normal*.mtraj')):
        edited=json.loads(parent.read_text())
        if not (edited.get('folder') or '').startswith('581 - '):continue
        side='Left' if parent.stem.startswith('Left') else 'Right'
        source=checkout/f'comp-bot/src/main/java/frc/robot/autos/auto_state_machines/{side}NormalAuto.java'
        _,segments=load_auto(source,constants)
        if 'FirstCycle' in parent.stem:key='intakeFirstCycle';suffix='N1'
        elif 'ThirdCycle' in parent.stem:key='intakeThirdCycle';suffix='N3'
        else:key='defaultIntakeSecondCycle';suffix='N2Far' if 'Far' in parent.stem else 'N2'
        points=segments[key]['points']
        if suffix=='N2Far':
            decision=next(i for i,p in enumerate(points) if p['marker']=='MAKE_CLUSTER_MAP_DECISION')
            points=points[:decision]+segments['intakeSecondCycleFar']['points']
        sequence=[];wps=edited['waypoints'];current=[wps[0][k] for k in ('x','y','heading')]
        for index in range(1,len(wps)):
            goal=wps[index];identifier=goal['id']
            source_index=int(identifier[1:])-1 if re.fullmatch(r'w\d+',identifier) else -1
            tolerance=points[source_index].get('transitionTolerance') if 0<=source_index<len(points) else None
            radius=tolerance[0] if tolerance else (goal['tolerance']['radius'] if goal['tolerance']['kind']=='circle' else 0)
            if index==len(wps)-1:radius=.5 if suffix=='N3' else .2
            name=f'{side}{suffix}-{index:02}'
            trajectory=base_path(name,current,[goal[k] for k in ('x','y','heading')],radius,edited['folder'])
            # Clip each existing edited constraint to this pair; retain all zones.
            for c in edited['constraints']:
                c=copy.deepcopy(c);scope=c['scope']
                if scope['kind']=='zone':trajectory['constraints'].append(c);continue
                a=scope['from'];b=scope['to'] if scope['kind']=='range' else a
                a,b=sorted((a,b));lo,hi=max(a,index-1),min(b,index)
                if lo>hi:continue
                c['scope']={'kind':'range' if lo<hi else 'waypoint','from':lo-(index-1),'to':hi-(index-1)}
                trajectory['constraints'].append(c)
            for marker in edited['markers']:
                if marker['waypoint']==index:
                    marker=copy.deepcopy(marker);marker['waypoint']=1;trajectory['markers'].append(marker)
            entry={'kind':'path','team':581,'parentPath':parent.stem,'sourceWaypointId':identifier,
                   'editedGoal':copy.deepcopy(goal),'sourceTransitionTolerance':tolerance}
            current=generate(solver,project,trajectory,stage,entry)
            sequence.append(entry)
            print('581',name,'arrival radius',radius,flush=True)
        manifest['routines'].append({'team':581,'parentPath':parent.stem,'sequence':sequence})


def main():
    parser=argparse.ArgumentParser();parser.add_argument('stage',type=Path)
    parser.add_argument('--source2910',type=Path,required=True);parser.add_argument('--source581',type=Path,required=True)
    args=parser.parse_args();args.stage.mkdir(parents=True,exist_ok=True)
    project=json.loads((DEPLOY/'project.mayhem').read_text())
    (args.stage/'project.mayhem').write_text(json.dumps(project,indent=2)+'\n')
    manifest={'routines':[],'note':'independent movements; source target radii preserved; desired goal headings and endpoint stops are adaptations for our Mayhem robot'}
    solver=Solver()
    try:
        make_2910(solver,project,args.source2910,args.stage,manifest)
        make_581(solver,project,args.source581,args.stage,manifest)
    finally:
        solver.close()
        (args.stage/'split-manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')


if __name__=='__main__':main()
