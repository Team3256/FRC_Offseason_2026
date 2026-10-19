"""Convert Valor's referenced Choreo legs into editable Mayhem definitions.

Usage: python tools/import_valor_paths.py SOURCE_CHECKOUT OUTPUT_DIRECTORY
OUTPUT_DIRECTORY must already contain project.mayhem (the receiving robot config).
The output is staged: this does not modify existing robot trajectories.
"""
import copy
import json
import math
from pathlib import Path
import re
import sys

WIDTH = 8.0692
NAMES = {'safefirstleg': 'SafeFirstLeg', 'safesecondleg': 'SafeSecondLeg',
         'safepsecondleg': 'PoofsSecondLeg', 'aggrofirstleg': 'AggroFirstLeg',
         'aggrosecondleg': 'AggroSecondLeg', 'optimizefirstleg': 'OptimizedFirstLeg',
         'optimizesecondleg': 'OptimizedSecondLeg', 'pointfive': 'HalfLeg',
         'trenchfirstleg': 'TrenchFirstLeg', 'trenchsecondleg': 'TrenchSecondLeg'}


def convert(source, depot, project):
    snapshot = source['snapshot']
    mirrored = (snapshot['waypoints'][0]['y'] > WIDTH / 2) != depot
    name = ('Depot' if depot else 'HP') + NAMES[source['name']]
    def xy(x, y):
        return [x, WIDTH-y if mirrored else y]
    wps = []
    for i, point in enumerate(snapshot['waypoints']):
        x, y = xy(point['x'], point['y'])
        angle = ((-point['heading'] if mirrored else point['heading']) + 2*math.pi) % (2*math.pi) - math.pi
        if angle == 0:
            angle = 0.0  # JSON.stringify in the desktop app normalizes negative zero.
        wps.append({'id': f'w{i}', 'x': x, 'y': y, 'heading': angle,
                    'translationMode': 'fixed' if point['fixTranslation'] else 'guide',
                    'headingMode': 'fixed' if point['fixHeading'] else 'free',
                    'headingTolerance': 0, 'tolerance': {'kind': 'none', 'radius': .1, 'dx': .1, 'dy': .1},
                    'stop': False, 'split': point['split'],
                    'intervals': point['intervals'] if point['overrideIntervals'] else None,
                    'poseRef': None})
    def index(value):
        return 0 if value == 'first' else len(wps)-1 if value == 'last' else value
    cons = []
    for i, c in enumerate(snapshot['constraints']):
        if not c['enabled']:
            continue
        a, b = index(c['from']), index(c['to']) if c['to'] is not None else index(c['from'])
        kind, props = c['data']['type'], c['data']['props']
        scope = {'kind': 'waypoint' if c['to'] is None else 'range', 'from': a, 'to': b}
        if kind == 'StopPoint':
            wps[a]['stop'] = True
            continue
        if kind in ('MaxVelocity', 'MaxAcceleration', 'MaxAngularVelocity'):
            data = {'type': kind[0].lower()+kind[1:], 'value': props['max']}
        elif kind == 'KeepInLane':
            data = {'type': 'straightLine', 'tolerance': props['tolerance']}
        elif kind == 'KeepInRectangle':
            x, y, w, h = [props[k] for k in ('x', 'y', 'w', 'h')]
            data = {'type': 'keepIn', 'points': [xy(x,y),xy(x+w,y),xy(x+w,y+h),xy(x,y+h)]}
        elif kind == 'KeepOutCircle':
            # Circumscribe the circle rather than underestimating keep-out area.
            n = 48
            r = props['r']/math.cos(math.pi/n)
            data = {'type': 'keepOut', 'points': [xy(props['x']+r*math.cos(2*math.pi*j/n),
                        props['y']+r*math.sin(2*math.pi*j/n)) for j in range(n)], 'margin': 0}
        else:
            raise ValueError(f'Unsupported enabled Choreo constraint: {kind}')
        cons.append({'id': f'source-{i}', 'enabled': True, 'scope': scope, 'data': data})
    first = source['name'].endswith('firstleg')
    cons.append({'id': 'intake-extended', 'enabled': True,
                 'scope': {'kind': 'range', 'from': 1 if first else 0, 'to': len(wps)-1},
                 'data': {'type': 'intakeExtended'}})
    for obstacle in project['field']['obstacles']:
        if obstacle['id'].startswith('bump-'):
            cons.append({'id': 'terrain-'+obstacle['id'], 'enabled': True,
                         'scope': {'kind': 'zone', 'region': obstacle['points']},
                         'data': {'type': 'roughTerrain', 'expectedSpeed': .7, 'feedbackScale': .3}})
    return {'formatVersion': 1, 'name': name, 'waypoints': wps, 'constraints': cons, 'markers': [],
            'settings': {'targetDt': .08, 'smoothing': .02, 'candidates': 3, 'maxIterations': 1500, 'timeLimit': 60},
            'folder': '6800 - Top - Depot' if depot else '6800 - Bottom - HP Station', 'output': None}


def adapt_intake_clearance(trajectory, project, route_trench=True):
    """Keep source anchors visible; allow only necessary local clearance changes.

    Requires Shapely from the Mayhem solver environment. Collision obstacles and
    intake coverage stay enabled. Mayhem keep-out polygons apply globally.
    """
    from shapely.geometry import Point, Polygon, box, MultiPoint
    from shapely.ops import nearest_points, unary_union
    adaptations = []
    corridor_minimum = .30 if 'Trench' in trajectory['name'] else .10
    for constraint in trajectory['constraints']:
        if constraint['data']['type'] == 'straightLine' and constraint['data']['tolerance'] < corridor_minimum:
            adaptations.append({'constraint': constraint['id'], 'sourceTolerance': constraint['data']['tolerance'],
                                'tolerance': corridor_minimum, 'reason': 'corridor clearance for destination intake'})
            constraint['data']['tolerance'] = corridor_minimum
    obstacles = [(o['points'], o.get('margin', 0)) for o in project['field']['obstacles'] if o['enabled']]
    obstacles += [(c['data']['points'],c['data'].get('margin',0)) for c in trajectory['constraints']
                  if c['enabled'] and c['data']['type']=='keepOut']
    intake_start = next(c['scope']['from'] for c in trajectory['constraints'] if c['data']['type']=='intakeExtended')
    bumper=project['robot']['bumper']
    for i, waypoint in enumerate(trajectory['waypoints']):
        front=bumper['front']+(project['robot']['intake']['extension'] if i>=intake_start else 0)
        angle=waypoint['heading']
        footprint=[(a*math.cos(angle)-b*math.sin(angle),a*math.sin(angle)+b*math.cos(angle))
                   for a,b in [(front,bumper['left']),(front,-bumper['right']),
                               (-bumper['back'],bumper['left']),(-bumper['back'],-bumper['right'])]]
        margin=project['field']['wallMargin']
        allowed=box(margin-min(a for a,b in footprint),margin-min(b for a,b in footprint),
                    project['field']['length']-margin-max(a for a,b in footprint),
                    project['field']['width']-margin-max(b for a,b in footprint))
        forbidden=[]
        for vertices, clearance in obstacles:
            hull=MultiPoint([(x-a,y-b) for x,y in vertices for a,b in footprint]).convex_hull
            forbidden.append(hull.buffer(clearance+1e-4,join_style='mitre'))
        free=allowed.difference(unary_union(forbidden))
        point=Point(waypoint['x'],waypoint['y'])
        if not free.covers(point):
            distance=point.distance(nearest_points(point,free)[1])
            radius=math.ceil((distance+.04)*100)/100
            if radius>.45:
                raise ValueError(f'{trajectory["name"]} waypoint {i+1} needs excessive adjustment: {radius}')
            if waypoint['tolerance']['kind']=='circle':
                radius=max(radius,waypoint['tolerance']['radius'])
            waypoint['tolerance']={'kind':'circle','radius':radius,'dx':.1,'dy':.1}
            adaptations.append({'waypoint':i,'radius':radius,'minimumShift':distance,
                                'reason':'destination bumper/intake collision clearance'})
    if route_trench and trajectory['name'].endswith('TrenchSecondLeg'):
        departure = trajectory['waypoints'][1]
        original_departure = [departure['x'],departure['y'],departure['heading']]
        departure['y'] += .04 if trajectory['name'].startswith('Depot') else -.04
        adaptations.append({'waypoint':1,'sourcePose':original_departure,
                            'destinationPose':[departure['x'],departure['y'],departure['heading']],
                            'reason':'add 0.04 m post clearance at trench departure'})
        waypoint = trajectory['waypoints'][6]
        original = [waypoint['x'], waypoint['y'], waypoint['heading']]
        waypoint['y'] += .22 if trajectory['name'].startswith('Depot') else -.22
        waypoint['tolerance'] = {'kind':'none','radius':.1,'dx':.1,'dy':.1}
        adaptations.append({'waypoint':6,'sourcePose':original,
                            'destinationPose':[waypoint['x'],waypoint['y'],waypoint['heading']],
                            'reason':'shift return anchor 0.22 m toward wall to clear trench post with intake'})
        old = trajectory['waypoints']
        routed, mapping = [], {}
        for i, point in enumerate(old):
            mapping[i] = len(routed)
            routed.append(point)
            if i in (0,1,2,5,6):
                extra = copy.deepcopy(point)
                extra.update(id=f'clearance-{i}', x={0:old[0]['x'],1:5.65,2:6.2,5:5.75,6:old[-1]['x']}[i],
                             y=(7.4 if i==2 else 7.54) if trajectory['name'].startswith('Depot')
                               else WIDTH-(7.4 if i==2 else 7.54),
                             heading=math.pi/2 if trajectory['name'].startswith('Depot') else -math.pi/2,
                             stop=i in (0,2,6), split=False,
                             tolerance={'kind':'none','radius':.1,'dx':.1,'dy':.1})
                routed.append(extra)
        for constraint in trajectory['constraints']:
            if constraint['scope']['kind'] in ('range','waypoint'):
                for key in ('from','to'):
                    constraint['scope'][key]=mapping[constraint['scope'][key]]
        for adaptation in adaptations:
            if 'waypoint' in adaptation:
                adaptation['sourceWaypoint']=adaptation['waypoint']
                adaptation['waypoint']=mapping[adaptation['waypoint']]
        for i, point in enumerate(routed):
            if point['id'].startswith('clearance-'):
                adaptations.append({'waypoint':i,'destinationPose':[point['x'],point['y'],point['heading']],
                                    'reason':'added clearance anchor to route around trench post before turning/crossing'})
        trajectory['waypoints']=routed
    return adaptations


def main():
    checkout, output = map(Path, sys.argv[1:3])
    project = json.loads((output/'project.mayhem').read_text())
    used = set()
    for header in (checkout/'src/main/include/autos').rglob('*.h'):
        used.update(re.findall(r'LoadTrajectory<choreo::SwerveSample>\("([^"]+)"\)', header.read_text()))
    assert used == set(NAMES), f'Source inventory changed: {used ^ set(NAMES)}'
    inventory = []
    for stem in sorted(used):
        path = checkout/'src/main/deploy/choreo'/f'{stem}.traj'
        source = json.loads(path.read_text())
        for depot in (True, False):
            trajectory = convert(source, depot, project)
            adaptations = adapt_intake_clearance(trajectory, project)
            # Routing can introduce new anchors: check their front-intake footprint too.
            adaptations += adapt_intake_clearance(trajectory, project, route_trench=False)
            (output/f'{trajectory["name"]}.mtraj').write_text(json.dumps(trajectory, indent=1)+'\n')
            inventory.append({'name': trajectory['name'], 'folder': trajectory['folder'],
                              'source': str(path.relative_to(checkout)),
                              'adaptations': adaptations,
                              'snapshot': copy.deepcopy(source['snapshot'])})
    (output/'valor-manifest.json').write_text(json.dumps(inventory, indent=2)+'\n')
    print(f'Staged {len(inventory)} Valor paths')


if __name__ == '__main__':
    main()
