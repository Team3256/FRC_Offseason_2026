"""2910 source parsing helpers for tools/split_target_autos.py.

These helpers feed the internal arrival-region reconstruction.
Use group_imported_autos.py for the final complete cycle options.
"""
import json
import math
from pathlib import Path
import re
import sys

from import_team581_comp_paths import arguments, closing, number


class Resolver:
    def __init__(self, text):
        self.poses = {}
        self.field = json.loads((Path(__file__).resolve().parents[1]/
            'src/main/deploy/apriltags/welded/2026-official.json').read_text())['field']
        self.getters = dict(re.findall(
            r'public static Pose2d (\w+)\([^)]*\)\s*\{\s*return alliance == DriverStation.Alliance.Blue\s*\?\s*(\w+)', text))
        for name, expression in re.findall(r'private static final Pose2d (\w+)\s*=\s*([^;]+);', text):
            if '_BLUE' in name:
                self.poses[name] = self.resolve(expression)

    def resolve(self, expression):
        expression = re.sub(r'\s+', '', expression)
        if expression in self.poses:
            return self.poses[expression]
        getter = re.fullmatch(r'FieldConstants\.(\w+)\(alliance\)', expression)
        if getter:
            return self.poses[self.getters[getter[1]]]
        accessor = re.fullmatch(r'(\w+|FieldConstants\.\w+\(alliance\))\.(getX|getY|getTranslation|getRotation)\(\)', expression)
        if accessor:
            value = self.resolve(accessor[1])
            return {'getX': value[0], 'getY': value[1], 'getTranslation': value[:2], 'getRotation': value[2]}[accessor[2]]
        times = re.fullmatch(r'(.*)\.times\(([^()]*)\)', expression)
        if times:
            return self.resolve(times[1])*float(times[2])
        if expression.startswith('newPose2d('):
            args = arguments(expression[len('newPose2d('):-1])
            if len(args) == 2:
                return [*self.resolve(args[0]), self.resolve(args[1])]
            return [self.resolve(v) for v in args]
        angle = re.fullmatch(r'Rotation2d.fromDegrees\(([^()]*)\)', expression)
        if angle:
            return math.radians(float(angle[1]))
        if expression in ('Rotation2d.kZero', 'Rotation2d.k180deg'):
            return 0.0 if expression.endswith('kZero') else math.pi
        # Scalar expressions reference previously defined X/Y coordinates.
        expression = re.sub(r'(\w+)\.(getX|getY)\(\)',
                            lambda m: repr(self.poses[m[1]][0 if m[2]=='getX' else 1]), expression)
        return number(expression, {'FIELD_WIDTH': self.field['width'], 'FIELD_LENGTH': self.field['length']})


def route_calls(body, resolver):
    events = []
    for match in re.finditer(r'driveToPoint(?:WhileIntaking)?\(|scoreStationary\(\)', body):
        if match[0] == 'scoreStationary()':
            events.append(None)
            continue
        opening = match.end()-1
        end = closing(body, opening)
        args = arguments(body[opening+1:end])
        speed = {'DTP_FAST_VELOCITY_METERS': 4.572, 'DTP_MEDIUM_VELOCITY_METERS': 3.3528,
                 'DTP_SLOW_VELOCITY_METERS': 1.524}.get(args[1], None)
        speed = float(args[1]) if speed is None else speed
        # 0.01 m/s followed by a 0.5-second race is a timed hold request, not
        # a path section that should be traversed at one centimeter per second.
        hold = speed == .01 and '.raceWith(Commands.waitSeconds(0.5))' in re.sub(r'\s+', '', body[end+1:end+110])
        events.append({'pose': resolver.resolve(args[0]), 'speed': speed,
                       'tolerance': float(args[2]) if len(args)>2 else .1524,
                       'hold': hold, 'sourceCall': re.sub(r'\s+', ' ', body[match.start():end+1])})
    return events



if __name__ == "__main__":
    raise SystemExit("Use tools/split_target_autos.py: exact-target bundled imports discard the source arrival thresholds.")
