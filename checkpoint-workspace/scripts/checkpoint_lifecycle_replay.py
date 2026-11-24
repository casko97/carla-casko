#!/usr/bin/env python3
"""Compare two deterministic lifecycle journals on a dedicated CARLA server.

This is a full-prefix replay reference, not a world checkpoint restore.
"""
import argparse
import json
import math
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT / 'deterministic-cpt/PythonAPI/examples'))
import carla
import vehicle_physics_checkpoint as single

FIELDS = ('throttle', 'steer', 'brake', 'hand_brake', 'reverse', 'manual_gear_shift', 'gear')


def sample_control(actor):
    control = actor.get_control()
    return {name: getattr(control, name) for name in FIELDS}


def run(client, run_index):
    world = single.load_world(client, 'Town01')
    actors = {}
    journal = []
    trace = []
    try:
        actors['ego'] = single.spawn(world, 'vehicle.lincoln.mkz_2020', 0)
        journal.append({'frame': 0, 'event': 'spawn', 'actor': 'ego', 'spawn_index': 0})
        for frame in range(80):
            if frame == 20:
                actors['other'] = single.spawn(world, 'vehicle.lincoln.mkz_2020', 1)
                journal.append({'frame': frame, 'event': 'spawn', 'actor': 'other', 'spawn_index': 1})
            if frame == 60:
                if not actors['other'].destroy():
                    raise RuntimeError('failed to destroy other actor')
                del actors['other']
                journal.append({'frame': frame, 'event': 'destroy', 'actor': 'other'})
            controls = {'ego': {'throttle': 0.25, 'steer': 0.04}}
            if 'other' in actors:
                controls['other'] = {'brake': 1.0}
            commands = [carla.command.ApplyVehicleControl(actors[label].id, carla.VehicleControl(**control))
                        for label, control in controls.items()]
            responses = client.apply_batch_sync(commands, False)
            if len(responses) != len(commands) or any(r.error for r in responses):
                raise RuntimeError('control batch failed: {}'.format(responses))
            before = world.get_snapshot().frame
            if world.tick() != before + 1:
                raise RuntimeError('unexpected tick; a second tick owner may exist')
            snapshot = world.get_snapshot()
            frame_data = {}
            for label, actor in sorted(actors.items()):
                if snapshot.find(actor.id) is None:
                    raise RuntimeError('actor disappeared: {}'.format(label))
                values = single.sample(world, actor)
                if any(not math.isfinite(v) for vector in values.values() for v in vector):
                    raise RuntimeError('non-finite actor state')
                frame_data[label] = {'type': actor.type_id, 'controller': 'manual',
                                     'observed_control': sample_control(actor), 'state': values}
            if world.get_snapshot().frame != snapshot.frame:
                raise RuntimeError('world advanced during observation')
            trace.append(frame_data)
        return {'run': run_index, 'journal': journal, 'trace': trace}
    finally:
        for actor in actors.values():
            actor.destroy()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', type=int, default=22000)
    p.add_argument('--timeout', type=float, default=180)
    p.add_argument('--report', required=True)
    args = p.parse_args()
    client = carla.Client('127.0.0.1', args.port)
    client.set_timeout(args.timeout)
    original_settings = client.get_world().get_settings()
    report = {'contract': 'full-prefix replay; not checkpoint restoration', 'runs': [],
              'match': None, 'first_difference': None, 'failure': None}
    try:
        for i in range(2):
            report['runs'].append(run(client, i))
        a, b = report['runs']
        report['match'] = a['journal'] == b['journal'] and a['trace'] == b['trace']
        for i, (x, y) in enumerate(zip(a['trace'], b['trace'])):
            if x != y:
                report['first_difference'] = {'frame': i, 'first': x, 'replay': y}
                break
        print(json.dumps({'match': report['match'], 'first_difference': report['first_difference']}, indent=2))
        return 0 if report['match'] else 1
    except Exception as exc:
        report['failure'] = {'type': type(exc).__name__, 'message': str(exc)}
        raise
    finally:
        try:
            client.get_world().apply_settings(original_settings)
        finally:
            Path(args.report).write_text(json.dumps(report, indent=2, sort_keys=True) + '\n')


if __name__ == '__main__':
    sys.exit(main())
