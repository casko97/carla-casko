#!/usr/bin/env python3
"""Persist and replay a journal of manually controlled Town01 vehicles.

This experimental harness uses a dedicated CARLA server. It does not capture
Traffic Manager, sensors, pedestrians, controller state, or PhysX scene caches.
"""
import argparse
import base64
import hashlib
import json
import math
from pathlib import Path

import carla
import checkpoint_msgpack_fields as fields
import checkpoint_vehicle_light_policy as light_policy
import checkpoint_scenario_bundle as common
import multi_vehicle_physics_checkpoint as multi
import vehicle_physics_checkpoint as single

SCHEMA = 'carla-manual-vehicle-journal-checkpoint-v1'


def validate_journal(journal, boundary):
    if not isinstance(journal, dict) or set(journal) != {'map', 'actors', 'initial_actors', 'frames'}:
        raise ValueError('journal requires map, actors, initial_actors and frames')
    common.map_artifact_paths(journal['map'])
    actors, initial, frames = journal['actors'], journal['initial_actors'], journal['frames']
    if not isinstance(actors, list) or not 1 <= len(actors) <= 64:
        raise ValueError('actors must contain 1 to 64 definitions')
    if not isinstance(frames, list) or not 2 <= len(frames) <= 10000:
        raise ValueError('frames must contain 2 to 10000 records')
    if type(boundary) is not int or not 1 <= boundary < len(frames):
        raise ValueError('checkpoint boundary must leave a nonempty prefix and suffix')
    # Reuse the existing vehicle specification and attribute validation.
    if any(not isinstance(a, dict) or a.get('controller', 'manual') != 'manual' for a in actors):
        raise ValueError('only manually controlled vehicles are supported')
    ids = [a.get('id') for a in actors]
    if any(not isinstance(label, str) or not label for label in ids) or len(set(ids)) != len(ids):
        raise ValueError('actor IDs must be unique nonempty strings')
    dummy = {label: {} for label in ids}
    multi.validate_scenario({'map': journal['map'], 'actors': actors, 'frames': [dummy, dummy]}, 1)
    if (not isinstance(initial, list) or not initial or any(not isinstance(x, str) for x in initial) or
            len(set(initial)) != len(initial) or not set(initial) <= set(ids)):
        raise ValueError('initial_actors must be a nonempty list of distinct defined IDs')
    alive = set(initial)
    at_boundary = None
    for index, frame in enumerate(frames):
        if not isinstance(frame, dict) or set(frame) != {'events', 'controls'}:
            raise ValueError('frame {} must contain events and controls'.format(index))
        if not isinstance(frame['events'], list) or not isinstance(frame['controls'], dict):
            raise ValueError('invalid events or controls at frame {}'.format(index))
        for event in frame['events']:
            if not isinstance(event, dict) or set(event) != {'action', 'label'}:
                raise ValueError('invalid lifecycle event at frame {}'.format(index))
            label, action = event['label'], event['action']
            if label not in ids or action not in ('spawn', 'destroy'):
                raise ValueError('unknown lifecycle event at frame {}'.format(index))
            if action == 'spawn':
                if label in alive:
                    raise ValueError('spawn of live actor {} at frame {}'.format(label, index))
                alive.add(label)
            else:
                if label not in alive:
                    raise ValueError('destroy of absent actor {} at frame {}'.format(label, index))
                alive.remove(label)
        if not alive or set(frame['controls']) != alive:
            raise ValueError('frame {} must control exactly the nonempty live actor set'.format(index))
        for label, control in frame['controls'].items():
            if not isinstance(control, dict) or set(control) - set(common.CONTROL_FIELDS):
                raise ValueError('invalid control for {} at frame {}'.format(label, index))
            for key, value in control.items():
                if key in ('hand_brake', 'reverse', 'manual_gear_shift'):
                    valid = type(value) is bool
                elif key == 'gear':
                    valid = type(value) is int and -(2**31) <= value < 2**31
                else:
                    valid = type(value) in (int, float) and math.isfinite(value) and (-1.0 if key == 'steer' else 0.0) <= value <= 1.0
                if not valid:
                    raise ValueError('invalid {} for {} at frame {}'.format(key, label, index))
        if index + 1 == boundary:
            at_boundary = set(alive)
    return {a['id']: a for a in actors}, at_boundary


def spawn(world, label, specs):
    return multi.spawn_actors(world, [specs[label]])[label]


def execute(world, client, actors, frame, specs, vehicle_light_policy=None):
    for event in frame['events']:
        label = event['label']
        if event['action'] == 'spawn':
            actors[label] = spawn(world, label, specs)
        else:
            if not actors[label].destroy():
                raise RuntimeError('failed to destroy {}'.format(label))
            del actors[label]
    light_policy.apply(client, world, actors, frame['controls'], vehicle_light_policy)
    # common.execute samples the same normalized physical fields and controls.
    return common.execute(world, client, actors, {'events': [], 'controls': frame['controls']})


def create_bundle(client, journal, boundary, path):
    specs, expected_alive = validate_journal(journal, boundary)
    if Path(path).exists():
        raise FileExistsError(path)
    identity = common.fingerprint(journal['map'])
    world = single.load_world(client, journal['map'])
    actors = {}
    try:
        for label in journal['initial_actors']:
            actors[label] = spawn(world, label, specs)
        for frame in journal['frames'][:boundary]:
            execute(world, client, actors, frame, specs)
        if set(actors) != expected_alive:
            raise RuntimeError('live actors differ from validated journal')
        frame_id, slots, mode = world.get_snapshot().frame, {}, None
        for label, actor in sorted(actors.items()):
            if not actor.save_physics_checkpoint():
                raise RuntimeError('save rejected for {}'.format(label))
            data = actor.export_physics_checkpoint(identity['selected_files_id'])
            actor_mode = fields.checkpoint_mode(data, identity['selected_files_id'])
            if mode is not None and mode != actor_mode:
                raise RuntimeError('checkpoint modes differ between actors')
            mode = actor_mode
            slots[label] = {'data_base64': base64.b64encode(data).decode('ascii'),
                            'sha256': hashlib.sha256(data).hexdigest(), 'attributes': dict(actor.attributes)}
            if world.get_snapshot().frame != frame_id:
                raise RuntimeError('world advanced during capture')
        observation = common.sample_checkpoint(world, actors)
        suffix = [execute(world, client, actors, frame, specs) for frame in journal['frames'][boundary:]]
        if common.fingerprint(journal['map']) != identity:
            raise RuntimeError('selected artifacts changed during capture')
        bundle = {'schema': SCHEMA, 'journal': journal, 'checkpoint_after': boundary,
                  'checkpoint_mode': mode, 'identity': identity, 'actors': slots,
                  'expected_boundary': observation, 'expected_suffix': suffix,
                  'origin_assumption': 'source and target origin are zero; not independently verified',
                  'limitations': ['manual vehicle controls only', 'sequential capture is not atomic',
                                  'no sensors, traffic manager, scenario controllers, or physics scene cache']}
        common.atomic_create(path, bundle)
        return {'saved': str(path), 'actors': sorted(slots), 'checkpoint_mode': mode, 'suffix_frames': len(suffix)}
    finally:
        for actor in actors.values():
            actor.destroy()


def load_bundle(path):
    bundle = json.loads(Path(path).read_text())
    if bundle.get('schema') != SCHEMA:
        raise ValueError('unsupported journal checkpoint')
    journal, boundary = bundle['journal'], bundle['checkpoint_after']
    specs, alive = validate_journal(journal, boundary)
    if bundle.get('identity') != common.fingerprint(journal['map']):
        raise ValueError('selected on-disk artifacts do not match capture')
    if set(bundle.get('actors', {})) != alive or len(bundle.get('expected_suffix', [])) != len(journal['frames']) - boundary:
        raise ValueError('checkpoint actors or suffix do not match journal')
    decoded = {}
    for label, slot in bundle['actors'].items():
        data = base64.b64decode(slot['data_base64'], validate=True)
        if not 0 < len(data) <= 1024 * 1024 or hashlib.sha256(data).hexdigest() != slot['sha256']:
            raise ValueError('invalid checkpoint data for {}'.format(label))
        if fields.checkpoint_mode(data, bundle['identity']['selected_files_id']) != bundle['checkpoint_mode']:
            raise ValueError('checkpoint mode mismatch for {}'.format(label))
        decoded[label] = data
    return bundle, specs, alive, decoded


def replay(client, path, report_path, mode):
    if Path(report_path).exists():
        raise FileExistsError(report_path)
    bundle, specs, alive, decoded = load_bundle(path)
    journal, boundary = bundle['journal'], bundle['checkpoint_after']
    world = single.load_world(client, journal['map'])
    actors = {}
    result = {'schema': SCHEMA, 'mode': mode, 'checkpoint_mode': bundle['checkpoint_mode'],
              'checkpoint_after': boundary, 'exact': False, 'first_difference': None,
              'scope': 'sampled vehicle states and observed controls; not full simulator state',
              'suffix': [], 'failure': None}
    try:
        if mode == 'restore':
            for label in sorted(alive):
                actors[label] = spawn(world, label, specs)
                if dict(actors[label].attributes) != bundle['actors'][label]['attributes']:
                    raise RuntimeError('actor attributes differ for {}'.format(label))
            for _ in range(20):
                common.execute(world, client, actors, {'events': [], 'controls': {label: {'brake': 1.0} for label in actors}})
            world.set_weather_synchronously(carla.WeatherParameters(**bundle['expected_boundary']['weather']))
            frame_id = world.get_snapshot().frame
            for label, actor in sorted(actors.items()):
                if not actor.import_physics_checkpoint(decoded[label], bundle['identity']['selected_files_id'], True):
                    raise RuntimeError('import rejected for {}'.format(label))
                if not actor.restore_physics_checkpoint():
                    raise RuntimeError('restore rejected for {}'.format(label))
                if world.get_snapshot().frame != frame_id:
                    raise RuntimeError('world advanced during restore')
        else:
            world.set_weather_synchronously(carla.WeatherParameters(**bundle['expected_boundary']['weather']))
            for label in journal['initial_actors']:
                actors[label] = spawn(world, label, specs)
            first = actors[journal['initial_actors'][0]]
            if not first.save_physics_checkpoint():
                raise RuntimeError('could not verify replay checkpoint mode')
            current = first.export_physics_checkpoint(bundle['identity']['selected_files_id'])
            if fields.checkpoint_mode(current, bundle['identity']['selected_files_id']) != bundle['checkpoint_mode']:
                raise RuntimeError('replay checkpoint mode mismatch')
            for frame in journal['frames'][:boundary]:
                execute(world, client, actors, frame, specs)
            result['boundary_matches'] = common.sample_checkpoint(world, actors) == bundle['expected_boundary']
        for index, frame in enumerate(journal['frames'][boundary:]):
            actual = execute(world, client, actors, frame, specs)
            result['suffix'].append(actual)
            if result['first_difference'] is None and actual != bundle['expected_suffix'][index]:
                result['first_difference'] = {'suffix_frame': index + 1,
                    'expected': bundle['expected_suffix'][index], 'actual': actual}
        if common.fingerprint(journal['map']) != bundle['identity']:
            raise RuntimeError('selected artifacts changed during replay')
        result['exact'] = result['first_difference'] is None and result.get('boundary_matches', True)
        return result
    except Exception as exc:
        result['failure'] = {'type': type(exc).__name__, 'message': str(exc)}
        raise
    finally:
        try:
            for actor in actors.values():
                actor.destroy()
        finally:
            common.atomic_create(report_path, result)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('operation', choices=('capture', 'restore', 'replay-prefix'))
    parser.add_argument('--journal', help='required for capture')
    parser.add_argument('--checkpoint', required=True)
    parser.add_argument('--checkpoint-after', type=int, help='completed journal frames before capture')
    parser.add_argument('--report', help='required for restore and replay-prefix')
    parser.add_argument('--port', type=int, default=22000)
    parser.add_argument('--timeout', type=float, default=180)
    args = parser.parse_args()
    if args.operation == 'capture':
        if not args.journal or args.checkpoint_after is None:
            parser.error('capture requires --journal and --checkpoint-after')
        journal = json.loads(Path(args.journal).read_text())
        validate_journal(journal, args.checkpoint_after)
    elif not args.report:
        parser.error('restore and replay-prefix require --report')
    client = carla.Client('127.0.0.1', args.port)
    client.set_timeout(args.timeout)
    settings = client.get_world().get_settings()
    try:
        if args.operation == 'capture':
            result = create_bundle(client, journal, args.checkpoint_after, args.checkpoint)
        else:
            result = replay(client, args.checkpoint, args.report, args.operation)
        print(json.dumps({k: v for k, v in result.items() if k != 'suffix'}, indent=2))
        return 0 if args.operation == 'capture' or result['exact'] else 1
    finally:
        client.get_world().apply_settings(settings)


if __name__ == '__main__':
    raise SystemExit(main())
