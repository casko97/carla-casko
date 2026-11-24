#!/usr/bin/env python3
"""Experimental persisted two-vehicle scenario checkpoint and replay.

Run capture and restore as separate processes on a dedicated CARLA server.
Only the fixed scripted scenario below is supported: this is not a world snapshot.
"""
import argparse
import base64
import hashlib
import json
import math
import os
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PROJECT = ROOT / 'deterministic-cpt'
ENGINE = ROOT.parent / 'carla-casko-engine'
sys.path.insert(0, str(PROJECT / 'PythonAPI/examples'))
import carla
import vehicle_physics_checkpoint as single
import checkpoint_msgpack_fields

SCHEMA = 'carla-scripted-scenario-checkpoint-v3'
LEGACY_SCHEMA = 'carla-scripted-scenario-checkpoint-v2'
WEATHER_FIELDS = ('cloudiness', 'precipitation', 'precipitation_deposits', 'wind_intensity',
                  'sun_azimuth_angle', 'sun_altitude_angle', 'fog_density', 'fog_distance',
                  'fog_falloff', 'wetness', 'scattering_intensity', 'mie_scattering_scale',
                  'rayleigh_scattering_scale', 'dust_storm')
CONTROL_FIELDS = ('throttle', 'steer', 'brake', 'hand_brake', 'reverse', 'manual_gear_shift', 'gear')
SPECS = {'ego': {'blueprint': 'vehicle.lincoln.mkz_2020', 'spawn_index': 0},
         'other': {'blueprint': 'vehicle.lincoln.mkz_2020', 'spawn_index': 1}}


def map_artifact_paths(map_name):
    """Selected files for locally available flat maps; reject names with paths."""
    if not isinstance(map_name, str) or re.fullmatch(r'[A-Za-z0-9_]+', map_name) is None:
        raise ValueError('map name must be a simple asset name')
    maps = PROJECT / 'Unreal/CarlaUE4/Content/Carla/Maps'
    paths = {'map': maps / (map_name + '.umap'),
             'map_built_data': maps / (map_name + '_BuiltData.uasset'),
             'opendrive': maps / 'OpenDrive' / (map_name + '.xodr'),
             'traffic_map': maps / 'TM' / (map_name + '.bin'),
             'nav': maps / 'Nav' / (map_name + '.bin')}
    if any(not path.is_file() for path in paths.values()):
        raise ValueError('map lacks selected flat-map artifacts: {}'.format(map_name))
    return paths


def fingerprint(map_name='Town01'):
    """Identity of selected on-disk artifacts, not all runtime modules/assets."""
    paths = {'editor': ENGINE / 'Engine/Binaries/Linux/UE4Editor',
             'server_plugin': PROJECT / 'Unreal/CarlaUE4/Plugins/Carla/Binaries/Linux/libUE4Editor-Carla.so',
             'client_egg': PROJECT / 'PythonAPI/carla/dist/carla-0.9.15-py3.8-linux-x86_64.egg'}
    paths.update(map_artifact_paths(map_name))
    result = {}
    for label, path in paths.items():
        before = path.stat()
        digest = hashlib.sha256()
        with path.open('rb') as source:
            for block in iter(lambda: source.read(1024 * 1024), b''):
                digest.update(block)
        after = path.stat()
        if (before.st_ino, before.st_size, before.st_mtime_ns) != (after.st_ino, after.st_size, after.st_mtime_ns):
            raise RuntimeError('artifact changed while hashing: {}'.format(path))
        result[label] = digest.hexdigest()
    build_id = hashlib.sha256(json.dumps(result, sort_keys=True).encode()).hexdigest()
    return {'sha256': result, 'selected_files_id': build_id,
            'scope': 'Selected on-disk files only, not all loaded modules, assets or server memory.'}


def schedule():
    """A fixed journal with actor creation and destruction at known frames."""
    frames = []
    for frame in range(80):
        events = []
        if frame == 20:
            events.append({'action': 'spawn', 'label': 'other'})
        if frame == 60:
            events.append({'action': 'destroy', 'label': 'other'})
        if frame == 65:
            events.append({'action': 'spawn', 'label': 'other'})
        controls = {'ego': {'throttle': 0.25, 'steer': 0.04}}
        if 20 <= frame < 60 or frame >= 65:
            controls['other'] = {'brake': 1.0}
        frames.append({'events': events, 'controls': controls})
    return frames


def spawn(world, spec):
    return single.spawn(world, spec['blueprint'], spec['spawn_index'])


def execute(world, client, actors, record):
    for event in record['events']:
        label = event['label']
        if event['action'] == 'destroy':
            if label not in actors or not actors[label].destroy():
                raise RuntimeError('destroy failed: {}'.format(label))
            del actors[label]
        elif event['action'] == 'spawn':
            if label in actors:
                raise RuntimeError('actor already exists: {}'.format(label))
            actors[label] = spawn(world, SPECS[label])
        else:
            raise ValueError('unsupported event')
    if set(record['controls']) != set(actors):
        raise RuntimeError('controls do not match live scenario actors')
    responses = client.apply_batch_sync([
        carla.command.ApplyVehicleControl(actors[label].id, carla.VehicleControl(**value))
        for label, value in record['controls'].items()], False)
    if len(responses) != len(actors) or any(response.error for response in responses):
        raise RuntimeError('control batch failed: {}'.format(responses))
    before = world.get_snapshot().frame
    if world.tick() != before + 1:
        raise RuntimeError('unexpected world tick')
    snapshot = world.get_snapshot()
    result = {}
    for label, actor in sorted(actors.items()):
        if snapshot.find(actor.id) is None:
            raise RuntimeError('actor missing from world: {}'.format(label))
        control = actor.get_control()
        state = single.sample(world, actor)
        if any(not math.isfinite(v) for values in state.values() for v in values):
            raise RuntimeError('non-finite actor state')
        result[label] = {'type': actor.type_id, 'control': {name: getattr(control, name) for name in CONTROL_FIELDS},
                         'state': state}
    if world.get_snapshot().frame != snapshot.frame:
        raise RuntimeError('world advanced during sampling')
    return result


def get_weather(world):
    weather = world.get_weather()
    return {name: getattr(weather, name) for name in WEATHER_FIELDS}


def sample_checkpoint(world, actors):
    """Read the latest client snapshot, which can predate a server-side restore."""
    frame = world.get_snapshot().frame
    states = {label: single.sample(world, actor) for label, actor in sorted(actors.items())}
    weather = get_weather(world)
    if world.get_snapshot().frame != frame:
        raise RuntimeError('world advanced during checkpoint sampling')
    return {'actors': states, 'weather': weather}


def atomic_create(path, value):
    # Exclusive creation protects existing experimental results.
    with open(path, 'x', encoding='utf-8') as output:
        json.dump(value, output, indent=2, sort_keys=True, allow_nan=False)
        output.write('\n')
        output.flush()
        os.fsync(output.fileno())


def capture(client, path, checkpoint_after):
    if Path(path).exists():
        raise FileExistsError(path)
    identity = fingerprint()
    world = single.load_world(client, 'Town01')
    actors = {}
    try:
        actors['ego'] = spawn(world, SPECS['ego'])
        frames = schedule()
        if not 1 <= checkpoint_after < len(frames):
            raise ValueError('checkpoint-after must leave a nonempty prefix and suffix')
        for frame in frames[:checkpoint_after]:
            execute(world, client, actors, frame)
        checkpoint_frame = world.get_snapshot().frame
        slots = {}
        checkpoint_mode = None
        for label, actor in sorted(actors.items()):
            if not actor.save_physics_checkpoint():
                raise RuntimeError('save rejected for {}'.format(label))
            data = actor.export_physics_checkpoint(identity['selected_files_id'])
            actor_mode = checkpoint_msgpack_fields.checkpoint_mode(data, identity['selected_files_id'])
            if checkpoint_mode is not None and checkpoint_mode != actor_mode:
                raise RuntimeError('vehicles reported different checkpoint modes')
            checkpoint_mode = actor_mode
            slots[label] = {'data_base64': base64.b64encode(data).decode('ascii'),
                            'sha256': hashlib.sha256(data).hexdigest(),
                            'attributes': dict(actor.attributes), 'spec': SPECS[label]}
            if world.get_snapshot().frame != checkpoint_frame:
                raise RuntimeError('world advanced during sequential checkpoint capture')
        weather = get_weather(world)
        boundary = sample_checkpoint(world, actors)
        suffix = [execute(world, client, actors, frame) for frame in frames[checkpoint_after:]]
        if fingerprint() != identity:
            raise RuntimeError('selected artifacts changed during capture')
        bundle = {'schema': SCHEMA, 'identity': identity, 'map': 'Town01',
                  'checkpoint_after': checkpoint_after, 'checkpoint_mode': checkpoint_mode,
                  'origin_assumption': 'source and target origin are zero; not independently verified',
                  'weather': weather, 'actors': slots, 'suffix_journal': frames[checkpoint_after:],
                  'expected_boundary': boundary,
                  'expected_suffix': suffix,
                  'limitations': ['manual controls only', 'sequential capture is not atomic',
                                  'no sensors, traffic manager, scenario controllers or physics scene cache']}
        atomic_create(path, bundle)
        return {'saved': str(path), 'checkpoint_after': checkpoint_after, 'actors': sorted(slots), 'suffix_frames': len(suffix)}
    finally:
        for actor in actors.values():
            actor.destroy()


def restore(client, path, report_path):
    if Path(report_path).exists():
        raise FileExistsError(report_path)
    bundle = json.loads(Path(path).read_text())
    if bundle.get('schema') not in (SCHEMA, LEGACY_SCHEMA) or bundle.get('map') != 'Town01':
        raise ValueError('unsupported checkpoint schema or map')
    checkpoint_after = bundle.get('checkpoint_after')
    frames = schedule()
    if (type(checkpoint_after) is not int or not 1 <= checkpoint_after < len(frames) or
            (bundle['schema'] == LEGACY_SCHEMA and checkpoint_after != 40)):
        raise ValueError('invalid checkpoint boundary')
    if bundle.get('identity') != fingerprint():
        raise ValueError('selected on-disk artifacts do not match the capture')
    expected_actors = set(frames[checkpoint_after - 1]['controls'])
    if set(bundle.get('actors', {})) != expected_actors or bundle.get('suffix_journal') != frames[checkpoint_after:]:
        raise ValueError('checkpoint is not the supported scripted scenario')
    if len(bundle.get('expected_suffix', [])) != len(bundle['suffix_journal']):
        raise ValueError('baseline and journal lengths differ')
    if not isinstance(bundle.get('expected_boundary'), dict):
        raise ValueError('checkpoint boundary observation missing')
    decoded = {}
    for label, slot in bundle['actors'].items():
        if slot.get('spec') != SPECS[label]:
            raise ValueError('actor spec mismatch: {}'.format(label))
        data = base64.b64decode(slot['data_base64'], validate=True)
        if not 0 < len(data) <= 1024 * 1024 or hashlib.sha256(data).hexdigest() != slot['sha256']:
            raise ValueError('invalid actor checkpoint: {}'.format(label))
        mode = checkpoint_msgpack_fields.checkpoint_mode(data, bundle['identity']['selected_files_id'])
        if 'checkpoint_mode' in bundle and mode != bundle['checkpoint_mode']:
            raise ValueError('checkpoint mode mismatch: {}'.format(label))
        decoded[label] = data
    world = single.load_world(client, bundle['map'])
    actors = {}
    result = {'schema': bundle['schema'], 'checkpoint_after': checkpoint_after,
              'checkpoint_mode': bundle.get('checkpoint_mode', 'not recorded in bundle'),
              'exact': False, 'first_difference': None,
              'scope': 'sampled vehicle states and observed controls; not full simulator state',
              'suffix': [], 'failure': None}
    try:
        for label in sorted(expected_actors):
            actors[label] = spawn(world, SPECS[label])
            if dict(actors[label].attributes) != bundle['actors'][label]['attributes']:
                raise RuntimeError('actor attributes mismatch: {}'.format(label))
        # Let newly created physics bodies initialize, then install and apply all slots.
        for _ in range(20):
            controls = {'events': [], 'controls': {label: {'brake': 1.0} for label in actors}}
            execute(world, client, actors, controls)
        world.set_weather_synchronously(carla.WeatherParameters(**bundle['weather']))
        frame = world.get_snapshot().frame
        for label, actor in sorted(actors.items()):
            if not actor.import_physics_checkpoint(decoded[label], bundle['identity']['selected_files_id'], True):
                raise RuntimeError('import rejected: {}'.format(label))
            if not actor.restore_physics_checkpoint():
                raise RuntimeError('restore rejected: {}'.format(label))
            if world.get_snapshot().frame != frame:
                raise RuntimeError('world advanced during sequential restore')
        boundary = sample_checkpoint(world, actors)
        result['boundary_observation'] = {
            'matches': boundary == bundle['expected_boundary'],
            'expected': bundle['expected_boundary'], 'actual': boundary,
            'limitation': 'Client world snapshot can predate restore; mismatch does not prove server state differs.'}
        # Recapture the live server-side state rather than relying on the last
        # streamed snapshot. Save replaces the actor's slot; reinstall the
        # original bytes so subsequent restores retain their original meaning.
        result['server_boundary'] = {}
        for label, actor in sorted(actors.items()):
            if not actor.save_physics_checkpoint():
                raise RuntimeError('server boundary capture rejected: {}'.format(label))
            try:
                observed = actor.export_physics_checkpoint(bundle['identity']['selected_files_id'])
                differences = checkpoint_msgpack_fields.compare(decoded[label], observed)
                result['server_boundary'][label] = {'differences': differences,
                    'scope': 'Encoded portable vehicle fields except capture frame; not full physics scene state.'}
            finally:
                if not actor.import_physics_checkpoint(decoded[label], bundle['identity']['selected_files_id'], True):
                    raise RuntimeError('failed to reinstall original slot: {}'.format(label))
            if world.get_snapshot().frame != frame:
                raise RuntimeError('world advanced during boundary recapture')
        for index, command in enumerate(bundle['suffix_journal']):
            actual = execute(world, client, actors, command)
            result['suffix'].append(actual)
            expected = bundle['expected_suffix'][index]
            if result['first_difference'] is None and actual != expected:
                result['first_difference'] = {'suffix_frame': index + 1, 'expected': expected, 'actual': actual}
        if fingerprint() != bundle['identity']:
            raise RuntimeError('selected artifacts changed during restore')
        result['exact'] = result['first_difference'] is None
        return result
    except Exception as exc:
        result['failure'] = {'type': type(exc).__name__, 'message': str(exc)}
        raise
    finally:
        try:
            for actor in actors.values():
                actor.destroy()
        finally:
            atomic_create(report_path, result)


def replay_prefix(client, path, report_path):
    """Reconstruct the fixed scenario by replaying its prefix from the beginning."""
    if Path(report_path).exists():
        raise FileExistsError(report_path)
    bundle = json.loads(Path(path).read_text())
    frames = schedule()
    count = bundle.get('checkpoint_after')
    if (bundle.get('schema') not in (SCHEMA, LEGACY_SCHEMA) or bundle.get('map') != 'Town01' or
            type(count) is not int or not 1 <= count < len(frames) or
            (bundle['schema'] == LEGACY_SCHEMA and count != 40) or
            bundle.get('suffix_journal') != frames[count:] or
            len(bundle.get('expected_suffix', [])) != len(frames) - count):
        raise ValueError('unsupported checkpoint or scripted journal')
    if bundle.get('identity') != fingerprint():
        raise ValueError('selected on-disk artifacts do not match the capture')
    world = single.load_world(client, bundle['map'])
    actors = {}
    result = {'schema': bundle['schema'], 'mode': 'full_prefix_replay', 'checkpoint_after': count,
              'scope': 'sampled vehicle states and observed controls; not full simulator state',
              'exact': False, 'first_difference': None, 'suffix': [], 'failure': None}
    try:
        world.set_weather_synchronously(carla.WeatherParameters(**bundle['weather']))
        actors['ego'] = spawn(world, SPECS['ego'])
        if 'checkpoint_mode' in bundle:
            if not actors['ego'].save_physics_checkpoint():
                raise RuntimeError('could not verify replay checkpoint mode')
            current = actors['ego'].export_physics_checkpoint(bundle['identity']['selected_files_id'])
            if checkpoint_msgpack_fields.checkpoint_mode(current, bundle['identity']['selected_files_id']) != bundle['checkpoint_mode']:
                raise RuntimeError('replay checkpoint mode mismatch')
        for command in frames[:count]:
            execute(world, client, actors, command)
        boundary = sample_checkpoint(world, actors)
        result['boundary_matches'] = boundary == bundle.get('expected_boundary')
        for index, command in enumerate(frames[count:]):
            actual = execute(world, client, actors, command)
            result['suffix'].append(actual)
            if result['first_difference'] is None and actual != bundle['expected_suffix'][index]:
                result['first_difference'] = {'suffix_frame': index + 1,
                    'expected': bundle['expected_suffix'][index], 'actual': actual}
        if fingerprint() != bundle['identity']:
            raise RuntimeError('selected artifacts changed during replay')
        result['exact'] = result['boundary_matches'] and result['first_difference'] is None
        return result
    except Exception as exc:
        result['failure'] = {'type': type(exc).__name__, 'message': str(exc)}
        raise
    finally:
        try:
            for actor in actors.values():
                actor.destroy()
        finally:
            atomic_create(report_path, result)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('operation', choices=('capture', 'restore', 'replay-prefix'))
    parser.add_argument('--checkpoint', required=True)
    parser.add_argument('--report', help='required for restore')
    parser.add_argument('--checkpoint-after', type=int, default=40,
                        help='completed journal frames before capture (1–79; default: 40)')
    parser.add_argument('--port', type=int, default=22000)
    parser.add_argument('--timeout', type=float, default=180)
    args = parser.parse_args()
    if args.operation != 'capture' and not args.report:
        parser.error('restore and replay-prefix require --report')
    if not 1 <= args.checkpoint_after < len(schedule()):
        parser.error('--checkpoint-after must be between 1 and 79')
    client = carla.Client('127.0.0.1', args.port)
    client.set_timeout(args.timeout)
    settings = client.get_world().get_settings()
    try:
        if args.operation == 'capture':
            result = capture(client, args.checkpoint, args.checkpoint_after)
        elif args.operation == 'restore':
            result = restore(client, args.checkpoint, args.report)
        else:
            result = replay_prefix(client, args.checkpoint, args.report)
        print(json.dumps({key: value for key, value in result.items() if key != 'suffix'}, indent=2))
        return 0 if args.operation == 'capture' or result['exact'] else 1
    finally:
        client.get_world().apply_settings(settings)


if __name__ == '__main__':
    sys.exit(main())
