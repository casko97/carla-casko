#!/usr/bin/env python3
"""Probe a scripted checkpoint without replacing its actors or reloading the map.

Use a dedicated server: this loads a map, changes settings and destroys actors.
The chosen suffix must contain no lifecycle events so the saved actors survive.
"""
import argparse
import base64
import json
import math
from pathlib import Path

import carla
import checkpoint_msgpack_fields as fields
import checkpoint_scenario_bundle as scenario
import multi_vehicle_physics_checkpoint as multi
import vehicle_physics_checkpoint as single


def run(client, checkpoint_after, suffix_frames, repetitions, report_path, exclude_other=False,
        immediate_reference=None, advance_before_restore=0, spawn_z_offset=0.0, collision_events=False,
        physics_trace=False):
    frames = scenario.schedule()
    if not math.isfinite(spawn_z_offset) or spawn_z_offset < 0 or (spawn_z_offset and immediate_reference):
        raise ValueError('invalid spawn offset or incompatible reference')
    if collision_events and (not exclude_other or immediate_reference):
        raise ValueError('collision probe currently requires --exclude-other and no external reference')
    if physics_trace and immediate_reference:
        raise ValueError('physics trace requires a local baseline')
    if exclude_other:
        frames = [{'events': [event for event in frame['events'] if event['label'] != 'other'],
                   'controls': {label: control for label, control in frame['controls'].items() if label != 'other'}}
                  for frame in frames]
    if not 1 <= checkpoint_after < len(frames) or not 1 <= suffix_frames <= len(frames) - checkpoint_after:
        raise ValueError('invalid checkpoint or suffix length')
    suffix = frames[checkpoint_after:checkpoint_after + suffix_frames]
    if (advance_before_restore < 0 or advance_before_restore > len(frames) - checkpoint_after or
            (advance_before_restore and not immediate_reference) or
            any(record['events'] for record in frames[checkpoint_after:checkpoint_after + advance_before_restore])):
        raise ValueError('invalid advance-before-restore for this probe')
    if any(record['events'] for record in suffix):
        raise ValueError('same-episode probe requires a suffix without lifecycle events')
    if Path(report_path).exists():
        raise FileExistsError(report_path)
    identity = scenario.fingerprint()
    reference = None
    if immediate_reference:
        reference = json.loads(Path(immediate_reference).read_text())
        if (exclude_other or repetitions != 1 or reference.get('schema') != scenario.SCHEMA or
                reference.get('checkpoint_after') != checkpoint_after or reference.get('identity') != identity or
                reference.get('suffix_journal') != frames[checkpoint_after:] or
                len(reference.get('expected_suffix', [])) != len(frames) - checkpoint_after):
            raise ValueError('incompatible immediate-restore reference')
    world = single.load_world(client, 'Town01')
    actors = {}
    sensors = {}
    report = {'checkpoint_after': checkpoint_after, 'suffix_frames': suffix_frames, 'exclude_other': exclude_other,
              'spawn_z_offset_m': spawn_z_offset,
              'collision_events': collision_events, 'prefix_collision_events': [], 'baseline_collision_events': [],
              'physics_trace': physics_trace, 'baseline_physics': [],
              'external_reference': reference is not None, 'advance_before_restore': advance_before_restore,
              'identity': identity, 'baseline': [], 'restores': [], 'failure': None,
              'scope': 'same episode; sampled vehicle state and controls, not full simulator state'}
    try:
        if spawn_z_offset:
            spec = scenario.SPECS['ego']
            blueprint = world.get_blueprint_library().find(spec['blueprint'])
            if blueprint.has_attribute('sticky_control'):
                blueprint.set_attribute('sticky_control', 'false')
            transform = world.get_map().get_spawn_points()[spec['spawn_index']]
            transform.location.z += spawn_z_offset
            actors['ego'] = world.try_spawn_actor(blueprint, transform)
            if actors['ego'] is None:
                del actors['ego']
                raise RuntimeError('elevated ego spawn failed')
        else:
            actors['ego'] = scenario.spawn(world, scenario.SPECS['ego'])
        if collision_events:
            sensors = multi.setup_collision_logs(world, actors)

        def step(record, event_trace):
            state = scenario.execute(world, client, actors, record)
            if collision_events:
                event_trace.append(multi.drain_collision_logs(world, sensors, actors, world.get_snapshot().frame))
            return state

        for record in frames[:checkpoint_after]:
            step(record, report['prefix_collision_events'])
        frame = world.get_snapshot().frame
        saved = {}
        for label, actor in sorted(actors.items()):
            if not actor.save_physics_checkpoint():
                raise RuntimeError('save rejected for {}'.format(label))
            saved[label] = actor.export_physics_checkpoint(identity['selected_files_id'])
            if world.get_snapshot().frame != frame:
                raise RuntimeError('world advanced during save')
        def capture_physics():
            frame = world.get_snapshot().frame
            result = {}
            for label, actor in sorted(actors.items()):
                if not actor.save_physics_checkpoint():
                    raise RuntimeError('physics trace capture rejected for {}'.format(label))
                try:
                    data = actor.export_physics_checkpoint(identity['selected_files_id'])
                    result[label] = base64.b64encode(data).decode('ascii')
                finally:
                    if not actor.import_physics_checkpoint(saved[label], identity['selected_files_id'], True):
                        raise RuntimeError('failed to reinstall slot for {}'.format(label))
                if world.get_snapshot().frame != frame:
                    raise RuntimeError('world advanced during physics trace capture')
            return result
        if reference:
            if scenario.sample_checkpoint(world, actors) != reference['expected_boundary']:
                raise RuntimeError('immediate-restore reference boundary differs')
            if set(saved) != set(reference['actors']):
                raise RuntimeError('immediate-restore reference actors differ')
            for label in saved:
                original = base64.b64decode(reference['actors'][label]['data_base64'], validate=True)
                if fields.compare(original, saved[label]):
                    raise RuntimeError('immediate-restore reference checkpoint differs for {}'.format(label))
            report['baseline'] = reference['expected_suffix'][:suffix_frames]
            for record in frames[checkpoint_after:checkpoint_after + advance_before_restore]:
                scenario.execute(world, client, actors, record)
        else:
            for record in suffix:
                report['baseline'].append(step(record, report['baseline_collision_events']))
                if physics_trace:
                    report['baseline_physics'].append(capture_physics())
        for _ in range(repetitions):
            frame = world.get_snapshot().frame
            for label, actor in sorted(actors.items()):
                if not actor.restore_physics_checkpoint():
                    raise RuntimeError('restore rejected for {}'.format(label))
                if world.get_snapshot().frame != frame:
                    raise RuntimeError('world advanced during restore')
            boundary = {}
            for label, actor in sorted(actors.items()):
                if not actor.save_physics_checkpoint():
                    raise RuntimeError('boundary recapture rejected for {}'.format(label))
                try:
                    observed = actor.export_physics_checkpoint(identity['selected_files_id'])
                    boundary[label] = fields.compare(saved[label], observed)
                finally:
                    if not actor.import_physics_checkpoint(saved[label], identity['selected_files_id'], True):
                        raise RuntimeError('failed to reinstall slot for {}'.format(label))
                if world.get_snapshot().frame != frame:
                    raise RuntimeError('world advanced during boundary recapture')
            event_trace = []
            trace, physics = [], []
            for record in suffix:
                trace.append(step(record, event_trace))
                if physics_trace:
                    physics.append(capture_physics())
            first = next((i + 1 for i, (a, b) in enumerate(zip(report['baseline'], trace)) if a != b), None)
            first_physics = None
            for index, (expected, actual) in enumerate(zip(report['baseline_physics'], physics), 1):
                diff = {label: fields.compare(base64.b64decode(expected[label]), base64.b64decode(actual[label]))
                        for label in expected}
                if any(diff.values()):
                    first_physics = {'suffix_frame': index, 'differences': diff}
                    break
            report['restores'].append({'first_difference': first, 'exact': first is None,
                                       'server_boundary_differences': boundary, 'suffix': trace,
                                       'collision_events': event_trace, 'physics': physics,
                                       'first_physics_difference': first_physics})
        if scenario.fingerprint() != identity:
            raise RuntimeError('selected artifacts changed during probe')
        return report
    except Exception as exc:
        report['failure'] = {'type': type(exc).__name__, 'message': str(exc)}
        raise
    finally:
        try:
            for sensor in sensors.values():
                sensor.destroy()
            for actor in actors.values():
                actor.destroy()
        finally:
            scenario.atomic_create(report_path, report)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--checkpoint-after', type=int, default=21)
    parser.add_argument('--suffix-frames', type=int, default=10)
    parser.add_argument('--repetitions', type=int, default=2)
    parser.add_argument('--exclude-other', action='store_true',
                        help='remove the second vehicle and its events for an isolated ego control')
    parser.add_argument('--immediate-reference', help='compare against this captured bundle; requires one repetition')
    parser.add_argument('--advance-before-restore', type=int, default=0,
                        help='advance this many journal frames before restoring when using --immediate-reference')
    parser.add_argument('--spawn-z-offset', type=float, default=0.0,
                        help='raise ego spawn point by this many metres')
    parser.add_argument('--collision-events', action='store_true', help='record diagnostic collision sensor hits')
    parser.add_argument('--physics-trace', action='store_true', help='recapture portable physics fields after every suffix tick')
    parser.add_argument('--report', required=True)
    parser.add_argument('--port', type=int, default=22000)
    parser.add_argument('--timeout', type=float, default=180)
    args = parser.parse_args()
    if args.repetitions < 1:
        parser.error('--repetitions must be positive')
    client = carla.Client('127.0.0.1', args.port)
    client.set_timeout(args.timeout)
    settings = client.get_world().get_settings()
    try:
        report = run(client, args.checkpoint_after, args.suffix_frames, args.repetitions, args.report,
                     args.exclude_other, args.immediate_reference, args.advance_before_restore, args.spawn_z_offset,
                     args.collision_events, args.physics_trace)
        print('First differing suffix frames:', [item['first_difference'] for item in report['restores']])
    finally:
        client.get_world().apply_settings(settings)


if __name__ == '__main__':
    main()
