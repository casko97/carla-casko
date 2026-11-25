#!/usr/bin/env python3
"""Capture a rolling recipe, reconstruct, and compare same-control rollouts."""
import argparse
import json
import math
import sys
from pathlib import Path

root = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(root / 'scripts'), str(root / 'deterministic-cpt/checkpoint-workspace/scripts'),
                str(root / 'deterministic-cpt/PythonAPI/examples')]
import carla
import checkpoint_rolling as rolling
import checkpoint_scenario_bundle as common
import checkpoint_tm_prefix_reconstruction as reactive
from checkpoint_tm_reactive_probe import record_step
import multi_vehicle_physics_checkpoint as multi


def sample(client, world, actors, action):
    result = record_step(client, world, actors, action)
    result['ego_control'] = multi.sample_tm_controls(actors, ['expert'])['expert']
    result['traffic_lights'] = multi.sample_traffic_lights(multi.traffic_lights(world))
    return result


def first_difference(a, b, path=''):
    if type(a) is not type(b):
        return {'field': path, 'expected': a, 'actual': b}
    if isinstance(a, dict):
        if set(a) != set(b):
            return {'field': path, 'expected_keys': sorted(a), 'actual_keys': sorted(b)}
        for key in sorted(a):
            result = first_difference(a[key], b[key], path + '.' + key)
            if result is not None:
                return result
    elif isinstance(a, list):
        if len(a) != len(b):
            return {'field': path, 'expected_length': len(a), 'actual_length': len(b)}
        for index, (left, right) in enumerate(zip(a, b)):
            result = first_difference(left, right, path + '[{}]'.format(index))
            if result is not None:
                return result
    elif a != b:
        return {'field': path, 'expected': a, 'actual': b}
    return None


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=38000)
    parser.add_argument('--tm-port', type=int, default=8800)
    parser.add_argument('--recipe', required=True)
    parser.add_argument('--report', required=True)
    parser.add_argument('--variant', choices=('straight', 'steer'), default='straight')
    args = parser.parse_args()
    if Path(args.recipe).exists() or Path(args.report).exists():
        raise FileExistsError('recipe and report must not exist')
    client = carla.Client('127.0.0.1', args.port)
    client.set_timeout(360)
    original = client.get_world().get_settings()
    actors, tm = {}, None
    report = {'scope': 'controlled Town01; prefix reconstruction, fixed props, two vehicles, no sensors or ScenarioRunner',
              'variant': args.variant,
              'failure': None}
    try:
        world, guard, specs, actors, tm = reactive.setup(client, 'Town01', args.tm_port, 17)
        weather = common.get_weather(world)
        buffer = rolling.ReactivePrefixBuffer(world)
        records = []
        selected = None
        for i in range(220):
            action = ({'brake': 1.0} if i < 140 else
                      ({'throttle': .4, 'steer': .15} if args.variant == 'steer' else {'throttle': .4}))
            observation = sample(client, world, actors, action)
            snap = world.get_snapshot()
            captured = buffer.observe_reactive(action, client, tm, actors, guard, specs, 17, weather)
            if world.get_snapshot().frame != snap.frame:
                raise RuntimeError('checkpoint advanced the world')
            records.append({'frame': snap.frame, 'action': action, 'observation': observation})
            if i == 139:
                selected = buffer.buffer.select_and_pin(snap.frame, snap.timestamp.elapsed_seconds)
                report.update({'trigger_frame': snap.frame, 'checkpoint_frame': selected.frame,
                               'lookback_seconds': snap.timestamp.elapsed_seconds - selected.elapsed_seconds,
                               'recipe_sha256': selected.sha256,
                               'vehicle_blobs': sorted(selected.recipe()['rolling_vehicle_checkpoints'])})
        if selected not in buffer.buffer.checkpoints:
            raise RuntimeError('selected checkpoint was evicted')
        tail = [r for r in records if r['frame'] > selected.frame]
        if not tail or tail[0]['frame'] != selected.frame + 1:
            raise RuntimeError('baseline does not start after checkpoint')
        common.atomic_create(args.recipe, selected.recipe())
        report['suffix_frames'] = len(tail)
        report['baseline'] = tail
        reactive.close(tm, actors, args.tm_port)
        actors, tm = {}, None
        world, actors, tm, _slot, _recipe = reactive.reconstruct(client, args.recipe, args.tm_port)
        report['reconstructed_boundary_exact'] = True
        report['candidate'] = [sample(client, world, actors, row['action']) for row in tail]
        report['same_control_sequence'] = [row['action'] for row in tail]
        report['first_difference'] = None
        for index, (baseline, replay) in enumerate(zip(tail, report['candidate']), 1):
            difference = first_difference(baseline['observation'], replay)
            if difference is not None:
                report['first_difference'] = {'suffix_tick': index, **difference}
                break
        report['exact_sampled_trace_match'] = report['first_difference'] is None
        report['max_position_error_m'] = {label: max(math.dist(a['observation']['states'][label]['location'],
                    b['states'][label]['location']) for a, b in zip(tail, report['candidate']))
                                          for label in actors}
        report['max_velocity_error_mps'] = {label: max(math.dist(a['observation']['states'][label]['velocity'],
                    b['states'][label]['velocity']) for a, b in zip(tail, report['candidate']))
                                            for label in actors}
        print(json.dumps({k: v for k, v in report.items() if k not in ('baseline', 'candidate', 'same_control_sequence')}, indent=2), flush=True)
    except Exception as exc:
        report['failure'] = {'type': type(exc).__name__, 'message': str(exc)}
        raise
    finally:
        try:
            reactive.close(tm, actors, args.tm_port)
            client.get_world().apply_settings(original)
        finally:
            common.atomic_create(args.report, report)


if __name__ == '__main__':
    main()
