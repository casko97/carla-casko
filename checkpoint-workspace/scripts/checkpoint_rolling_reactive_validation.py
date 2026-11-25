#!/usr/bin/env python3
"""Validate a rolling pre-trigger recipe and repeated reactive branches.

Loads and replaces worlds. Run only on a dedicated checkpoint-enabled server.
"""
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
from checkpoint_rolling_same_control_probe import first_difference, sample


def compare(left, right):
    if not left or len(left) != len(right):
        raise ValueError('trajectories must be nonempty and the same length')
    labels = set(left[0]['states'])
    if not labels or any(set(x['states']) != labels for x in left + right):
        raise ValueError('vehicle labels differ between frames')
    first = None
    for index, (a, b) in enumerate(zip(left, right), 1):
        difference = first_difference(a, b)
        if difference is not None:
            first = {'tick': index, **difference}
            break
    result = {'exact_sampled_match': first is None, 'first_difference': first,
              'max_position_error_m': {}, 'max_velocity_error_mps': {}}
    for label in sorted(labels):
        for field, target in [('location', 'max_position_error_m'), ('velocity', 'max_velocity_error_mps')]:
            distances = [math.dist(a['states'][label][field], b['states'][label][field]) for a, b in zip(left, right)]
            if not all(math.isfinite(x) for x in distances):
                raise ValueError('non-finite vehicle state')
            result[target][label] = max(distances)
    return result


def reaction(original, changed):
    if not original or len(original) != len(changed):
        raise ValueError('branches must be nonempty and the same length')
    return {'max_traffic_position_difference_m': max(math.dist(a['states']['traffic']['location'],
                b['states']['traffic']['location']) for a, b in zip(original, changed)),
            'max_traffic_control_difference': max(abs(a['traffic_control'][key] - b['traffic_control'][key])
                for a, b in zip(original, changed) for key in ('steer', 'throttle', 'brake')),
            'min_center_distance_m': min(x['center_distance_m'] for x in original + changed)}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', type=int, default=38000)
    p.add_argument('--tm-port', type=int, default=8800)
    p.add_argument('--recipe', required=True)
    p.add_argument('--report', required=True)
    p.add_argument('--variant', choices=('straight', 'steer'), default='straight')
    args = p.parse_args()
    if Path(args.recipe).exists() or Path(args.report).exists():
        raise FileExistsError('recipe and report must be new paths')
    client = carla.Client('127.0.0.1', args.port)
    client.set_timeout(360)
    original_settings = None
    actors, tm = {}, None
    report = {'scope': 'Town01 ego + reactive TM vehicle, fixed props, no cameras or ScenarioRunner',
              'variant': args.variant, 'failure': None}
    try:
        original_settings = client.get_world().get_settings()
        print('server connected; loading controlled scenario', flush=True)
        world, guard, specs, actors, tm = reactive.setup(client, 'Town01', args.tm_port, 17)
        print('controlled scenario ready; recording original continuation', flush=True)
        weather = common.get_weather(world)
        buffer = rolling.ReactivePrefixBuffer(world)
        frames = []
        chosen = None
        for index in range(240):
            action = ({'brake': 1.0} if index < 140 else
                      ({'throttle': .4, 'steer': .05} if args.variant == 'steer' else {'throttle': .4}))
            observation = sample(client, world, actors, action)
            snap = world.get_snapshot()
            buffer.observe_reactive(action, client, tm, actors, guard, specs, 17, weather)
            frames.append({'frame': snap.frame, 'action': action, 'observation': observation})
            if index == 139:
                chosen = buffer.buffer.select_and_pin(snap.frame, snap.timestamp.elapsed_seconds)
                report.update(trigger_frame=snap.frame, checkpoint_frame=chosen.frame,
                              lookback_seconds=snap.timestamp.elapsed_seconds - chosen.elapsed_seconds,
                              recipe_sha256=chosen.sha256)
        if chosen is None or chosen not in buffer.buffer.checkpoints:
            raise RuntimeError('selected checkpoint was not retained')
        suffix = [row for row in frames if row['frame'] > chosen.frame]
        if not suffix or suffix[0]['frame'] != chosen.frame + 1:
            raise RuntimeError('baseline is incomplete at the checkpoint')
        common.atomic_create(args.recipe, chosen.recipe())
        print('checkpoint pinned; reconstructing same-control branch', flush=True)
        report['source'] = suffix
        report['suffix_frames'] = len(suffix)
        report['trigger_to_end_frames'] = frames[-1]['frame'] - report['trigger_frame']
        reactive.close(tm, actors, args.tm_port)
        actors, tm = {}, None
        world, actors, tm, checkpoint, _ = reactive.reconstruct(client, args.recipe, args.tm_port)
        report['boundary_exact'] = True
        recorded = [sample(client, world, actors, row['action']) for row in suffix]
        report['same_control'] = recorded
        report['same_control_comparison'] = compare([row['observation'] for row in suffix], recorded)
        print('same-control branch recorded; testing changed controls', flush=True)
        checkpoint.restore()
        changed = [sample(client, world, actors, {'throttle': .4}) for _ in suffix]
        report['alternative_control'] = changed
        checkpoint.restore()
        repeat = [sample(client, world, actors, {'throttle': .4}) for _ in suffix]
        report['alternative_repeat'] = repeat
        report['alternative_repeat_comparison'] = compare(changed, repeat)
        report['traffic_reaction'] = reaction(recorded, changed)
        report['traffic_reacted'] = report['traffic_reaction']['max_traffic_control_difference'] > .01
        summary = {k: v for k, v in report.items() if k not in ('source', 'same_control',
                   'alternative_control', 'alternative_repeat')}
        print(json.dumps(summary, indent=2), flush=True)
        if (not report['same_control_comparison']['exact_sampled_match'] or
                not report['alternative_repeat_comparison']['exact_sampled_match'] or
                not report['traffic_reacted']):
            raise AssertionError('controlled replay or reactive-traffic check did not pass')
    except Exception as exc:
        report['failure'] = {'type': type(exc).__name__, 'message': str(exc)}
        raise
    finally:
        try:
            reactive.close(tm, actors, args.tm_port)
            if original_settings is not None:
                client.get_world().apply_settings(original_settings)
        finally:
            common.atomic_create(args.report, report)


if __name__ == '__main__':
    main()
