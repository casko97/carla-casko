#!/usr/bin/env python3
"""Exercise live capture and reconstruction with two reactive traffic vehicles."""
import argparse
import json
import math
from pathlib import Path

import carla
import checkpoint_live_vehicles as live
import checkpoint_scenario_bundle as common
import checkpoint_tm_prefix_reconstruction as prefix
import multi_vehicle_physics_checkpoint as multi


def step(client, world, actors, command):
    states = multi.step(client, world, actors, {'expert': command})
    distances = [math.dist(a['location'], b['location']) for i, a in enumerate(states.values())
                 for b in list(states.values())[i + 1:]]
    if min(distances) < 6:
        raise RuntimeError('vehicles approached within 6 m')
    return {'states': states, 'controls': multi.sample_tm_controls(actors, ['traffic', 'traffic2']),
            'min_distance': min(distances)}


def branches(client, world, actors, slot):
    result = {}
    for index, (name, action) in enumerate((('hold', {'brake': 1.0}), ('move', {'throttle': 0.4}),
                                             ('move_repeat', {'throttle': 0.4}), ('hold_repeat', {'brake': 1.0}))):
        if index:
            slot.restore()
        result[name] = [step(client, world, actors, action) for _ in range(120)]
    return result


def summarize(traces, reference=None):
    result = {'same_action_exact': {n: traces[n] == traces[n + '_repeat'] for n in ('hold', 'move')},
              'min_center_distance_m': min(f['min_distance'] for branch in traces.values() for f in branch),
              'traffic_max_control_difference': max(abs(a['controls']['traffic'][k] - b['controls']['traffic'][k])
                  for a, b in zip(traces['hold'], traces['move']) for k in ('throttle', 'brake', 'steer')),
              'traffic_max_position_difference_m': max(math.dist(a['states']['traffic']['location'],
                  b['states']['traffic']['location']) for a, b in zip(traces['hold'], traces['move']))}
    if reference:
        result['source_action_exact'] = {n: traces[n] == reference[n] for n in ('hold', 'move')}
        result['source_max_position_error_m'] = {n: {label: max(math.dist(a['states'][label]['location'],
            b['states'][label]['location']) for a, b in zip(traces[n], reference[n]))
            for label in ('expert', 'traffic', 'traffic2')} for n in ('hold', 'move')}
    return result


def main():
    p = argparse.ArgumentParser()
    p.add_argument('operation', choices=('capture', 'reconstruct'))
    p.add_argument('--recipe', required=True)
    p.add_argument('--reference', required=True)
    p.add_argument('--report', required=True)
    p.add_argument('--port', type=int, default=37000)
    p.add_argument('--tm-port', type=int, default=8700)
    a = p.parse_args()
    client = carla.Client('127.0.0.1', a.port)
    client.set_timeout(360)
    original = client.get_world().get_settings()
    actors, tm = {}, None
    report = {'operation': a.operation}
    try:
        if a.operation == 'capture':
            # Use the standard initializer, then rebuild with a distant third actor.
            world, guard, specs, actors, tm = prefix.setup(client, 'Town01', a.tm_port, 17)
            spawns = world.get_map().get_spawn_points()
            origin = spawns[0].location
            index = max(range(len(spawns)), key=lambda i: origin.distance(spawns[i].location))
            prefix.close(tm, actors, a.tm_port)
            actors, tm = {}, None
            specs.append({'id': 'traffic2', 'blueprint': 'vehicle.lincoln.mkz_2020',
                          'spawn_index': index, 'controller': 'traffic_manager'})
            config = {'traffic': {'auto_lane_change': False, 'distance_to_leading_vehicle': 6.0},
                      'traffic2': {'auto_lane_change': False, 'distance_to_leading_vehicle': 5.0,
                                   'speed_difference': 10.0}}
            world, guard, specs, actors, tm = prefix.setup(client, 'Town01', a.tm_port, 17, specs, tm_config=config)
            weather = common.get_weather(world)
            history = [{'brake': 1.0} for _ in range(60)]
            for action in history:
                step(client, world, actors, action)
            frame = world.get_snapshot().frame
            slot = live.capture_reactive_traffic(client, world, tm, actors, guard, specs, 17, history, weather, a.recipe, config)
            report['capture_did_not_tick'] = world.get_snapshot().frame == frame
            traces = branches(client, world, actors, slot)
            common.atomic_create(a.reference, {n: traces[n] for n in ('hold', 'move')})
            report['summary'] = summarize(traces)
        else:
            world, actors, tm, slot, recipe = live.restore_reactive_traffic(client, a.recipe, a.tm_port)
            report['boundary_exact'] = True
            traces = branches(client, world, actors, slot)
            report['summary'] = summarize(traces, json.loads(Path(a.reference).read_text()))
        print(json.dumps(report, indent=2))
        common.atomic_create(a.report, report)
        summary = report['summary']
        return 0 if (all(summary['same_action_exact'].values()) and summary['traffic_max_control_difference'] > .01
                     and all(summary.get('source_action_exact', {'none': True}).values())) else 1
    finally:
        prefix.close(tm, actors, a.tm_port, ['traffic', 'traffic2'])
        client.get_world().apply_settings(original)


if __name__ == '__main__':
    raise SystemExit(main())
