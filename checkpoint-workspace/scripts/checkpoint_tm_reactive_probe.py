#!/usr/bin/env python3
"""Test same-episode TM checkpoint reactions to different expert actions.

Requires a dedicated development server with the experimental TM Python API.
The checkpoint is volatile and valid only for the same episode and actors.
"""
import argparse
import math
from pathlib import Path

import carla
import checkpoint_fixed_props as fixed_props
import checkpoint_scenario_bundle as common
from checkpoint_tm_same_episode import SameEpisodeTrafficCheckpoint
import multi_vehicle_physics_checkpoint as multi
import vehicle_physics_checkpoint as single


def record_step(client, world, actors, expert_control):
    states = multi.step(client, world, actors, {'expert': expert_control})
    control = actors['traffic'].get_control()
    distance = math.dist(states['expert']['location'], states['traffic']['location'])
    if distance < 6.0:
        raise RuntimeError('stopped before a close vehicle approach (center distance below 6 m)')
    return {'states': states, 'traffic_control': {key: getattr(control, key) for key in common.CONTROL_FIELDS},
            'center_distance_m': distance}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True)
    parser.add_argument('--map', default='Town01', choices=('Town01', 'Town02'))
    parser.add_argument('--port', type=int, default=36000)
    parser.add_argument('--tm-port', type=int, default=8600)
    parser.add_argument('--seed', type=int, default=17)
    parser.add_argument('--timeout', type=float, default=360)
    args = parser.parse_args()
    if Path(args.output).exists():
        raise FileExistsError(args.output)
    client = carla.Client('127.0.0.1', args.port)
    client.set_timeout(args.timeout)
    original_settings = client.get_world().get_settings()
    world, tm, actors = None, None, {}
    report = {'scope': 'same-episode reactive TM experiment; not a portable scenario checkpoint',
              'map': args.map, 'tm_seed': args.seed, 'branches': {}, 'failure': None}
    try:
        world = single.load_world(client, args.map)
        guard = fixed_props.prepare(client, world)
        spawn = world.get_map().get_spawn_points()[0]
        yaw = math.radians(spawn.rotation.yaw)
        lead = {'x': spawn.location.x + 18.0 * math.cos(yaw),
                'y': spawn.location.y + 18.0 * math.sin(yaw), 'z': spawn.location.z,
                'pitch': spawn.rotation.pitch, 'yaw': spawn.rotation.yaw, 'roll': spawn.rotation.roll}
        specs = [{'id': 'expert', 'blueprint': 'vehicle.lincoln.mkz_2020', 'transform': lead},
                 {'id': 'traffic', 'blueprint': 'vehicle.lincoln.mkz_2020', 'spawn_index': 0,
                  'controller': 'traffic_manager'}]
        actors = multi.spawn_actors(world, specs)
        tm = multi.setup_traffic_manager(client, actors, specs, args.tm_port, args.seed)
        tm.auto_lane_change(actors['traffic'], False)
        tm.distance_to_leading_vehicle(actors['traffic'], 6.0)
        for _ in range(60):
            record_step(client, world, actors, {'brake': 1.0})
        checkpoint = SameEpisodeTrafficCheckpoint(client, world, tm, actors, guard)
        frame = checkpoint.capture()
        report['checkpoint_frame'] = frame
        report['checkpoint_state'] = {label: single.sample(world, actor) for label, actor in actors.items()}
        for index, (name, command) in enumerate((('hold', {'brake': 1.0}), ('move', {'throttle': 0.4}),
                                                  ('move_repeat', {'throttle': 0.4}), ('hold_repeat', {'brake': 1.0}))):
            if index:
                checkpoint.restore()
            report['branches'][name] = [record_step(client, world, actors, command) for _ in range(120)]
        hold, move = report['branches']['hold'], report['branches']['move']
        control_fields = ('throttle', 'brake', 'steer')
        report['max_traffic_control_difference'] = max(
            abs(a['traffic_control'][field] - b['traffic_control'][field])
            for a, b in zip(hold, move) for field in control_fields)
        report['max_traffic_position_difference_m'] = max(
            math.dist(a['states']['traffic']['location'], b['states']['traffic']['location']) for a, b in zip(hold, move))
        report['min_center_distance_m'] = min(frame['center_distance_m'] for branch in report['branches'].values() for frame in branch)
        report['repeat_max_position_error_m'] = {name: max(
            math.dist(a['states']['traffic']['location'], b['states']['traffic']['location'])
            for a, b in zip(report['branches'][name], report['branches'][name + '_repeat'])) for name in ('hold', 'move')}
        report['repeat_max_control_error'] = {name: max(
            abs(a['traffic_control'][field] - b['traffic_control'][field])
            for a, b in zip(report['branches'][name], report['branches'][name + '_repeat']) for field in control_fields)
            for name in ('hold', 'move')}
        report['max_expert_position_difference_m'] = max(
            math.dist(a['states']['expert']['location'], b['states']['expert']['location']) for a, b in zip(hold, move))
        repeat_noise = max(report['repeat_max_control_error'].values())
        report['observed_reaction'] = (report['max_expert_position_difference_m'] > 1.0 and
                                       report['max_traffic_control_difference'] > max(0.01, 2.0 * repeat_noise))
        fixed_props.validate_current(world, guard, ())
        print('TM observed reaction: {}; max control difference {:.4f}; repeat errors {}'.format(
            report['observed_reaction'], report['max_traffic_control_difference'], report['repeat_max_position_error_m']))
        return 0 if report['observed_reaction'] else 1
    except Exception as exc:
        report['failure'] = {'type': type(exc).__name__, 'message': str(exc)}
        raise
    finally:
        try:
            if tm is not None:
                if 'traffic' in actors:
                    actors['traffic'].set_autopilot(False, args.tm_port)
                tm.set_synchronous_mode(False)
            for actor in actors.values():
                actor.destroy()
            client.get_world().apply_settings(original_settings)
        finally:
            common.atomic_create(args.output, report)


if __name__ == '__main__':
    raise SystemExit(main())
