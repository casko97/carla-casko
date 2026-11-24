#!/usr/bin/env python3
"""Compare replay of observed Traffic Manager controls on a dedicated server.

The recorded controls are observations after each synchronous tick. This probe
checks whether those controls reproduce that tick's state after fresh restore;
it does not restore Traffic Manager's decision-making state.
"""
import argparse
import math
from pathlib import Path

import carla
import checkpoint_fixed_props as fixed_props
import checkpoint_live_vehicles as live
import checkpoint_scenario_bundle as common
import multi_vehicle_physics_checkpoint as multi
import vehicle_physics_checkpoint as single


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=35000)
    parser.add_argument('--tm-port', type=int, default=8500)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--frames', type=int, default=120)
    parser.add_argument('--vehicles', type=int, choices=(1, 2), default=1)
    parser.add_argument('--seed', type=int, default=17)
    parser.add_argument('--following', action='store_true', help='start two vehicles in the same lane')
    parser.add_argument('--traffic-rpc', help='capture traffic-light controller state with this helper')
    args = parser.parse_args()
    if args.frames < 1:
        parser.error('--frames must be positive')
    if args.following and args.vehicles != 2:
        parser.error('--following requires --vehicles 2')
    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=False)
    client = carla.Client('127.0.0.1', args.port)
    client.set_timeout(180)
    settings = client.get_world().get_settings()
    world = None
    actors = {}
    tm = client.get_trafficmanager(args.tm_port)
    try:
        world = single.load_world(client, 'Town01')
        guard = fixed_props.prepare(client, world)
        tm.set_synchronous_mode(True)
        tm.set_random_device_seed(args.seed)
        bootstrap = {}
        for index in range(args.vehicles):
            label = 'vehicle{}'.format(index)
            initial = world.get_map().get_spawn_points()[0 if args.following else index]
            if args.following and index == 1:
                yaw = math.radians(initial.rotation.yaw)
                initial.location.x += 12.0 * math.cos(yaw)
                initial.location.y += 12.0 * math.sin(yaw)
            bootstrap[label] = {k: getattr(initial.location, k) for k in ('x', 'y', 'z')}
            bootstrap[label].update({k: getattr(initial.rotation, k) for k in ('pitch', 'yaw', 'roll')})
            blueprint = world.get_blueprint_library().find('vehicle.lincoln.mkz_2020')
            blueprint.set_attribute('sticky_control', 'false')
            actors[label] = world.try_spawn_actor(blueprint, initial)
            if actors[label] is None:
                raise RuntimeError('vehicle spawn failed: {}'.format(label))
            actors[label].set_autopilot(True, args.tm_port)
            if args.following:
                tm.auto_lane_change(actors[label], False)
        for _ in range(80):
            world.tick()
        live.capture(world, {label: actor.id for label, actor in actors.items()}, output / 'checkpoint.json',
                     bootstrap_transforms=bootstrap, fixed_props_guard=guard, port=args.port,
                     traffic_rpc=args.traffic_rpc)
        actions, reference = [], []
        lights = multi.traffic_lights(world) if args.traffic_rpc else {}
        light_ids = sorted(lights)
        light_ids = [light_ids[i * len(light_ids) // min(8, len(light_ids))]
                     for i in range(min(8, len(light_ids)))]
        lights = {key: lights[key] for key in light_ids}
        traffic_reference = []
        for _ in range(args.frames):
            world.tick()
            observed_controls = {label: actor.get_control() for label, actor in actors.items()}
            controls = {label: {name: getattr(control, name) for name in common.CONTROL_FIELDS}
                        for label, control in observed_controls.items()}
            actions.append(controls)
            reference.append({label: {'type': actor.type_id, 'control': controls[label],
                                      'state': single.sample(world, actor)}
                              for label, actor in actors.items()})
            if lights:
                traffic_reference.append(multi.sample_traffic_lights(lights))
        common.atomic_create(output / 'actions.json', actions)
        common.atomic_create(output / 'baseline.json', reference)
        if traffic_reference:
            common.atomic_create(output / 'traffic_baseline.json', traffic_reference)
        for actor in actors.values():
            actor.set_autopilot(False, args.tm_port)
            actor.destroy()
        actors.clear()
        tm.set_synchronous_mode(False)
        result = live.replay(client, output / 'checkpoint.json', actions,
                             output / 'restore_report.json', reference, port=args.port,
                             traffic_reference=traffic_reference or None, traffic_rpc=args.traffic_rpc)
        errors = {}
        for label in sorted(reference[0]):
            errors[label] = {field: max(math.dist(expected[label]['state'][field], actual[label]['state'][field])
                                        for expected, actual in zip(reference, result['suffix']))
                             for field in ('location', 'velocity')}
        summary = {'exact': result['exact'], 'first_difference': result['first_difference'],
                   'traffic_exact': result['traffic_exact'], 'max_errors': errors}
        if args.following:
            summary['min_center_distance_m'] = min(math.dist(frame['vehicle0']['state']['location'],
                                                            frame['vehicle1']['state']['location'])
                                                   for frame in reference)
        common.atomic_create(output / 'summary.json', summary)
        print('Recorded Traffic Manager controls: exact = {}; traffic exact = {}; max errors = {}'.format(
            result['exact'], result['traffic_exact'], errors))
        return 0 if result['exact'] and result['traffic_exact'] is not False else 1
    finally:
        for actor in actors.values():
            actor.set_autopilot(False, args.tm_port)
            actor.destroy()
        tm.set_synchronous_mode(False)
        client.get_world().apply_settings(settings)


if __name__ == '__main__':
    raise SystemExit(main())
