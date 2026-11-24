#!/usr/bin/env python3
"""Test replay of controls observed from Traffic Manager on a dedicated server.

This is a diagnostic for recorded-action replay, not a Traffic Manager state
checkpoint. A source TM runs synchronously; the fresh episode replays its
sampled controls as manual commands and does not continue autonomous planning.
"""
import argparse
import json
from pathlib import Path

import carla
import checkpoint_fixed_props as fixed_props
import checkpoint_live_vehicles as live
import checkpoint_scenario_bundle as common
import multi_vehicle_physics_checkpoint as multi
import vehicle_physics_checkpoint as single


def sample(world, actors):
    snapshot = world.get_snapshot()
    result = {}
    for label, actor in sorted(actors.items()):
        control = actor.get_control()
        result[label] = {'type': actor.type_id,
                         'control': {field: getattr(control, field) for field in common.CONTROL_FIELDS},
                         'state': single.sample(world, actor)}
    if world.get_snapshot().frame != snapshot.frame:
        raise RuntimeError('world advanced during sampling')
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output-dir', required=True)
    p.add_argument('--port', type=int, default=35000)
    p.add_argument('--tm-port', type=int, default=35100)
    p.add_argument('--traffic-rpc', required=True)
    p.add_argument('--map', default='Town01')
    args = p.parse_args()
    out = Path(args.output_dir)
    out.mkdir(parents=True, exist_ok=False)
    client = carla.Client('127.0.0.1', args.port)
    client.set_timeout(180)
    original_settings = client.get_world().get_settings()
    actors, tm = {}, None
    report = {'failure': None, 'scope': 'replay observed TM controls; TM internal state not restored'}
    try:
        world = single.load_world(client, args.map)
        guard = fixed_props.prepare(client, world)
        specs = [{'id': 'ego', 'blueprint': 'vehicle.lincoln.mkz_2020', 'spawn_index': 0},
                 {'id': 'traffic', 'blueprint': 'vehicle.lincoln.mkz_2020', 'spawn_index': 1,
                  'controller': 'traffic_manager'}]
        actors = multi.spawn_actors(world, specs)
        tm = multi.setup_traffic_manager(client, actors, specs, args.tm_port, 17)
        for _ in range(60):
            multi.step(client, world, actors, {'ego': {'brake': 1.0}})
        points = world.get_map().get_spawn_points()
        bootstrap = {label: {'x': points[index].location.x, 'y': points[index].location.y,
                             'z': points[index].location.z, 'pitch': points[index].rotation.pitch,
                             'yaw': points[index].rotation.yaw, 'roll': points[index].rotation.roll}
                     for index, label in enumerate(('ego', 'traffic'))}
        lights_all = multi.traffic_lights(world)
        names = sorted(lights_all)
        lights = {k: lights_all[k] for k in names[::max(1, len(names)//8)][:8]}
        boundary = world.get_snapshot().frame
        receipt = live.capture(world, {label: actor.id for label, actor in actors.items()}, out / 'checkpoint.json',
                               bootstrap_transforms=bootstrap, fixed_props_guard=guard,
                               traffic_rpc=args.traffic_rpc, port=args.port)
        if world.get_snapshot().frame != boundary:
            raise RuntimeError('world advanced during capture')
        actions, reference, traffic_reference = [], [], []
        for _ in range(80):
            multi.step(client, world, actors, {'ego': {'brake': 1.0}})
            observation = sample(world, actors)
            actions.append({label: state['control'] for label, state in observation.items()})
            reference.append(observation)
            traffic_reference.append(multi.sample_traffic_lights(lights))
        common.atomic_create(out / 'actions.json', actions)
        common.atomic_create(out / 'baseline.json', reference)
        common.atomic_create(out / 'traffic_reference.json', traffic_reference)
        common.atomic_create(out / 'capture_receipt.json', receipt)
        report['source_traffic_controls_distinct'] = len({json.dumps(a['traffic'], sort_keys=True) for a in actions})
        report['source_traffic_max_speed_m_s'] = max(sum(v*v for v in f['traffic']['state']['velocity'])**0.5 for f in reference)
        actors['traffic'].set_autopilot(False, args.tm_port)
        tm.set_synchronous_mode(False)
        tm = None
        for actor in actors.values():
            actor.destroy()
        actors = {}
        replay = live.replay(client, out / 'checkpoint.json', actions, out / 'restore_report.json', reference,
                             port=args.port, traffic_reference=traffic_reference, traffic_rpc=args.traffic_rpc)
        report['vehicle_exact'] = replay['exact']
        report['traffic_light_exact'] = replay['traffic_exact']
        report['first_difference'] = replay['first_difference']
        print('recorded TM controls: {} distinct; maximum traffic speed {:.3f} m/s; vehicle exact {}; lights exact {}'.format(
            report['source_traffic_controls_distinct'], report['source_traffic_max_speed_m_s'],
            report['vehicle_exact'], report['traffic_light_exact']))
    except Exception as exc:
        report['failure'] = {'type': type(exc).__name__, 'message': str(exc)}
        raise
    finally:
        if tm is not None:
            if 'traffic' in actors:
                actors['traffic'].set_autopilot(False, args.tm_port)
            tm.set_synchronous_mode(False)
        for actor in actors.values():
            actor.destroy()
        common.atomic_create(out / 'probe_report.json', report)
        client.get_world().apply_settings(original_settings)


if __name__ == '__main__':
    main()
