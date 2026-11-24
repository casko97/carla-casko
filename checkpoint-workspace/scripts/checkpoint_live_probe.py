#!/usr/bin/env python3
"""Exercise live capture and fresh-episode replay on a dedicated test server."""
import argparse
import math
import json
from pathlib import Path

import carla
import checkpoint_fixed_props as fixed_props
import checkpoint_live_vehicles as live
import checkpoint_journal_bundle as journal
import checkpoint_vehicle_light_policy as light_policy
import checkpoint_live_walkers as live_walkers
import checkpoint_live_recorder as live_recorder
import checkpoint_walker_public_probe as walker_probe
import checkpoint_scenario_bundle as common
import vehicle_physics_checkpoint as single
import multi_vehicle_physics_checkpoint as multi


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', required=True, help='new directory for checkpoint and reports')
    parser.add_argument('--map', default='Town01', help='locally fingerprintable flat map')
    parser.add_argument('--vehicles', type=int, choices=(1, 2), default=1)
    parser.add_argument('--fixed-props', action='store_true', help='freeze map props before starting the scenario')
    parser.add_argument('--dynamic-cone', action='store_true', help='capture a dynamic cone before a scripted impact')
    parser.add_argument('--prop-rpc', help='native prop RPC helper, required with --dynamic-cone')
    parser.add_argument('--derived-lights', action='store_true', help='derive brake/reverse lights and fix position/low beam')
    parser.add_argument('--walker-rpc', help='include a manually controlled walker using the native velocity helper')
    parser.add_argument('--walker-lifecycle', action='store_true', help='destroy and respawn the captured walker during the suffix')
    parser.add_argument('--use-recorder', action='store_true', help='record suffix commands using the live recorder')
    parser.add_argument('--lifecycle', action='store_true', help='spawn, destroy and respawn another vehicle in the suffix')
    parser.add_argument('--suffix-frames', type=int, help='number of recorded replay frames')
    parser.add_argument('--prefix-frames', type=int, help='number of ticks before checkpoint capture')
    parser.add_argument('--brake-after', type=int, help='in the cone scenario, brake the ego from this zero-based prefix tick')
    parser.add_argument('--traffic-rpc', help='experimental portable traffic-light RPC helper')
    parser.add_argument('--traffic-diagnostics', action='store_true', help='compare sampled traffic lights during replay')
    parser.add_argument('--traffic-count', type=int, default=4, help='number of evenly selected traffic lights to compare')
    parser.add_argument('--port', type=int, default=23000)
    parser.add_argument('--timeout', type=float, default=180)
    args = parser.parse_args()
    if not 1 <= args.traffic_count <= 512:
        parser.error('--traffic-count must be between 1 and 512')
    if args.suffix_frames is not None and not 2 <= args.suffix_frames <= 1000:
        parser.error('--suffix-frames must be between 2 and 1000')
    if args.prefix_frames is not None and not 1 <= args.prefix_frames <= 1000:
        parser.error('--prefix-frames must be between 1 and 1000')
    if args.lifecycle and args.suffix_frames is not None and args.suffix_frames < (91 if args.vehicles == 2 else 81):
        parser.error('--lifecycle requires at least 81 frames, or 91 with two initial vehicles')
    if args.lifecycle and args.suffix_frames is None:
        args.suffix_frames = 120
    if args.dynamic_cone and not args.prop_rpc:
        parser.error('--dynamic-cone requires --prop-rpc')
    if args.brake_after is not None and (not args.dynamic_cone or args.brake_after < 20 or args.brake_after >= (args.prefix_frames or 100)):
        parser.error('--brake-after requires a cone scenario and must lie within the prefix after tick 19')
    if args.walker_lifecycle and (not args.walker_rpc or (args.suffix_frames is not None and args.suffix_frames < 91)):
        parser.error('--walker-lifecycle requires --walker-rpc and at least 91 suffix frames')
    if args.walker_lifecycle and args.suffix_frames is None:
        args.suffix_frames = 120
    if args.use_recorder and args.walker_rpc:
        parser.error('--use-recorder does not support walkers')
    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=False)
    client = carla.Client('127.0.0.1', args.port)
    client.set_timeout(args.timeout)
    original_settings = client.get_world().get_settings()
    world, actors, props, walkers = None, {}, {}, {}
    try:
        world = single.load_world(client, args.map)
        guard = fixed_props.prepare(client, world) if args.fixed_props else None
        if guard is not None:
            common.atomic_create(output / 'fixed_props_guard.json', guard)
        prop_bootstrap = {}
        if args.dynamic_cone:
            spawn = world.get_map().get_spawn_points()[0]
            yaw = math.radians(spawn.rotation.yaw)
            transform = carla.Transform(carla.Location(x=spawn.location.x + 8 * math.cos(yaw),
                y=spawn.location.y + 8 * math.sin(yaw), z=0.05), carla.Rotation(yaw=spawn.rotation.yaw))
            prop_bootstrap['cone'] = {'x': transform.location.x, 'y': transform.location.y, 'z': transform.location.z,
                                      'pitch': transform.rotation.pitch, 'yaw': transform.rotation.yaw, 'roll': transform.rotation.roll}
            prop = world.try_spawn_actor(world.get_blueprint_library().find('static.prop.trafficcone01'), transform)
            if prop is None:
                raise RuntimeError('cone spawn failed')
            props['cone'] = prop
            replies = client.apply_batch_sync([carla.command.SetSimulatePhysics(prop.id, True)], False)
            if len(replies) != 1 or replies[0].error:
                raise RuntimeError('enable cone physics failed')
            world.tick()
        bootstrap = {}
        for index, label in enumerate(('ego', 'other')[:args.vehicles]):
            initial = world.get_map().get_spawn_points()[index]
            bootstrap[label] = {'x': initial.location.x, 'y': initial.location.y, 'z': initial.location.z,
                                'pitch': initial.rotation.pitch, 'yaw': initial.rotation.yaw, 'roll': initial.rotation.roll}
            actors[label] = single.spawn(world, 'vehicle.lincoln.mkz_2020', index)
        walker_bootstrap = {}
        walker_direction = None
        if args.walker_rpc:
            pedestrian, initial = walker_probe.walker_spawn(world)
            walkers['pedestrian'] = pedestrian
            walker_bootstrap['pedestrian'] = {'x': initial.location.x, 'y': initial.location.y, 'z': initial.location.z,
                'pitch': initial.rotation.pitch, 'yaw': initial.rotation.yaw, 'roll': initial.rotation.roll}
            yaw = math.radians(initial.rotation.yaw)
            walker_direction = [math.cos(yaw), math.sin(yaw), 0.0]
        prefix = [{label: ({'throttle': 0.25, 'steer': 0.04} if label == 'ego' else {'brake': 1.0})
                   for label in actors} for _ in range(args.prefix_frames or 40)]
        actions = [{label: ({'throttle': 0.3, 'steer': 0.08} if label == 'ego' else {'brake': 1.0})
                    for label in actors} for _ in range(args.suffix_frames or 40)]
        if args.dynamic_cone:
            prefix = [{label: ({'brake': 1.0} if i < 20 or label != 'ego' or
                                (args.brake_after is not None and i >= args.brake_after) else {'throttle': 0.4})
                       for label in actors} for i in range(args.prefix_frames or 100)]
            actions = [{label: ({'throttle': 0.4} if label == 'ego' else {'brake': 1.0})
                        for label in actors} for _ in range(args.suffix_frames or 60)]
        policy = {'mode': light_policy.POLICY, 'fixed_mask': 3} if args.derived_lights else None
        for control in prefix:
            if walkers:
                live_walkers.apply_controls(client, world, walkers, {'pedestrian': {'direction': walker_direction, 'speed': 1.4, 'jump': False}})
            journal.execute(world, client, actors, {'events': [], 'controls': control}, {}, policy)
        if policy:
            for index, control in enumerate(actions):
                if 20 <= index < 25:
                    control['ego'] = {'brake': 1.0}
                elif 40 <= index < 45:
                    control['ego'] = {'reverse': True, 'brake': 0.0}

        lights = multi.traffic_lights(world) if args.traffic_diagnostics else {}
        keys = sorted(lights)
        selected = [keys[i * len(keys) // min(args.traffic_count, len(keys))]
                    for i in range(min(args.traffic_count, len(keys)))]
        lights = {key: lights[key] for key in selected}
        traffic_boundary = multi.sample_traffic_lights(lights) if lights else None
        frame = world.get_snapshot().frame
        before = common.sample_checkpoint(world, actors)
        receipt = live.capture(world, {label: actor.id for label, actor in actors.items()}, output / 'checkpoint.json',
                               allow_uncaptured_props=not args.fixed_props, bootstrap_transforms=bootstrap,
                               fixed_props_guard=guard, prop_mapping={label: a.id for label, a in props.items()} or None,
                               prop_bootstrap_transforms=prop_bootstrap if props else None,
                               prop_rpc=args.prop_rpc if props else None, port=args.port, traffic_rpc=args.traffic_rpc,
                               vehicle_light_policy=policy, walker_mapping={k: v.id for k, v in walkers.items()} or None,
                               walker_bootstrap_transforms=walker_bootstrap if walkers else None, walker_rpc=args.walker_rpc)
        if world.get_snapshot().frame != frame or common.sample_checkpoint(world, actors) != before:
            raise RuntimeError('capture changed the observed frame or sampled state')
        if args.lifecycle:
            extra = [{'id': 'later', 'blueprint': 'vehicle.lincoln.mkz_2020', 'spawn_index': args.vehicles}]
            frames, alive, other_alive = [], False, True
            for i, controls in enumerate(actions):
                events = []
                if i in (10, 50):
                    events.append({'action': 'spawn', 'label': 'later'})
                    alive = True
                elif i in (30, 80):
                    events.append({'action': 'destroy', 'label': 'later'})
                    alive = False
                if args.vehicles == 2 and i == 65:
                    events.append({'action': 'destroy', 'label': 'other'})
                    other_alive = False
                elif args.vehicles == 2 and i == 90:
                    events.append({'action': 'spawn', 'label': 'other'})
                    other_alive = True
                controls = dict(controls)
                if args.vehicles == 2 and not other_alive:
                    controls.pop('other')
                if alive:
                    controls['later'] = {'brake': 1.0}
                frames.append({'events': events, 'controls': controls})
            actions = {'actors': extra, 'frames': frames}
        bundle = json.loads((output / 'checkpoint.json').read_text())
        frames, action_specs = live.prepare_actions(bundle, actions)
        walker_actions = []
        if walkers:
            walker_actions = [{'pedestrian': {'direction': (walker_direction if i < 30 or i >= 70 else
                                  [-walker_direction[1], walker_direction[0], 0.0]),
                                  'speed': (0.0 if 50 <= i < 60 else 1.4), 'jump': False}}
                              for i in range(len(frames))]
            if args.walker_lifecycle:
                lifecycle_frames = []
                alive = True
                later_alive = False
                for index, controls in enumerate(walker_actions):
                    events = []
                    if index in (30, 70):
                        events.append({'action': 'destroy', 'label': 'pedestrian'})
                        alive = False
                    elif index in (50, 90):
                        events.append({'action': 'spawn', 'label': 'pedestrian'})
                        alive = True
                    if index == 35:
                        events.append({'action': 'spawn', 'label': 'later_pedestrian'})
                        later_alive = True
                    elif index == 45:
                        events.append({'action': 'destroy', 'label': 'later_pedestrian'})
                        later_alive = False
                    active_controls = dict(controls) if alive else {}
                    if later_alive:
                        active_controls['later_pedestrian'] = controls['pedestrian']
                    lifecycle_frames.append({'events': events, 'controls': active_controls})
                captured_walker = bundle['walkers']['pedestrian']
                new_spec = {k: captured_walker[k] for k in ('blueprint', 'attributes', 'spawn_attributes', 'spawn_transform')}
                walker_actions = {'actors': {'later_pedestrian': new_spec}, 'frames': lifecycle_frames}
            walker_frames, walker_specs = live_walkers.prepare_actions(walker_actions, bundle['walkers'], len(frames), set(action_specs))
        baseline, prop_baseline, traffic_baseline, vehicle_light_baseline, walker_baseline = [], [], [], [], []
        recorder = None
        if args.use_recorder:
            extra_specs = actions['actors'] if isinstance(actions, dict) else []
            recorder = live_recorder.LiveActionRecorder(client, world, output / 'checkpoint.json', actors,
                                                        props, extra_specs, tuple(lights))
        for index, action in enumerate(frames):
            if args.walker_rpc:
                live_walkers.execute_frame(client, world, walkers, walker_frames[index], walker_specs)
            baseline.append(recorder.step(action) if recorder else journal.execute(world, client, actors, action, action_specs, policy))
            if policy:
                vehicle_light_baseline.append(multi.sample_vehicle_lights(world, actors))
            prop_baseline.append({label: single.sample(world, actor) for label, actor in sorted(props.items())})
            if args.walker_rpc:
                walker_baseline.append({label: single.sample(world, actor) for label, actor in sorted(walkers.items())})
            if lights:
                traffic_baseline.append(multi.sample_traffic_lights(lights))
        common.atomic_create(output / 'actions.json', actions)
        common.atomic_create(output / 'baseline.json', baseline)
        common.atomic_create(output / 'prop_baseline.json', prop_baseline)
        if policy:
            common.atomic_create(output / 'vehicle_light_baseline.json', vehicle_light_baseline)
        if args.walker_rpc:
            common.atomic_create(output / 'walker_actions.json', walker_actions)
            common.atomic_create(output / 'walker_baseline.json', walker_baseline)
        if lights:
            common.atomic_create(output / 'traffic_baseline.json', {'boundary': traffic_boundary, 'suffix': traffic_baseline})
        common.atomic_create(output / 'capture_receipt.json', receipt)
        if recorder:
            recorder.finish(output / 'recording')
            recorded_dir = output / 'recording'
            recorded_actions = json.loads((recorded_dir / 'actions.json').read_text())
            recorded_baseline = json.loads((recorded_dir / 'vehicle_reference.json').read_text())
            recorded_props = json.loads((recorded_dir / 'prop_reference.json').read_text())
            if recorded_actions['frames'] != frames or recorded_baseline != baseline or recorded_props != prop_baseline:
                raise RuntimeError('saved recording differs from source observations')
            if lights and json.loads((recorded_dir / 'traffic_reference.json').read_text()) != traffic_baseline:
                raise RuntimeError('saved traffic recording differs from source observations')
            if policy:
                recorded_vehicle_lights = json.loads((recorded_dir / 'vehicle_light_reference.json').read_text())
                if recorded_vehicle_lights != vehicle_light_baseline:
                    raise RuntimeError('saved vehicle light recording differs from source observations')
                vehicle_light_baseline = recorded_vehicle_lights
            actions, baseline, prop_baseline = recorded_actions, recorded_baseline, recorded_props
        for actor in list(actors.values()) + list(props.values()) + list(walkers.values()):
            actor.destroy()
        actors.clear()
        props.clear()
        walkers.clear()
        result = live.replay(client, output / 'checkpoint.json', actions, output / 'restore_report.json', baseline,
                             args.prop_rpc if args.dynamic_cone else None, args.port, prop_baseline,
                             traffic_baseline if lights else None, args.traffic_rpc, vehicle_light_baseline if policy else None,
                             args.walker_rpc, walker_actions if args.walker_rpc else None, walker_baseline if args.walker_rpc else None)
        print('Live capture frame {}; vehicle exact: {}; prop exact: {}; traffic exact: {}; vehicle lights exact: {}; walker within tolerance: {}'.format(
            frame, result['exact'], result['prop_exact'], result['traffic_exact'], result['vehicle_light_exact'], result['walker_within_tolerance']))
        return 0 if result['exact'] and result['prop_exact'] and result['traffic_exact'] is not False and result['vehicle_light_exact'] is not False and result['walker_within_tolerance'] is not False else 1
    finally:
        for actor in list(actors.values()) + list(props.values()) + list(walkers.values()):
            actor.destroy()
        client.get_world().apply_settings(original_settings)


if __name__ == '__main__':
    raise SystemExit(main())
