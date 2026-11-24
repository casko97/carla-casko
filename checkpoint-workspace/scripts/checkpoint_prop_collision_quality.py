#!/usr/bin/env python3
"""Compare a driven vehicle hitting a dynamic or fixed prop on a dedicated server.

This is a simulation-quality experiment, not a checkpoint or a full benchmark.
The server's vehicle sleep mode should be held constant across both runs.
"""
import argparse
import math
import subprocess
from pathlib import Path

import carla
import checkpoint_fixed_props as fixed_props
import checkpoint_scenario_bundle as common
import multi_vehicle_physics_checkpoint as multi
import vehicle_physics_checkpoint as single


def native_rpc(path, port, operation, actor_id, *extra):
    result = subprocess.run([path, str(port), operation, str(actor_id), *extra],
                            timeout=45, capture_output=True, text=True)
    if result.returncode != 0:
        raise RuntimeError('native prop {} failed: {}'.format(operation, (result.stderr or result.stdout).strip()))


def run(client, mode, report_path, checkpoint_after=None, native_prop_rpc=None, port=23000,
        portable_prop_checkpoint=None):
    if Path(report_path).exists():
        raise FileExistsError(report_path)
    if checkpoint_after is not None and (mode != 'dynamic' or not 1 <= checkpoint_after < 160):
        raise ValueError('prop restore probe requires dynamic mode and a boundary from 1 to 159')
    if native_prop_rpc is not None and checkpoint_after is None:
        raise ValueError('native prop RPC requires a checkpoint boundary')
    if portable_prop_checkpoint is not None and (native_prop_rpc is None or Path(portable_prop_checkpoint).exists()):
        raise ValueError('portable prop restore requires a native client and a new output file')
    identity = common.fingerprint()
    world = single.load_world(client, 'Town01')
    actors, sensors = {}, {}
    result = {'mode': mode, 'identity': identity, 'frames': [], 'failure': None,
              'checkpoint_after': checkpoint_after,
              'prop_restore_method': 'portable_fresh_episode' if portable_prop_checkpoint else ('native_slot' if native_prop_rpc else 'public_state'),
              'scope': 'single scripted prop impact; collision sensor is diagnostic only'}
    try:
        if mode == 'fixed':
            guard = fixed_props.prepare(client, world)
            result['fixed_map_prop_count'] = len(guard['records'])
        else:
            world.tick()  # Match the fixed-prop preparation frame.
            result['fixed_map_prop_count'] = 0
        spawn = world.get_map().get_spawn_points()[0]
        yaw = math.radians(spawn.rotation.yaw)
        target = carla.Transform(carla.Location(x=spawn.location.x + 8.0 * math.cos(yaw),
                                                  y=spawn.location.y + 8.0 * math.sin(yaw),
                                                  z=0.05),
                                 carla.Rotation(yaw=spawn.rotation.yaw))
        blueprint = world.get_blueprint_library().find('static.prop.trafficcone01')
        prop = world.try_spawn_actor(blueprint, target)
        if prop is None:
            raise RuntimeError('prop spawn failed')
        actors['prop'] = prop
        response = client.apply_batch_sync([carla.command.SetSimulatePhysics(prop.id, mode == 'dynamic')], False)
        if len(response) != 1 or response[0].error:
            raise RuntimeError('prop physics command failed')
        if mode == 'fixed':
            # Town01 cone's measured settled height in the dynamic control run.
            # Align the fixed cone with that position before the vehicle starts.
            target.location.z = 0.0001615
            response = client.apply_batch_sync([carla.command.ApplyTransform(prop.id, target)], False)
            if len(response) != 1 or response[0].error:
                raise RuntimeError('fixed prop placement failed')
        world.tick()
        actors['ego'] = single.spawn(world, 'vehicle.lincoln.mkz_2020', 0)
        sensors = multi.setup_collision_logs(world, {'ego': actors['ego']})
        initial = world.get_snapshot()
        result['prop_initial'] = single.sample(world, prop) if initial.find(prop.id) else None
        checkpoint_prop = None
        vehicle_checkpoint = None
        for index in range(160):
            control = {'brake': 1.0} if index < 20 else {'throttle': 0.4}
            state = common.execute(world, client, {'ego': actors['ego']},
                                   {'events': [], 'controls': {'ego': control}})['ego']
            frame_id = world.get_snapshot().frame
            event = multi.drain_collision_logs(world, sensors, actors, frame_id)['ego']
            result['frames'].append({'ego': state, 'prop': single.sample(world, prop), 'collision_events': event})
            if checkpoint_after == index + 1:
                if not actors['ego'].save_physics_checkpoint():
                    raise RuntimeError('vehicle checkpoint rejected')
                if native_prop_rpc:
                    native_rpc(native_prop_rpc, port, 'save', prop.id)
                if portable_prop_checkpoint:
                    native_rpc(native_prop_rpc, port, 'export', prop.id, portable_prop_checkpoint, identity['selected_files_id'])
                    vehicle_checkpoint = actors['ego'].export_physics_checkpoint(identity['selected_files_id'])
                checkpoint_prop = result['frames'][-1]['prop']
                result['checkpoint_prop'] = checkpoint_prop
                if world.get_snapshot().frame != frame_id:
                    raise RuntimeError('world advanced during checkpoint')
        result['collision_event_records'] = sum(len(frame['collision_events']) for frame in result['frames'])
        result['first_collision_frame'] = next((i + 1 for i, frame in enumerate(result['frames'])
                                                if frame['collision_events']), None)
        if checkpoint_prop is not None:
            location, rotation = checkpoint_prop['location'], checkpoint_prop['rotation']
            transform = carla.Transform(carla.Location(x=location[0], y=location[1], z=location[2]),
                                        carla.Rotation(pitch=rotation[0], yaw=rotation[1], roll=rotation[2]))
            linear, angular = checkpoint_prop['velocity'], checkpoint_prop['angular_velocity']
            if portable_prop_checkpoint:
                # A map reload destroys the source actors and checkpoint slots.
                world = single.load_world(client, 'Town01')
                actors, sensors = {}, {}
                prop = world.try_spawn_actor(world.get_blueprint_library().find('static.prop.trafficcone01'), target)
                if prop is None:
                    raise RuntimeError('fresh prop spawn failed')
                actors['prop'] = prop
                response = client.apply_batch_sync([carla.command.SetSimulatePhysics(prop.id, True)], False)
                if len(response) != 1 or response[0].error:
                    raise RuntimeError('fresh prop physics command failed')
                world.tick()
                actors['ego'] = single.spawn(world, 'vehicle.lincoln.mkz_2020', 0)
                sensors = multi.setup_collision_logs(world, {'ego': actors['ego']})
                for _ in range(20):
                    common.execute(world, client, {'ego': actors['ego']},
                                   {'events': [], 'controls': {'ego': {'brake': 1.0}}})
                    multi.drain_collision_logs(world, sensors, actors, world.get_snapshot().frame)
                native_rpc(native_prop_rpc, port, 'import', prop.id, portable_prop_checkpoint, identity['selected_files_id'])
                if not actors['ego'].import_physics_checkpoint(vehicle_checkpoint, identity['selected_files_id'], True):
                    raise RuntimeError('fresh vehicle import rejected')
            before = world.get_snapshot().frame
            if native_prop_rpc:
                native_rpc(native_prop_rpc, port, 'restore', prop.id)
            else:
                commands = [carla.command.ApplyTransform(prop.id, transform),
                            carla.command.ApplyTargetVelocity(prop.id, carla.Vector3D(*linear)),
                            carla.command.ApplyTargetAngularVelocity(prop.id, carla.Vector3D(*angular))]
                replies = client.apply_batch_sync(commands, False)
                if len(replies) != len(commands) or any(reply.error for reply in replies):
                    raise RuntimeError('prop public-state restoration failed')
            if not actors['ego'].restore_physics_checkpoint() or world.get_snapshot().frame != before:
                raise RuntimeError('vehicle restore rejected or world advanced')
            result['restore'] = {'frames': [], 'first_difference': None,
                                 'scope': 'prop state; contact, solver and sleep-filter history not saved'}
            for index in range(checkpoint_after, 160):
                control = {'brake': 1.0} if index < 20 else {'throttle': 0.4}
                state = common.execute(world, client, {'ego': actors['ego']},
                                       {'events': [], 'controls': {'ego': control}})['ego']
                frame_id = world.get_snapshot().frame
                event = multi.drain_collision_logs(world, sensors, actors, frame_id)['ego']
                observed = {'ego': state, 'prop': single.sample(world, prop), 'collision_events': event}
                result['restore']['frames'].append(observed)
                if result['restore']['first_difference'] is None and observed != result['frames'][index]:
                    result['restore']['first_difference'] = {'suffix_frame': index - checkpoint_after + 1,
                        'expected': result['frames'][index], 'actual': observed}
            result['restore']['exact'] = result['restore']['first_difference'] is None
        if common.fingerprint() != identity:
            raise RuntimeError('selected artifacts changed during experiment')
        return result
    except Exception as exc:
        result['failure'] = {'type': type(exc).__name__, 'message': str(exc)}
        raise
    finally:
        try:
            for sensor in sensors.values():
                sensor.destroy()
            for actor in actors.values():
                actor.destroy()
        finally:
            common.atomic_create(report_path, result)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=('dynamic', 'fixed'), required=True)
    parser.add_argument('--report', required=True)
    parser.add_argument('--checkpoint-after', type=int, help='also probe same-episode dynamic prop restoration')
    parser.add_argument('--native-prop-rpc', help='path to the experimental checkpoint_prop_rpc helper')
    parser.add_argument('--portable-prop-checkpoint', help='new file for a fresh-episode portable prop test')
    parser.add_argument('--port', type=int, default=23000)
    parser.add_argument('--timeout', type=float, default=180)
    args = parser.parse_args()
    client = carla.Client('127.0.0.1', args.port)
    client.set_timeout(args.timeout)
    settings = client.get_world().get_settings()
    try:
        result = run(client, args.mode, args.report, args.checkpoint_after, args.native_prop_rpc, args.port,
                     args.portable_prop_checkpoint)
        print('Collision event records: {}; first frame: {}'.format(result['collision_event_records'], result['first_collision_frame']))
        if args.checkpoint_after is not None:
            print('Prop restore exact: {}; first difference: {}'.format(
                result['restore']['exact'], result['restore']['first_difference']['suffix_frame']
                if result['restore']['first_difference'] else None))
            return 0 if result['restore']['exact'] else 1
        return 0
    finally:
        client.get_world().apply_settings(settings)


if __name__ == '__main__':
    raise SystemExit(main())
