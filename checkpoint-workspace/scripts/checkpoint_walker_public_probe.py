#!/usr/bin/env python3
"""Measure public walker-state restoration alongside a vehicle on a dedicated server.

This is a diagnostic, not a portable walker checkpoint. It teleports a fresh
walker and requests a target velocity using public CARLA commands; character
movement internals, animation and AI state are not saved.
"""
import argparse
import json
import math
import subprocess
import tempfile
from pathlib import Path

import carla
import checkpoint_scenario_bundle as common
import vehicle_physics_checkpoint as single


def walker_spawn(world):
    point = world.get_map().get_spawn_points()[0]
    yaw = math.radians(point.rotation.yaw)
    transform = carla.Transform(carla.Location(x=point.location.x - 5.0 * math.sin(yaw),
                                               y=point.location.y + 5.0 * math.cos(yaw), z=1.0),
                                carla.Rotation(yaw=point.rotation.yaw))
    walker = world.try_spawn_actor(world.get_blueprint_library().find('walker.pedestrian.0001'), transform)
    if walker is None:
        raise RuntimeError('walker spawn failed')
    return walker, transform


def step(world, client, vehicle, walker, speed, direction):
    frame = world.get_snapshot().frame
    control = carla.WalkerControl(direction=direction, speed=speed, jump=False)
    commands = [carla.command.ApplyVehicleControl(vehicle.id, carla.VehicleControl(brake=1.0)),
                carla.command.ApplyWalkerControl(walker.id, control)]
    replies = client.apply_batch_sync(commands, False)
    if len(replies) != len(commands) or any(r.error for r in replies):
        raise RuntimeError('control batch failed')
    if world.tick() != frame + 1:
        raise RuntimeError('world tick mismatch')
    result = {'walker': single.sample(world, walker), 'vehicle': single.sample(world, vehicle)}
    if world.get_snapshot().frame != frame + 1:
        raise RuntimeError('world advanced while sampling')
    return result


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--report', required=True)
    p.add_argument('--mode', choices=('moving', 'idle'), required=True)
    p.add_argument('--native-velocity-rpc', help='optional narrow native velocity probe helper')
    p.add_argument('--port', type=int, default=30000)
    args = p.parse_args()
    if Path(args.report).exists():
        raise FileExistsError(args.report)
    client = carla.Client('127.0.0.1', args.port)
    client.set_timeout(180)
    original_settings = client.get_world().get_settings()
    actors = []
    temporary = tempfile.TemporaryDirectory(prefix='carla-walker-probe-')
    velocity_file = Path(temporary.name) / 'velocity.msgpack'
    report = {'mode': args.mode, 'method': ('fresh episode; public transform and native velocity probe' if args.native_velocity_rpc
                                            else 'fresh episode; public transform and target velocity'),
              'scope': 'sampled root state; animation, controllers, contacts and character movement internals omitted',
              'failure': None, 'first_difference': None, 'baseline': [], 'replay': []}
    try:
        identity = common.fingerprint()
        report['identity'] = identity
        world = single.load_world(client, 'Town01')
        vehicle = single.spawn(world, 'vehicle.lincoln.mkz_2020', 0)
        walker, initial = walker_spawn(world)
        actors = [vehicle, walker]
        yaw = math.radians(initial.rotation.yaw)
        direction = carla.Vector3D(x=math.cos(yaw), y=math.sin(yaw), z=0.0)
        speed = 1.4 if args.mode == 'moving' else 0.0
        for _ in range(60):
            step(world, client, vehicle, walker, speed, direction)
        source_frame = world.get_snapshot().frame
        source_walker = single.sample(world, walker)
        if args.native_velocity_rpc:
            subprocess.run([args.native_velocity_rpc, str(args.port), 'capture', str(walker.id), str(velocity_file)],
                           check=True, timeout=45, capture_output=True, text=True)
        if not vehicle.save_physics_checkpoint():
            raise RuntimeError('vehicle checkpoint rejected')
        vehicle_data = vehicle.export_physics_checkpoint(identity['selected_files_id'])
        if world.get_snapshot().frame != source_frame:
            raise RuntimeError('world advanced during capture')
        report['checkpoint_frame'] = source_frame
        report['checkpoint_walker'] = source_walker
        for _ in range(40):
            report['baseline'].append(step(world, client, vehicle, walker, speed, direction))
        for actor in actors:
            actor.destroy()
        actors = []
        world = single.load_world(client, 'Town01')
        vehicle = single.spawn(world, 'vehicle.lincoln.mkz_2020', 0)
        walker, _ = walker_spawn(world)
        actors = [vehicle, walker]
        for _ in range(20):
            step(world, client, vehicle, walker, 0.0, direction)
        frame = world.get_snapshot().frame
        if not vehicle.import_physics_checkpoint(vehicle_data, identity['selected_files_id'], True):
            raise RuntimeError('vehicle checkpoint import rejected')
        if not vehicle.restore_physics_checkpoint():
            raise RuntimeError('vehicle restore rejected')
        x, y, z = source_walker['location']
        pitch, yaw, roll = source_walker['rotation']
        transform = carla.Transform(carla.Location(x=x, y=y, z=z), carla.Rotation(pitch=pitch, yaw=yaw, roll=roll))
        velocity = carla.Vector3D(*source_walker['velocity'])
        replies = client.apply_batch_sync([carla.command.ApplyTransform(walker.id, transform),
                                          carla.command.ApplyTargetVelocity(walker.id, velocity)], False)
        if len(replies) != 2 or any(r.error for r in replies):
            raise RuntimeError('walker public-state restore command rejected')
        if args.native_velocity_rpc:
            subprocess.run([args.native_velocity_rpc, str(args.port), 'restore', str(walker.id), str(velocity_file)],
                           check=True, timeout=45, capture_output=True, text=True)
        if world.get_snapshot().frame != frame:
            raise RuntimeError('world advanced during restore')
        for index in range(40):
            actual = step(world, client, vehicle, walker, speed, direction)
            report['replay'].append(actual)
            if report['first_difference'] is None and actual != report['baseline'][index]:
                report['first_difference'] = {'suffix_frame': index + 1,
                                              'expected': report['baseline'][index], 'actual': actual}
        if common.fingerprint() != identity:
            raise RuntimeError('selected artifacts changed during probe')
        report['exact'] = report['first_difference'] is None
        print('Walker {} public restore exact: {}; first difference: {}'.format(
            args.mode, report['exact'], None if report['exact'] else report['first_difference']['suffix_frame']))
    except Exception as exc:
        report['failure'] = {'type': type(exc).__name__, 'message': str(exc)}
        raise
    finally:
        for actor in actors:
            actor.destroy()
        common.atomic_create(args.report, report)
        client.get_world().apply_settings(original_settings)
        temporary.cleanup()


if __name__ == '__main__':
    main()
