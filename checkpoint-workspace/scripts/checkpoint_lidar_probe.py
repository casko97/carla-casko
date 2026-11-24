#!/usr/bin/env python3
"""Compare LiDAR samples after vehicle checkpoint restoration on a dedicated server.

The fixed mode suppresses LiDAR noise/dropout and uses one full rotation per
simulation tick. These settings change the sensor model; this is a diagnostic,
not a general sensor checkpoint.
"""
import argparse
import hashlib
import math
import queue
import struct
from pathlib import Path

import carla
import checkpoint_scenario_bundle as common
import vehicle_physics_checkpoint as single


def setup_sensor(world, vehicle, mode):
    bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
    attrs = {'channels': '16', 'points_per_second': '32000', 'range': '50',
             'upper_fov': '10', 'lower_fov': '-30', 'sensor_tick': '0'}
    if mode == 'fixed':
        attrs.update({'rotation_frequency': '20', 'noise_stddev': '0',
                      'dropoff_general_rate': '0', 'dropoff_zero_intensity': '0'})
    for key, value in attrs.items():
        if not bp.has_attribute(key):
            raise ValueError('sensor lacks {}'.format(key))
        bp.set_attribute(key, value)
    transform = carla.Transform(carla.Location(x=0.0, y=0.0, z=2.0))
    sensor = world.spawn_actor(bp, transform, attach_to=vehicle)
    measurements = queue.Queue()
    sensor.listen(measurements.put)
    return sensor, measurements, dict(sensor.attributes)


def await_frame(measurements, frame):
    while True:
        sample = measurements.get(timeout=30)
        if sample.frame == frame:
            return sample
        if sample.frame > frame:
            raise RuntimeError('missing lidar frame {}'.format(frame))


def step(client, world, vehicle, measurements, control):
    previous = world.get_snapshot().frame
    replies = client.apply_batch_sync([carla.command.ApplyVehicleControl(vehicle.id, carla.VehicleControl(**control))], False)
    if len(replies) != 1 or replies[0].error:
        raise RuntimeError('vehicle control failed')
    frame = world.tick()
    if frame != previous + 1:
        raise RuntimeError('unexpected tick')
    measurement = await_frame(measurements, frame)
    raw = bytes(measurement.raw_data)
    if len(raw) % 16:
        raise RuntimeError('invalid lidar point data')
    if world.get_snapshot().frame != frame:
        raise RuntimeError('world advanced during sensor sampling')
    return {'vehicle': single.sample(world, vehicle), 'points': len(raw) // 16,
            'sha256': hashlib.sha256(raw).hexdigest(), 'horizontal_angle': measurement.horizontal_angle}, raw


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--mode', choices=('fixed', 'default'), required=True)
    p.add_argument('--report', required=True)
    p.add_argument('--port', type=int, default=33000)
    args = p.parse_args()
    if Path(args.report).exists():
        raise FileExistsError(args.report)
    client = carla.Client('127.0.0.1', args.port)
    client.set_timeout(180)
    original = client.get_world().get_settings()
    actors = []
    result = {'mode': args.mode, 'scope': 'ray-cast lidar raw points and sampled vehicle state; no other sensors',
              'failure': None, 'baseline': [], 'replay': [], 'first_raw_difference': None,
              'first_vehicle_difference': None, 'max_point_component_difference_m': 0.0,
              'count_mismatch_frames': 0}
    try:
        result['identity'] = common.fingerprint()
        world = single.load_world(client, 'Town01')
        vehicle = single.spawn(world, 'vehicle.lincoln.mkz_2020', 0)
        actors.append(vehicle)
        sensor, q, result['attributes'] = setup_sensor(world, vehicle, args.mode)
        actors.append(sensor)
        for i in range(60):
            control = {'brake': 1.0} if i < 20 else {'throttle': 0.25, 'steer': 0.04}
            step(client, world, vehicle, q, control)
        frame = world.get_snapshot().frame
        if not vehicle.save_physics_checkpoint():
            raise RuntimeError('vehicle checkpoint rejected')
        vehicle_data = vehicle.export_physics_checkpoint(result['identity']['selected_files_id'])
        if world.get_snapshot().frame != frame:
            raise RuntimeError('world advanced during capture')
        result['checkpoint_frame'] = frame
        actions = [{'throttle': 0.3, 'steer': 0.08} for _ in range(40)]
        baseline_raw = []
        for action in actions:
            sample, raw = step(client, world, vehicle, q, action)
            result['baseline'].append(sample)
            baseline_raw.append(raw)
        sensor.stop()
        for actor in reversed(actors):
            actor.destroy()
        actors = []
        world = single.load_world(client, 'Town01')
        vehicle = single.spawn(world, 'vehicle.lincoln.mkz_2020', 0)
        actors.append(vehicle)
        for _ in range(20):
            common.execute(world, client, {'ego': vehicle}, {'events': [], 'controls': {'ego': {'brake': 1.0}}})
        frame = world.get_snapshot().frame
        if not vehicle.import_physics_checkpoint(vehicle_data, result['identity']['selected_files_id'], True):
            raise RuntimeError('vehicle checkpoint import rejected')
        if not vehicle.restore_physics_checkpoint():
            raise RuntimeError('vehicle checkpoint restore rejected')
        if world.get_snapshot().frame != frame:
            raise RuntimeError('world advanced during restore')
        sensor, q, target_attributes = setup_sensor(world, vehicle, args.mode)
        actors.append(sensor)
        if target_attributes != result['attributes']:
            raise RuntimeError('sensor attributes changed')
        for i, action in enumerate(actions):
            sample, raw = step(client, world, vehicle, q, action)
            result['replay'].append(sample)
            if sample['vehicle'] != result['baseline'][i]['vehicle'] and result['first_vehicle_difference'] is None:
                result['first_vehicle_difference'] = i + 1
            if raw != baseline_raw[i] and result['first_raw_difference'] is None:
                result['first_raw_difference'] = i + 1
            if len(raw) != len(baseline_raw[i]):
                result['count_mismatch_frames'] += 1
            else:
                for left, right in zip(struct.iter_unpack('<ffff', baseline_raw[i]), struct.iter_unpack('<ffff', raw)):
                    result['max_point_component_difference_m'] = max(result['max_point_component_difference_m'],
                        *(abs(left[k] - right[k]) for k in range(3)))
        if common.fingerprint() != result['identity']:
            raise RuntimeError('selected artifacts changed during probe')
        result['vehicle_exact'] = result['first_vehicle_difference'] is None
        result['raw_exact'] = result['first_raw_difference'] is None
        print('mode {}; vehicle exact {}; lidar raw exact {}; count mismatches {}'.format(
            args.mode, result['vehicle_exact'], result['raw_exact'], result['count_mismatch_frames']))
    except Exception as exc:
        result['failure'] = {'type': type(exc).__name__, 'message': str(exc)}
        raise
    finally:
        for actor in reversed(actors):
            if actor.type_id.startswith('sensor.'):
                actor.stop()
            actor.destroy()
        common.atomic_create(args.report, result)
        client.get_world().apply_settings(original)


if __name__ == '__main__':
    main()
