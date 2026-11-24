#!/usr/bin/env python3
"""Probe origin guards on a dedicated large-map editor server, without contacts."""
import argparse
import json
import subprocess
from pathlib import Path

import carla
import checkpoint_scenario_bundle as common
import vehicle_physics_checkpoint as single


def origin(helper, port):
    reply = subprocess.run([helper, str(port)], capture_output=True, text=True, timeout=200)
    if reply.returncode:
        raise RuntimeError('origin query failed: {}'.format(reply.stderr.strip() or reply.stdout.strip()))
    value = json.loads(reply.stdout)
    if not isinstance(value, list) or len(value) != 3 or any(type(x) is not int for x in value):
        raise RuntimeError('invalid origin response')
    return value


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', type=int, default=36000)
    p.add_argument('--origin-rpc', required=True)
    p.add_argument('--report', required=True)
    p.add_argument('--map', required=True, help='large map selected for an explicit rebase experiment')
    p.add_argument('--timeout', type=float, default=360)
    args = p.parse_args()
    if Path(args.report).exists():
        raise FileExistsError(args.report)
    client = carla.Client('127.0.0.1', args.port)
    client.set_timeout(args.timeout)
    settings = client.get_world().get_settings()
    world, vehicle = None, None
    report = {'map': args.map, 'failure': None, 'stages': [],
              'scope': 'one airborne manually controlled vehicle and an induced world-origin shift'}
    try:
        world = single.load_world(client, args.map)
        initial = origin(args.origin_rpc, args.port)
        report['initial_origin_cm'] = initial
        if initial != [0, 0, 0]:
            raise RuntimeError('map did not start at origin zero')
        bp = world.get_blueprint_library().find('vehicle.lincoln.mkz_2020')
        bp.set_attribute('role_name', 'hero')
        bp.set_attribute('sticky_control', 'false')
        vehicle = world.try_spawn_actor(bp, carla.Transform(carla.Location(x=0, y=0, z=100)))
        if vehicle is None:
            raise RuntimeError('airborne vehicle spawn failed')
        for _ in range(5):
            world.tick()
        if origin(args.origin_rpc, args.port) != [0, 0, 0]:
            raise RuntimeError('origin shifted before baseline save')
        if not vehicle.save_physics_checkpoint():
            raise RuntimeError('baseline save rejected')
        baseline = vehicle.export_physics_checkpoint('origin-guard-probe')
        if not baseline:
            raise RuntimeError('baseline export returned no data')
        report['zero_origin_restore_accepted'] = vehicle.restore_physics_checkpoint()
        if not report['zero_origin_restore_accepted']:
            raise RuntimeError('baseline restore rejected at zero origin')
        vehicle.set_transform(carla.Transform(carla.Location(x=3000, y=0, z=100)))
        shifted = [0, 0, 0]
        for _ in range(30):
            world.tick()
            shifted = origin(args.origin_rpc, args.port)
            if shifted != [0, 0, 0]:
                break
        report['shifted_origin_cm'] = shifted
        if shifted == [0, 0, 0]:
            raise RuntimeError('map did not perform the expected rebase')
        report['old_slot_restore_accepted'] = vehicle.restore_physics_checkpoint()
        if report['old_slot_restore_accepted']:
            raise RuntimeError('restore accepted a different world origin')
        report['shifted_save_accepted'] = vehicle.save_physics_checkpoint()
        if not report['shifted_save_accepted']:
            raise RuntimeError('shifted save rejected for an unrelated reason')
        try:
            vehicle.export_physics_checkpoint('origin-guard-probe')
            report['shifted_export_rejected'] = False
        except RuntimeError as exc:
            report['shifted_export_error'] = str(exc)
            report['shifted_export_rejected'] = 'vehicle has no exportable physics checkpoint' in str(exc)
        if not report['shifted_export_rejected']:
            raise RuntimeError('portable export accepted a shifted source')
        print('Origin guard rejected restore and portable export after rebase')
        return 0
    except Exception as exc:
        report['failure'] = {'type': type(exc).__name__, 'message': str(exc)}
        raise
    finally:
        try:
            if vehicle is not None:
                vehicle.destroy()
            client.get_world().apply_settings(settings)
        finally:
            common.atomic_create(args.report, report)


if __name__ == '__main__':
    raise SystemExit(main())
