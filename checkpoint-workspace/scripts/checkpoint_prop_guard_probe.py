#!/usr/bin/env python3
"""Exercise native prop checkpoint rejection conditions on a dedicated server."""
import argparse
import subprocess
from pathlib import Path

import carla
import checkpoint_scenario_bundle as common
import vehicle_physics_checkpoint as single


def call(helper, port, operation, actor_id, *extra):
    result = subprocess.run([helper, str(port), operation, str(actor_id), *extra], capture_output=True, text=True, timeout=45)
    if result.returncode not in (0, 1):
        raise RuntimeError('RPC failed: {}'.format(result.stderr.strip()))
    return result.returncode == 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--native-prop-rpc', required=True)
    parser.add_argument('--portable-file', help='new file for portable export and import checks')
    parser.add_argument('--incompatible-config', help='valid checkpoint with intentionally incompatible mass')
    parser.add_argument('--report', required=True)
    parser.add_argument('--port', type=int, default=23000)
    args = parser.parse_args()
    if Path(args.report).exists():
        raise FileExistsError(args.report)
    if args.portable_file and (Path(args.portable_file).exists() or Path(args.portable_file + '.after-rejected').exists()):
        raise FileExistsError('portable probe output exists')
    if args.incompatible_config and not args.portable_file:
        parser.error('--incompatible-config requires --portable-file')
    client = carla.Client('127.0.0.1', args.port)
    client.set_timeout(180)
    settings = client.get_world().get_settings()
    actors = []
    result = {'checks': {}, 'failure': None}
    try:
        world = single.load_world(client, 'Town01')
        spawn = world.get_map().get_spawn_points()[0]
        target = carla.Transform(carla.Location(x=spawn.location.x, y=spawn.location.y + 8, z=0.05), carla.Rotation())
        prop = world.try_spawn_actor(world.get_blueprint_library().find('static.prop.trafficcone01'), target)
        if prop is None:
            raise RuntimeError('prop spawn failed')
        actors.append(prop)
        vehicle = single.spawn(world, 'vehicle.lincoln.mkz_2020', 0)
        actors.append(vehicle)
        replies = client.apply_batch_sync([carla.command.SetSimulatePhysics(prop.id, True)], False)
        if len(replies) != 1 or replies[0].error:
            raise RuntimeError('enable prop physics failed')
        world.tick()
        def check(name, actual, expected):
            result['checks'][name] = actual == expected
            if actual != expected:
                raise RuntimeError('unexpected result: {}'.format(name))
        check('restore_without_save_rejected', call(args.native_prop_rpc, args.port, 'restore', prop.id), False)
        check('vehicle_rejected', call(args.native_prop_rpc, args.port, 'save', vehicle.id), False)
        check('prop_save_accepted', call(args.native_prop_rpc, args.port, 'save', prop.id), True)
        build_id = 'prop-guard-probe-v1'
        if args.portable_file:
            check('portable_export_accepted', call(args.native_prop_rpc, args.port, 'export', prop.id,
                                                    args.portable_file, build_id), True)
            check('wrong_build_rejected', call(args.native_prop_rpc, args.port, 'import', prop.id,
                                               args.portable_file, 'different-build'), False)
            if args.incompatible_config:
                check('incompatible_mass_rejected', call(args.native_prop_rpc, args.port, 'import', prop.id,
                                                         args.incompatible_config, build_id), False)
            check('export_after_rejection_accepted', call(args.native_prop_rpc, args.port, 'export', prop.id,
                  args.portable_file + '.after-rejected', build_id), True)
            check('rejected_import_preserves_slot', Path(args.portable_file).read_bytes() ==
                  Path(args.portable_file + '.after-rejected').read_bytes(), True)
        cfg = world.get_settings()
        previous_substeps = cfg.max_substeps
        cfg.max_substeps += 1
        world.apply_settings(cfg)
        check('changed_settings_rejected', call(args.native_prop_rpc, args.port, 'restore', prop.id), False)
        cfg.max_substeps = previous_substeps
        world.apply_settings(cfg)
        check('matching_restore_accepted', call(args.native_prop_rpc, args.port, 'restore', prop.id), True)
        replies = client.apply_batch_sync([carla.command.SetSimulatePhysics(prop.id, False)], False)
        if len(replies) != 1 or replies[0].error:
            raise RuntimeError('disable prop physics failed')
        check('save_without_physics_rejected', call(args.native_prop_rpc, args.port, 'save', prop.id), False)
        replies = client.apply_batch_sync([carla.command.SetSimulatePhysics(prop.id, True)], False)
        if len(replies) != 1 or replies[0].error:
            raise RuntimeError('re-enable prop physics failed')
        world.tick()
        check('failed_save_invalidated_slot', call(args.native_prop_rpc, args.port, 'restore', prop.id), False)
        check('resave_accepted', call(args.native_prop_rpc, args.port, 'save', prop.id), True)
        prop_id = prop.id
        prop.destroy()
        actors.remove(prop)
        check('destroyed_actor_rejected', call(args.native_prop_rpc, args.port, 'restore', prop_id), False)
        next_prop = world.try_spawn_actor(world.get_blueprint_library().find('static.prop.trafficcone01'), target)
        if next_prop is None:
            raise RuntimeError('second prop spawn failed')
        actors.append(next_prop)
        replies = client.apply_batch_sync([carla.command.SetSimulatePhysics(next_prop.id, True)], False)
        if len(replies) != 1 or replies[0].error:
            raise RuntimeError('enable second prop physics failed')
        world.tick()
        check('second_prop_save_accepted', call(args.native_prop_rpc, args.port, 'save', next_prop.id), True)
        old_id = next_prop.id
        single.load_world(client, 'Town01')
        actors.clear()  # The map reload destroys the old episode's actors.
        check('previous_episode_rejected', call(args.native_prop_rpc, args.port, 'restore', old_id), False)
        if args.portable_file:
            world = client.get_world()
            fresh = world.try_spawn_actor(world.get_blueprint_library().find('static.prop.trafficcone01'), target)
            if fresh is None:
                raise RuntimeError('fresh prop spawn failed')
            actors.append(fresh)
            replies = client.apply_batch_sync([carla.command.SetSimulatePhysics(fresh.id, True)], False)
            if len(replies) != 1 or replies[0].error:
                raise RuntimeError('fresh prop physics failed')
            world.tick()
            check('fresh_actor_import_accepted', call(args.native_prop_rpc, args.port, 'import', fresh.id,
                                                      args.portable_file, build_id), True)
            check('fresh_actor_restore_accepted', call(args.native_prop_rpc, args.port, 'restore', fresh.id), True)
        return 0
    except Exception as exc:
        result['failure'] = {'type': type(exc).__name__, 'message': str(exc)}
        raise
    finally:
        try:
            for actor in actors:
                actor.destroy()
            client.get_world().apply_settings(settings)
        finally:
            common.atomic_create(args.report, result)


if __name__ == '__main__':
    raise SystemExit(main())
