#!/usr/bin/env python3
"""Compare an uninterrupted vehicle rollout with restores and prefix replay.

Run only against a dedicated server built from this branch. The script loads a
map, changes world settings and destroys its actor; it must not share a server.
"""

import argparse
import json
import math
import platform
import sys

import carla


def vector(value):
    return [value.x, value.y, value.z]


def export_checkpoint(vehicle, build_id, path):
    data = vehicle.export_physics_checkpoint(build_id)
    with open(path, "wb") as output:
        output.write(data)
    return {"path": str(path), "bytes": len(data)}


def make_journal(warmup_steps, prefix_steps, suffix_steps, profile):
    """Return explicit, frame-indexed commands independent of the CARLA API."""
    warmup = [{"brake": 1.0} for _ in range(warmup_steps)]
    if profile == "straight":
        prefix = [{"throttle": 0.35} for _ in range(prefix_steps)]
        suffix = [{"throttle": 0.35, "steer": 0.08} for _ in range(suffix_steps)]
    elif profile == "maneuver":
        if suffix_steps < 4:
            raise ValueError("maneuver profile requires at least four suffix steps")
        prefix = [{"throttle": 0.35, "manual_gear_shift": True, "gear": 1}
                  for _ in range(prefix_steps)]
        suffix = []
        for i in range(suffix_steps):
            phase = min(3, (4 * i) // suffix_steps)
            suffix.append((
                {"throttle": 0.4, "steer": 0.12, "manual_gear_shift": True, "gear": 1},
                {"throttle": 0.35, "steer": -0.12, "manual_gear_shift": True, "gear": 2},
                {"brake": 0.45, "steer": -0.08, "manual_gear_shift": True, "gear": 2},
                {"throttle": 0.2, "manual_gear_shift": True, "gear": 1},
            )[phase].copy())
    else:
        raise ValueError("unknown control profile: {}".format(profile))
    return {"warmup": warmup, "prefix": prefix, "suffix": suffix}


def sample(world, actor):
    snapshot = world.get_snapshot().find(actor.id)
    if snapshot is None:
        raise RuntimeError("vehicle missing from world snapshot")
    transform = snapshot.get_transform()
    rotation = transform.rotation
    return {
        "location": vector(transform.location),
        "rotation": [rotation.pitch, rotation.yaw, rotation.roll],
        "velocity": vector(snapshot.get_velocity()),
        "angular_velocity": vector(snapshot.get_angular_velocity()),
        "acceleration": vector(snapshot.get_acceleration()),
    }


def step(client, world, vehicle, control):
    # Submit exactly one command, check its acknowledgement, then tick once.
    responses = client.apply_batch_sync([carla.command.ApplyVehicleControl(vehicle.id, control)], False)
    if len(responses) != 1 or responses[0].error:
        raise RuntimeError("control command failed: {}".format(responses))
    previous_frame = world.get_snapshot().frame
    tick_frame = world.tick()
    if tick_frame != previous_frame + 1:
        raise RuntimeError("unexpected world tick: another client may be ticking")
    return sample(world, vehicle)


def run_journal(client, world, vehicle, commands, trace=None):
    if trace is None:
        trace = []
    for control in commands:
        trace.append(step(client, world, vehicle, carla.VehicleControl(**control)))
    return trace


def compare(reference, candidate):
    if len(reference) != len(candidate):
        raise ValueError("rollouts have different lengths")
    first = None
    maxima = {name: 0.0 for name in reference[0]} if reference else {}
    for frame, (expected, actual) in enumerate(zip(reference, candidate), 1):
        if expected.keys() != actual.keys():
            raise ValueError("rollout fields differ")
        for name, values in expected.items():
            if len(values) != len(actual[name]):
                raise ValueError("rollout field dimensions differ: {}".format(name))
            for index, (a, b) in enumerate(zip(values, actual[name])):
                if not math.isfinite(a) or not math.isfinite(b):
                    raise ValueError("non-finite {} at suffix frame {}".format(name, frame))
                error = abs(a - b)
                maxima[name] = max(maxima[name], error)
                if first is None and a != b:
                    first = {"suffix_frame": frame, "field": name,
                             "component": index, "expected": a, "actual": b}
    return {"exact": first is None, "first_difference": first, "max_absolute_error": maxima}


def load_world(client, map_name):
    world = client.get_world()
    settings = world.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = 0.05
    settings.substepping = True
    settings.max_substep_delta_time = 0.01
    settings.max_substeps = 10
    settings.deterministic_ragdolls = True
    world.apply_settings(settings)
    return client.load_world(map_name, reset_settings=False)


def spawn(world, blueprint_id, spawn_index):
    blueprint = world.get_blueprint_library().find(blueprint_id)
    if blueprint.has_attribute("sticky_control"):
        blueprint.set_attribute("sticky_control", "false")
    points = world.get_map().get_spawn_points()
    if not 0 <= spawn_index < len(points):
        raise ValueError("spawn index is outside the map's spawn-point list")
    vehicle = world.try_spawn_actor(blueprint, points[spawn_index])
    if vehicle is None:
        raise RuntimeError("vehicle spawn failed")
    return vehicle


def collect_manifest(client, world, vehicle, spawn_index, build_id, spawn_transform=None):
    settings = world.get_settings()
    scene_flags = world.get_physx_scene_flags()
    if spawn_transform is None:
        spawn_transform = world.get_map().get_spawn_points()[spawn_index]
    rotation = spawn_transform.rotation
    return {
        "client_version": client.get_client_version(),
        "server_version": client.get_server_version(),
        "build_id": build_id,
        "python_version": platform.python_version(),
        "carla_module": getattr(carla, "__file__", None),
        "map": world.get_map().name,
        "vehicle": vehicle.type_id,
        "vehicle_attributes": dict(vehicle.attributes),
        "physics_control": str(vehicle.get_physics_control()),
        "physx_scene_flags": scene_flags,
        "physx_scene_flags_physx_3_4": {
            "enhanced_determinism": bool(scene_flags & (1 << 20)),
            "gpu_dynamics": bool(scene_flags & (1 << 19)),
            "stabilization": bool(scene_flags & (1 << 14)),
            "contact_cache_disabled": bool(scene_flags & (1 << 11)),
            "persistent_contact_manifolds": bool(scene_flags & (1 << 9)),
        },
        "spawn_index": spawn_index,
        "spawn_location": vector(spawn_transform.location),
        "spawn_rotation": [rotation.pitch, rotation.yaw, rotation.roll],
        "world_settings": {
            "synchronous_mode": settings.synchronous_mode,
            "fixed_delta_seconds": settings.fixed_delta_seconds,
            "substepping": settings.substepping,
            "max_substep_delta_time": settings.max_substep_delta_time,
            "max_substeps": settings.max_substeps,
            "deterministic_ragdolls": settings.deterministic_ragdolls,
            "no_rendering_mode": settings.no_rendering_mode,
        },
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--timeout", type=float, default=30.0, help="client RPC timeout in seconds")
    parser.add_argument("--map", default="Town03")
    parser.add_argument("--vehicle", default="vehicle.lincoln.mkz_2020")
    parser.add_argument("--spawn-index", type=int, default=0)
    parser.add_argument("--warmup", type=int, default=20)
    parser.add_argument("--prefix-steps", type=int, default=60)
    parser.add_argument("--suffix-steps", type=int, default=80)
    parser.add_argument("--repetitions", type=int, default=3)
    parser.add_argument("--profile", choices=("straight", "maneuver"), default="straight")
    parser.add_argument("--report", help="write the complete manifest, command journal and traces as JSON")
    parser.add_argument("--build-id", help="user-supplied server, engine and content build identifiers")
    parser.add_argument("--export-checkpoint", help="write the saved vehicle values as MessagePack bytes")
    parser.add_argument("--test-portable-import", action="store_true",
                        help="import into a new vehicle after reloading; assumes source world origin is zero")
    args = parser.parse_args()
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("timeout must be a positive finite number")
    if min(args.warmup, args.prefix_steps) < 0 or min(args.suffix_steps, args.repetitions) < 1:
        parser.error("warmup and prefix must be non-negative; suffix and repetitions must be positive")
    if (args.export_checkpoint or args.test_portable_import) and not args.build_id:
        parser.error("checkpoint export or import testing requires --build-id")
    try:
        journal = make_journal(args.warmup, args.prefix_steps, args.suffix_steps, args.profile)
    except ValueError as error:
        parser.error(str(error))

    client = carla.Client(args.host, args.port)
    client.set_timeout(args.timeout)
    original_settings = client.get_world().get_settings()
    world = None
    vehicle = None
    traces = {"warmup": [], "prefix": [], "baseline_suffix": [], "restore_suffixes": [],
              "replay_warmup": [], "replay_prefix": [], "replay_suffix": [], "portable_import_suffix": []}
    report = {"summary": None, "manifest": None, "replay_manifest": None,
              "command_journal": journal, "traces": traces, "failure": None,
              "exported_checkpoint": None}
    stage = "load_world"
    try:
        world = load_world(client, args.map)
        stage = "spawn"
        vehicle = spawn(world, args.vehicle, args.spawn_index)
        stage = "warmup"
        warmup_samples = run_journal(client, world, vehicle, journal["warmup"], traces["warmup"])
        stage = "manifest"
        manifest = collect_manifest(client, world, vehicle, args.spawn_index, args.build_id)
        report["manifest"] = manifest
        stage = "prefix"
        prefix_samples = run_journal(client, world, vehicle, journal["prefix"], traces["prefix"])
        checkpoint_frame = world.get_snapshot().frame
        stage = "save_checkpoint"
        if not vehicle.save_physics_checkpoint():
            raise RuntimeError("checkpoint rejected by server (unsupported vehicle or settings)")
        checkpoint_bytes = None
        if args.test_portable_import:
            stage = "export_checkpoint"
            checkpoint_bytes = vehicle.export_physics_checkpoint(args.build_id)
            if args.export_checkpoint:
                with open(args.export_checkpoint, "wb") as output:
                    output.write(checkpoint_bytes)
                report["exported_checkpoint"] = {"path": str(args.export_checkpoint), "bytes": len(checkpoint_bytes)}
        elif args.export_checkpoint:
            stage = "export_checkpoint"
            report["exported_checkpoint"] = export_checkpoint(vehicle, args.build_id, args.export_checkpoint)
        stage = "baseline_suffix"
        baseline = run_journal(client, world, vehicle, journal["suffix"], traces["baseline_suffix"])
        comparisons = []
        for repetition in range(args.repetitions):
            stage = "restore_{}".format(repetition)
            if not vehicle.restore_physics_checkpoint():
                raise RuntimeError("restore rejected by server")
            stage = "restore_suffix_{}".format(repetition)
            rollout = []
            traces["restore_suffixes"].append(rollout)
            run_journal(client, world, vehicle, journal["suffix"], rollout)
            comparisons.append(compare(baseline, rollout))

        # Prefix replay is an independent reference, not CARLA recorder playback.
        stage = "replay_load_world"
        world = load_world(client, args.map)
        stage = "replay_spawn"
        vehicle = spawn(world, args.vehicle, args.spawn_index)
        stage = "replay_warmup"
        replay_warmup = run_journal(client, world, vehicle, journal["warmup"], traces["replay_warmup"])
        stage = "replay_manifest"
        replay_manifest = collect_manifest(client, world, vehicle, args.spawn_index, args.build_id)
        report["replay_manifest"] = replay_manifest
        stage = "replay_prefix"
        replay_prefix = run_journal(client, world, vehicle, journal["prefix"], traces["replay_prefix"])
        stage = "replay_suffix"
        replay_suffix = run_journal(client, world, vehicle, journal["suffix"], traces["replay_suffix"])
        portable_import = None
        if args.test_portable_import:
            stage = "portable_load_world"
            world = load_world(client, args.map)
            stage = "portable_spawn"
            vehicle = spawn(world, args.vehicle, args.spawn_index)
            stage = "portable_warmup"
            run_journal(client, world, vehicle, journal["warmup"])
            stage = "portable_import"
            if not vehicle.import_physics_checkpoint(checkpoint_bytes, args.build_id, True):
                raise RuntimeError("portable checkpoint import rejected by server")
            if vehicle.import_physics_checkpoint(checkpoint_bytes, args.build_id + "-mismatch", True):
                raise RuntimeError("portable checkpoint accepted a mismatched build identifier")
            if vehicle.import_physics_checkpoint(checkpoint_bytes, args.build_id, False):
                raise RuntimeError("portable checkpoint accepted an unconfirmed source origin")
            try:
                vehicle.import_physics_checkpoint(checkpoint_bytes + b"trailing", args.build_id, True)
            except ValueError:
                pass
            else:
                raise RuntimeError("portable checkpoint accepted trailing bytes")
            if not vehicle.restore_physics_checkpoint():
                raise RuntimeError("portable checkpoint restore rejected by server")
            stage = "portable_suffix"
            imported_suffix = run_journal(client, world, vehicle, journal["suffix"], traces["portable_import_suffix"])
            portable_import = compare(baseline, imported_suffix)
        stage = "compare"
        result = {
            "map": args.map, "vehicle": args.vehicle, "spawn_index": args.spawn_index,
            "client_version": client.get_client_version(), "server_version": client.get_server_version(),
            "fixed_delta_seconds": 0.05, "checkpoint_frame": checkpoint_frame,
            "manifest_matches_replay": manifest == replay_manifest,
            "warmup_steps": args.warmup, "prefix_steps": args.prefix_steps,
            "suffix_steps": args.suffix_steps, "repetitions": args.repetitions, "profile": args.profile,
            "restores": comparisons, "prefix_replay_warmup": compare(warmup_samples, replay_warmup),
            "prefix_replay_prefix": compare(prefix_samples, replay_prefix),
            "prefix_replay_suffix": compare(baseline, replay_suffix),
        }
        if portable_import is not None:
            result["portable_import"] = portable_import
        report["summary"] = result
        print(json.dumps(result, indent=2, sort_keys=True))
        replay_matches = all(result[key]["exact"] for key in
                             ("prefix_replay_warmup", "prefix_replay_prefix", "prefix_replay_suffix"))
        portable_matches = portable_import is None or portable_import["exact"]
        return 0 if all(item["exact"] for item in comparisons) and replay_matches and portable_matches and result["manifest_matches_replay"] else 1
    except Exception as error:
        report["failure"] = {"stage": stage, "type": type(error).__name__, "message": str(error)}
        raise
    finally:
        try:
            if args.report:
                with open(args.report, "w", encoding="utf-8") as output:
                    json.dump(report, output, indent=2, sort_keys=True)
                    output.write("\n")
        finally:
            if vehicle is not None:
                vehicle.destroy()
            if world is not None:
                world.apply_settings(original_settings)


if __name__ == "__main__":
    sys.exit(main())
