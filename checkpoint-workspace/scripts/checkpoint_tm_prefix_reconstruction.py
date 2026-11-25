#!/usr/bin/env python3
"""Reconstruct a reactive TM checkpoint from a recorded prefix on a fresh server.

Experimental restricted reconstruction. It replays the prefix;
it does not serialize Traffic Manager internals or arbitrary scenario state.
Both operations replace the target episode and require a dedicated server.
"""
import argparse
import json
import math
from pathlib import Path

import carla
import checkpoint_fixed_props as fixed_props
import checkpoint_scenario_bundle as common
from checkpoint_tm_reactive_probe import record_step
from checkpoint_tm_same_episode import SameEpisodeTrafficCheckpoint
import multi_vehicle_physics_checkpoint as multi
import vehicle_physics_checkpoint as single

SCHEMA = 'carla-tm-prefix-reconstruction-v1'


def traffic_labels(specs):
    return [s.get('id') for s in specs if s.get('controller', 'manual') == 'traffic_manager']


def traffic_config(recipe):
    labels = traffic_labels(recipe['actors'])
    config = recipe.get('tm_config')
    if config is None and labels == ['traffic']:
        config = {'traffic': {'auto_lane_change': False, 'distance_to_leading_vehicle': 6.0}}
    if not isinstance(config, dict) or set(config) != set(labels):
        raise ValueError('Traffic Manager configuration must cover every traffic vehicle')
    for entry in config.values():
        if (not isinstance(entry, dict) or set(entry) != {'auto_lane_change', 'distance_to_leading_vehicle', 'speed_difference'} and
                set(entry) != {'auto_lane_change', 'distance_to_leading_vehicle'}):
            raise ValueError('unsupported Traffic Manager configuration')
        if type(entry['auto_lane_change']) is not bool or any(
                type(entry[k]) not in (int, float) or not math.isfinite(entry[k])
                for k in entry if k != 'auto_lane_change') or entry['distance_to_leading_vehicle'] < 0:
            raise ValueError('invalid Traffic Manager configuration')
    return config


def validate_prefix(prefix, map_name, specs):
    if not isinstance(prefix, list) or not 1 <= len(prefix) <= 10000:
        raise ValueError('prefix must contain 1 to 10000 expert actions')
    frames = [{'expert': action} for action in prefix]
    multi.validate_scenario({'map': map_name, 'actors': specs, 'frames': frames + [frames[-1]]}, len(frames))
    return prefix


def validate_recipe(recipe):
    if not isinstance(recipe, dict) or recipe.get('schema') != SCHEMA:
        raise ValueError('unsupported recipe')
    if recipe.get('map') not in ('Town01', 'Town02') or recipe.get('identity') != common.fingerprint(recipe.get('map')):
        raise ValueError('selected map or artifacts differ')
    if type(recipe.get('tm_seed')) is not int or not 0 <= recipe['tm_seed'] < 2**64:
        raise ValueError('invalid Traffic Manager seed')
    weather = recipe.get('initial_weather')
    if (not isinstance(weather, dict) or set(weather) != set(common.WEATHER_FIELDS) or
            any(type(v) not in (int, float) or not math.isfinite(v) for v in weather.values())):
        raise ValueError('invalid initial weather')
    specs = recipe.get('actors')
    if (not isinstance(specs, list) or len(specs) < 2 or any(not isinstance(a, dict) for a in specs) or
            [a.get('id') for a in specs if a.get('controller', 'manual') == 'manual'] != ['expert'] or
            not traffic_labels(specs)):
        raise ValueError('recipe requires one manual expert and at least one Traffic Manager vehicle')
    dummy = {'expert': {'brake': 1.0}}
    multi.validate_scenario({'map': recipe['map'], 'actors': specs, 'frames': [dummy, dummy]}, 1)
    traffic_config(recipe)
    if not isinstance(recipe.get('fixed_props'), list) or not isinstance(recipe.get('boundary'), dict):
        raise ValueError('missing fixed props or boundary')
    validate_prefix(recipe.get('prefix'), recipe['map'], specs)
    refs = recipe.get('source_branches')
    if refs is not None and (not isinstance(refs, dict) or set(refs) != {'hold', 'move'} or any(
            not isinstance(frames, list) or len(frames) != 120 for frames in refs.values())):
        raise ValueError('invalid source branch references')
    return recipe


def boundary(world, actors, tm_labels=None):
    frame = world.get_snapshot().frame
    result = {'vehicles': {name: single.sample(world, actor) for name, actor in sorted(actors.items())},
              'traffic_control': multi.sample_tm_controls(actors, tm_labels or ['traffic']),
              'traffic_lights': multi.sample_traffic_lights(multi.traffic_lights(world)),
              'weather': common.get_weather(world)}
    if world.get_snapshot().frame != frame:
        raise RuntimeError('world advanced during boundary sampling')
    return result


def setup(client, map_name, tm_port, seed, specs=None, weather=None, tm_config=None):
    world = single.load_world(client, map_name)
    guard = fixed_props.prepare(client, world)
    if weather is not None:
        world.set_weather_synchronously(carla.WeatherParameters(**weather))
    if specs is None:
        spawn = world.get_map().get_spawn_points()[0]
        yaw = math.radians(spawn.rotation.yaw)
        lead = {'x': spawn.location.x + 18 * math.cos(yaw), 'y': spawn.location.y + 18 * math.sin(yaw),
                'z': spawn.location.z, 'pitch': spawn.rotation.pitch, 'yaw': spawn.rotation.yaw,
                'roll': spawn.rotation.roll}
        specs = [{'id': 'expert', 'blueprint': 'vehicle.lincoln.mkz_2020', 'transform': lead},
                 {'id': 'traffic', 'blueprint': 'vehicle.lincoln.mkz_2020', 'spawn_index': 0,
                  'controller': 'traffic_manager'}]
    actors = multi.spawn_actors(world, specs)
    tm = None
    try:
        tm = multi.setup_traffic_manager(client, actors, specs, tm_port, seed)
        config = traffic_config({'actors': specs, 'tm_config': tm_config})
        for label in traffic_labels(specs):
            entry = config[label]
            tm.auto_lane_change(actors[label], entry['auto_lane_change'])
            tm.distance_to_leading_vehicle(actors[label], entry['distance_to_leading_vehicle'])
            if 'speed_difference' in entry:
                tm.vehicle_percentage_speed_difference(actors[label], entry['speed_difference'])
        return world, guard, specs, actors, tm
    except Exception:
        if tm is not None:
            for label in traffic_labels(specs):
                actors[label].set_autopilot(False, tm_port)
            tm.set_synchronous_mode(False)
        for actor in actors.values():
            actor.destroy()
        raise


def close(tm, actors, tm_port, tm_labels=None):
    if tm is not None:
        for label in (tm_labels if tm_labels is not None else ['traffic']):
            if label in actors:
                actors[label].set_autopilot(False, tm_port)
        tm.set_synchronous_mode(False)
    for actor in actors.values():
        actor.destroy()


def branch(client, world, actors, checkpoint, reference=None):
    if set(actors) != {'expert', 'traffic'}:
        raise ValueError('the demonstration branch requires exactly expert and traffic; use the live API for other scenarios')
    result = {}
    for index, (name, command) in enumerate((('hold', {'brake': 1.0}), ('move', {'throttle': 0.4}),
                                              ('move_repeat', {'throttle': 0.4}), ('hold_repeat', {'brake': 1.0}))):
        if index:
            checkpoint.restore()
        result[name] = [record_step(client, world, actors, command) for _ in range(120)]
    report = {'same_action_exact': {name: result[name] == result[name + '_repeat'] for name in ('hold', 'move')},
              'max_expert_position_difference_m': max(math.dist(a['states']['expert']['location'],
                  b['states']['expert']['location']) for a, b in zip(result['hold'], result['move'])),
              'max_traffic_control_difference': max(abs(a['traffic_control'][field] - b['traffic_control'][field])
                  for a, b in zip(result['hold'], result['move']) for field in ('throttle', 'brake', 'steer')),
              'max_traffic_position_difference_m': max(math.dist(a['states']['traffic']['location'],
                  b['states']['traffic']['location']) for a, b in zip(result['hold'], result['move'])),
              'min_center_distance_m': min(f['center_distance_m'] for frames in result.values() for f in frames)}
    if reference is not None:
        report['source_action_exact'] = {name: result[name] == reference[name] for name in ('hold', 'move')}
        report['source_max_position_error_m'] = {name: {label: max(math.dist(a['states'][label]['location'],
            b['states'][label]['location']) for a, b in zip(result[name], reference[name]))
            for label in ('expert', 'traffic')} for name in ('hold', 'move')}
    return result, report


def prepare_running_checkpoint(client, world, tm, actors, guard, specs, seed, prefix, initial_weather, tm_config=None):
    """Capture a live boundary given the caller's complete initialization history.

    The caller must have initialized the source exactly as setup() does, used
    this prefix, fixed weather and props from initialization, and prevented
    other mutation. Observable boundary matching cannot prove hidden TM state.
    """
    map_name = world.get_map().name.rsplit('/', 1)[-1]
    identity = common.fingerprint(map_name)
    skeleton = {'schema': SCHEMA, 'map': map_name, 'identity': identity, 'tm_seed': seed,
                'initial_weather': initial_weather, 'actors': specs,
                'fixed_props': fixed_props.canonical(guard['records']), 'prefix': prefix,
                'boundary': {}, 'source_branches': None}
    if tm_config is not None:
        skeleton['tm_config'] = tm_config
    validate_recipe(skeleton)
    if set(actors) != {s['id'] for s in specs} or common.get_weather(world) != initial_weather:
        raise ValueError('live actors or fixed weather differ from the declared initialization')
    for spec in specs:
        actor = actors[spec['id']]
        if actor.type_id != spec['blueprint'] or actor.attributes.get('sticky_control', 'false').lower() != 'false' or any(
                actor.attributes.get(key) != value for key, value in spec.get('attributes', {}).items()):
            raise ValueError('live vehicle does not match its declared blueprint or attributes')
    checkpoint = SameEpisodeTrafficCheckpoint(client, world, tm, actors, guard)
    frame = checkpoint.capture()
    skeleton['boundary'] = boundary(world, actors, traffic_labels(specs))
    if world.get_snapshot().frame != frame:
        raise RuntimeError('world advanced during capture')
    if common.fingerprint(map_name) != identity:
        raise RuntimeError('selected artifacts changed during capture')
    return skeleton, checkpoint


def capture_running(client, world, tm, actors, guard, specs, seed, prefix, initial_weather, path, tm_config=None):
    """Write a recipe without advancing the source episode and return its live slot."""
    if Path(path).exists():
        raise FileExistsError(path)
    recipe, checkpoint = prepare_running_checkpoint(client, world, tm, actors, guard, specs, seed, prefix, initial_weather, tm_config)
    common.atomic_create(path, recipe)
    return checkpoint


def capture(client, map_name, tm_port, seed, path, prefix=None):
    if Path(path).exists():
        raise FileExistsError(path)
    identity = common.fingerprint(map_name)
    world, guard, specs, actors, tm = setup(client, map_name, tm_port, seed)
    try:
        initial_weather = common.get_weather(world)
        prefix = validate_prefix(prefix if prefix is not None else [{'brake': 1.0} for _ in range(60)], map_name, specs)
        for command in prefix:
            record_step(client, world, actors, command)
        recipe, checkpoint = prepare_running_checkpoint(client, world, tm, actors, guard, specs, seed, prefix, initial_weather)
        traces, summary = branch(client, world, actors, checkpoint)
        if (not all(summary['same_action_exact'].values()) or summary['max_traffic_control_difference'] <= 0.01 or
                summary['max_expert_position_difference_m'] <= 1.0):
            raise RuntimeError('source branching did not provide a repeatable reactive reference')
        if common.get_weather(world) != initial_weather or common.fingerprint(map_name) != identity:
            raise RuntimeError('weather or selected artifacts changed during capture')
        recipe['source_branches'] = {'hold': traces['hold'], 'move': traces['move']}
        recipe['source_summary'] = summary
        common.atomic_create(path, recipe)
        return summary
    finally:
        close(tm, actors, tm_port)


def reconstruct(client, path, tm_port):
    recipe = validate_recipe(json.loads(Path(path).read_text()))
    world, guard, specs, actors, tm = setup(client, recipe['map'], tm_port, recipe['tm_seed'],
                                            recipe['actors'], recipe['initial_weather'], recipe.get('tm_config'))
    try:
        if fixed_props.canonical(guard['records']) != recipe['fixed_props']:
            raise RuntimeError('fixed map props differ')
        for command in recipe['prefix']:
            multi.step(client, world, actors, {'expert': command})
        checkpoint = SameEpisodeTrafficCheckpoint(client, world, tm, actors, guard)
        checkpoint.capture()
        observed = boundary(world, actors, traffic_labels(specs))
        if observed != recipe['boundary']:
            raise RuntimeError('reconstructed boundary differs; reactive replay is not accepted')
        return world, actors, tm, checkpoint, recipe
    except Exception:
        close(tm, actors, tm_port, traffic_labels(specs))
        raise


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('operation', choices=('capture', 'reconstruct'))
    p.add_argument('--recipe', required=True)
    p.add_argument('--report', required=True)
    p.add_argument('--map', default='Town01', choices=('Town01', 'Town02'))
    p.add_argument('--seed', type=int, default=17)
    p.add_argument('--prefix', help='capture: JSON list of recorded expert controls; defaults to 60 brake frames')
    p.add_argument('--port', type=int, default=36000)
    p.add_argument('--tm-port', type=int, default=8600)
    p.add_argument('--timeout', type=float, default=360)
    args = p.parse_args()
    if Path(args.report).exists():
        raise FileExistsError(args.report)
    client = carla.Client('127.0.0.1', args.port)
    client.set_timeout(args.timeout)
    original = client.get_world().get_settings()
    actors, tm = {}, None
    report = {'operation': args.operation, 'failure': None}
    try:
        if args.operation == 'capture':
            recorded_prefix = json.loads(Path(args.prefix).read_text()) if args.prefix else None
            report['summary'] = capture(client, args.map, args.tm_port, args.seed, args.recipe, recorded_prefix)
        else:
            world, actors, tm, checkpoint, recipe = reconstruct(client, args.recipe, args.tm_port)
            report['boundary_exact'] = True
            _, report['summary'] = branch(client, world, actors, checkpoint, recipe.get('source_branches'))
        print(json.dumps(report, indent=2))
        return 0 if all(report['summary']['same_action_exact'].values()) and (
            args.operation == 'capture' or report['summary'].get('source_action_exact') is None or
            all(report['summary']['source_action_exact'].values())) else 1
    except Exception as exc:
        report['failure'] = {'type': type(exc).__name__, 'message': str(exc)}
        raise
    finally:
        try:
            close(tm, actors, args.tm_port)
            client.get_world().apply_settings(original)
        finally:
            common.atomic_create(args.report, report)


if __name__ == '__main__':
    raise SystemExit(main())
