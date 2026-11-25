#!/usr/bin/env python3
"""Capture selected live manual vehicles and replay controls in a fresh episode.

Capture does not tick or reload the source world, but replaces each selected
vehicle's experimental checkpoint slot. Use one tick owner and exclude all
other clients and asynchronous mutations during capture. This is not a world
checkpoint; see checkpoint_live_vehicles.md for supported scope and limits.
"""
import argparse
import base64
import hashlib
import json
import math
from pathlib import Path

import carla
import checkpoint_msgpack_fields as fields
import checkpoint_fixed_props as fixed_props
import checkpoint_scenario_bundle as common
import checkpoint_live_props as live_props
import checkpoint_live_walkers as live_walkers
import checkpoint_live_traffic as live_traffic
import checkpoint_vehicle_light_policy as light_policy
import checkpoint_journal_bundle as journal
import multi_vehicle_physics_checkpoint as multi
import vehicle_physics_checkpoint as single

SCHEMA = 'carla-live-manual-vehicles-v1'


def check_settings(world):
    settings = world.get_settings()
    if (not settings.synchronous_mode or settings.fixed_delta_seconds is None or
            not math.isclose(settings.fixed_delta_seconds, 0.05, abs_tol=1e-7) or
            not settings.substepping or not math.isclose(settings.max_substep_delta_time, 0.01, abs_tol=1e-7) or
            settings.max_substeps != 10 or not settings.deterministic_ragdolls):
        raise ValueError('source world must use synchronous 0.05 s steps and the harness physics settings')
    common.map_artifact_paths(world.get_map().name.rsplit('/', 1)[-1])
    return {'synchronous_mode': settings.synchronous_mode, 'fixed_delta_seconds': settings.fixed_delta_seconds,
            'substepping': settings.substepping, 'max_substep_delta_time': settings.max_substep_delta_time,
            'max_substeps': settings.max_substeps, 'deterministic_ragdolls': settings.deterministic_ragdolls}


def capture(world, mapping, path, allow_uncaptured_props=False, bootstrap_transforms=None, fixed_props_guard=None,
            prop_mapping=None, prop_bootstrap_transforms=None, prop_rpc=None, port=23000, traffic_rpc=None,
            vehicle_light_policy=None, walker_mapping=None, walker_bootstrap_transforms=None, walker_rpc=None):
    if Path(path).exists():
        raise FileExistsError(path)
    if (not isinstance(mapping, dict) or not mapping or len(mapping) > 64 or
            any(not isinstance(k, str) or not k or type(v) is not int or v <= 0 for k, v in mapping.items()) or
            len(set(mapping.values())) != len(mapping)):
        raise ValueError('mapping must contain unique nonempty labels and positive vehicle IDs')
    if bootstrap_transforms is not None and (not isinstance(bootstrap_transforms, dict) or
                                               set(bootstrap_transforms) != set(mapping)):
        raise ValueError('bootstrap transforms must map exactly the selected labels')
    if fixed_props_guard is not None and allow_uncaptured_props:
        raise ValueError('fixed props and uncaptured props are mutually exclusive')
    if prop_mapping is None:
        prop_mapping = {}
    if not isinstance(prop_mapping, dict):
        raise ValueError('prop mapping must be a JSON object')
    if prop_mapping:
        live_props.validate_selection(prop_mapping, prop_bootstrap_transforms, prop_rpc)
    if set(prop_mapping) & set(mapping):
        raise ValueError('vehicle and prop labels must be distinct')
    if not prop_mapping and (prop_bootstrap_transforms is not None or prop_rpc is not None):
        raise ValueError('prop helper and transforms require selected props')
    if walker_mapping is None:
        walker_mapping = {}
    if not isinstance(walker_mapping, dict):
        raise ValueError('walker mapping must be a JSON object')
    if walker_mapping:
        live_walkers.validate_selection(walker_mapping, walker_bootstrap_transforms, walker_rpc)
    elif walker_bootstrap_transforms is not None or walker_rpc is not None:
        raise ValueError('walker helper and transforms require selected walkers')
    if set(walker_mapping) & (set(mapping) | set(prop_mapping)):
        raise ValueError('walker, vehicle and prop labels must be distinct')
    light_policy.validate(vehicle_light_policy)
    settings = check_settings(world)
    map_name = world.get_map().name.rsplit('/', 1)[-1]
    identity = common.fingerprint(map_name)
    snapshot = world.get_snapshot()
    frame = snapshot.frame
    all_actors = list(world.get_actors())
    by_id = {actor.id: actor for actor in all_actors}
    vehicle_ids = {actor.id for actor in all_actors if actor.type_id.startswith('vehicle.')}
    prop_count = sum(actor.type_id.startswith('static.prop.') for actor in all_actors)
    walker_ids = {actor.id for actor in all_actors if actor.type_id.startswith('walker.pedestrian.')}
    if walker_ids != set(walker_mapping.values()):
        raise ValueError('selected walkers must cover all visible pedestrians')
    if fixed_props_guard is not None:
        fixed_props.validate_current(world, fixed_props_guard, prop_mapping.values())
    unsupported = [actor.type_id for actor in all_actors if actor.type_id.startswith(
        ('controller.ai.', 'sensor.')) or
        (actor.type_id.startswith('walker.') and not actor.type_id.startswith('walker.pedestrian.')) or
        (actor.type_id.startswith('static.prop.') and actor.id not in prop_mapping.values() and
         not allow_uncaptured_props and fixed_props_guard is None)]
    if vehicle_ids != set(mapping.values()) or unsupported:
        raise ValueError('selected vehicles must cover all visible vehicles; unselected props, sensors, controllers and unsupported walkers are not supported')
    actors = {}
    for label, actor_id in mapping.items():
        actor = by_id.get(actor_id)
        if actor is None or snapshot.find(actor_id) is None or not actor.type_id.startswith('vehicle.'):
            raise ValueError('selected vehicle is not present at the observed frame')
        if actor.attributes.get('sticky_control', 'false').lower() != 'false':
            raise ValueError('selected vehicles must use sticky_control=false')
        actors[label] = actor
    light_policy.check_current(world, actors, vehicle_light_policy)
    specs, slots, mode = [], {}, None
    for label, actor in sorted(actors.items()):
        observed = snapshot.find(actor.id)
        transform = observed.get_transform()
        blueprint = world.get_blueprint_library().find(actor.type_id)
        attributes = {k: v for k, v in actor.attributes.items()
                      if blueprint.has_attribute(k) and blueprint.get_attribute(k).is_modifiable}
        # A fresh vehicle needs space to initialize before the saved pose is restored.
        bootstrap = (bootstrap_transforms[label] if bootstrap_transforms is not None else
                     {'x': transform.location.x, 'y': transform.location.y,
                      'z': transform.location.z + 1.0, 'pitch': transform.rotation.pitch,
                      'yaw': transform.rotation.yaw, 'roll': transform.rotation.roll})
        spec = {'id': label, 'blueprint': actor.type_id, 'attributes': attributes, 'transform': bootstrap}
        specs.append(spec)
    dummy = {label: {} for label in mapping}
    multi.validate_scenario({'map': map_name, 'actors': specs, 'frames': [dummy, dummy]}, 1)
    if world.get_snapshot().frame != frame:
        raise RuntimeError('world advanced during capture preflight')
    for label, actor in sorted(actors.items()):
        if not actor.save_physics_checkpoint():
            raise RuntimeError('save rejected for {}'.format(label))
        data = actor.export_physics_checkpoint(identity['selected_files_id'])
        actor_mode = fields.checkpoint_mode(data, identity['selected_files_id'])
        if mode is not None and mode != actor_mode:
            raise RuntimeError('vehicles reported different checkpoint modes')
        mode = actor_mode
        slots[label] = {'source_actor_id': actor.id, 'data_base64': base64.b64encode(data).decode('ascii'),
                        'sha256': hashlib.sha256(data).hexdigest(), 'attributes': dict(actor.attributes)}
        if world.get_snapshot().frame != frame:
            raise RuntimeError('world advanced during capture')
    prop_slots = (live_props.capture(world, prop_mapping, prop_bootstrap_transforms, prop_rpc, port,
                                     identity['selected_files_id'], frame) if prop_mapping else {})
    walker_slots = (live_walkers.capture(world, walker_mapping, walker_bootstrap_transforms, walker_rpc, port, frame)
                    if walker_mapping else {})
    traffic_slot = (live_traffic.capture(world, traffic_rpc, port, identity['selected_files_id'], frame)
                    if traffic_rpc else None)
    observation = common.sample_checkpoint(world, actors)
    if world.get_snapshot().frame != frame or common.fingerprint(map_name) != identity:
        raise RuntimeError('world frame or selected artifacts changed during capture')
    bundle = {'schema': SCHEMA, 'map': map_name, 'source_frame': frame, 'settings': settings,
              'identity': identity, 'checkpoint_mode': mode, 'actors': slots, 'spawn_specs': specs,
              'boundary_observation': observation,
              'uncaptured_prop_count': prop_count - len(prop_slots) if fixed_props_guard is None else 0,
              'fixed_props': fixed_props_guard, 'dynamic_props': prop_slots, 'traffic_lights': traffic_slot,
              'vehicle_light_policy': vehicle_light_policy, 'walkers': walker_slots,
              'limitations': ['single tick owner required; sequential capture is not atomic',
                              'manual controls assumed; Traffic Manager and external controllers not captured',
                              'sensors, walker animation and movement internals, uncaptured props and physics scene caches not captured',
                              'traffic-light controller state saved only if selected; overlap and actor control state omitted',
                              'source origin assumed zero; selected artifact hashes do not identify all runtime content']}
    common.atomic_create(path, bundle)
    return {'saved': str(path), 'source_frame': frame, 'actors': sorted(slots), 'checkpoint_mode': mode,
            'uncaptured_prop_count': prop_count - len(prop_slots) if fixed_props_guard is None else 0,
            'fixed_prop_count': len(fixed_props_guard['records']) if fixed_props_guard is not None else 0}


def prepare_actions(bundle, actions):
    """Validate a manual-vehicle suffix before loading or modifying the target world."""
    specs = bundle['spawn_specs']
    initial = sorted(bundle['actors'])
    if {spec['id'] for spec in specs} != set(initial):
        raise ValueError('checkpoint actor specifications differ from saved actors')
    if isinstance(actions, list):
        if len(actions) < 2:
            raise ValueError('actions must contain at least two frames')
        frames = [{'events': [], 'controls': controls} for controls in actions]
        extra = []
    elif isinstance(actions, dict) and set(actions) == {'actors', 'frames'}:
        extra, frames = actions['actors'], actions['frames']
        if not isinstance(extra, list) or not isinstance(frames, list) or not frames:
            raise ValueError('lifecycle actions require actor definitions and nonempty frames')
    else:
        raise ValueError('actions must be a control list or an actor lifecycle journal')
    prop_labels = set(bundle.get('dynamic_props', {})) | set(bundle.get('walkers', {}))
    if any(not isinstance(spec, dict) or spec.get('id') in prop_labels for spec in extra):
        raise ValueError('lifecycle vehicle labels must differ from prop labels')
    fake_prefix = {'events': [], 'controls': {label: {} for label in initial}}
    definition = {'map': bundle['map'], 'actors': specs + extra, 'initial_actors': initial,
                  'frames': [fake_prefix] + frames}
    by_label, _ = journal.validate_journal(definition, 1)
    return frames, by_label


def replay(client, checkpoint_path, actions, report_path, reference=None, prop_rpc=None, port=23000,
           prop_reference=None, traffic_reference=None, traffic_rpc=None, vehicle_light_reference=None,
           walker_rpc=None, walker_actions=None, walker_reference=None,
           walker_position_tolerance=0.01, walker_velocity_tolerance=0.05):
    if Path(report_path).exists():
        raise FileExistsError(report_path)
    bundle = json.loads(Path(checkpoint_path).read_text())
    if bundle.get('schema') != SCHEMA or bundle.get('identity') != common.fingerprint(bundle.get('map')):
        raise ValueError('unsupported checkpoint or selected artifacts differ')
    specs = bundle['spawn_specs']
    vehicle_light_policy = light_policy.validate(bundle.get('vehicle_light_policy'))
    frames, action_specs = prepare_actions(bundle, actions)
    if reference is not None and len(reference) != len(frames):
        raise ValueError('reference and action lengths differ')
    decoded = {}
    for label, slot in bundle['actors'].items():
        data = base64.b64decode(slot['data_base64'], validate=True)
        if (not 0 < len(data) <= 1024 * 1024 or hashlib.sha256(data).hexdigest() != slot['sha256'] or
                fields.checkpoint_mode(data, bundle['identity']['selected_files_id']) != bundle['checkpoint_mode']):
            raise ValueError('checkpoint data or mode mismatch for {}'.format(label))
        decoded[label] = data
    walker_slots = bundle.get('walkers', {})
    walker_decoded = live_walkers.decode(walker_slots)
    if (walker_slots or walker_actions is not None) and not walker_rpc:
        raise ValueError('walker replay requires the walker RPC helper')
    walker_frames, walker_specs = None, {}
    if walker_slots or walker_actions is not None:
        reserved = set(action_specs) | set(bundle.get('dynamic_props', {}))
        walker_frames, walker_specs = live_walkers.prepare_actions(walker_actions, walker_slots, len(frames), reserved)
    if walker_reference is not None:
        if walker_frames is None or not isinstance(walker_reference, list) or len(walker_reference) != len(frames):
            raise ValueError('walker reference requires one sample per walker action frame')
        for index, record in enumerate(walker_reference):
            live_walkers.validate_reference([record], walker_frames[index]['controls'], 1)
    for tolerance in (walker_position_tolerance, walker_velocity_tolerance):
        if type(tolerance) not in (int, float) or not math.isfinite(tolerance) or tolerance < 0:
            raise ValueError('walker tolerances must be finite and nonnegative')
    prop_slots = bundle.get('dynamic_props', {})
    prop_decoded = live_props.decode(prop_slots)
    if prop_slots and not prop_rpc:
        raise ValueError('dynamic prop replay requires prop RPC helper')
    if prop_reference is not None and len(prop_reference) != len(frames):
        raise ValueError('prop reference and action lengths differ')
    traffic_slot = bundle.get('traffic_lights')
    traffic_data = live_traffic.decode(traffic_slot) if traffic_slot is not None else None
    if traffic_data is not None and not traffic_rpc:
        raise ValueError('traffic-light replay requires the traffic RPC helper')
    if traffic_reference is not None and len(traffic_reference) != len(frames):
        raise ValueError('traffic light reference and action lengths differ')
    if vehicle_light_reference is not None and len(vehicle_light_reference) != len(frames):
        raise ValueError('vehicle-light reference and action lengths differ')
    world = single.load_world(client, bundle['map'])
    check_settings(world)
    if world.get_map().name.rsplit('/', 1)[-1] != bundle['map']:
        raise RuntimeError('target loaded a different map')
    actors, props, walkers = {}, {}, {}
    result = {'schema': SCHEMA, 'source_frame': bundle['source_frame'], 'checkpoint_mode': bundle['checkpoint_mode'],
              'scope': 'sampled vehicle state and controls; not full simulator state', 'suffix': [],
              'vehicle_light_policy': vehicle_light_policy,
              'exact': None, 'first_difference': None, 'failure': None}
    try:
        if bundle.get('fixed_props') is not None:
            target_props = fixed_props.prepare(client, world)
            if fixed_props.canonical(target_props['records']) != fixed_props.canonical(bundle['fixed_props']['records']):
                raise RuntimeError('fixed prop initial state differs from source')
            result['fixed_prop_count'] = len(target_props['records'])
        props = live_props.spawn(client, world, prop_slots)
        actors = multi.spawn_actors(world, specs)
        walkers = live_walkers.spawn(world, walker_slots)
        for label, actor in actors.items():
            if dict(actor.attributes) != bundle['actors'][label]['attributes']:
                raise RuntimeError('actor attributes differ for {}'.format(label))
        for _ in range(20):
            if walkers:
                live_walkers.apply_controls(client, world, walkers, {label: {'direction': [0, 0, 0], 'speed': 0, 'jump': False} for label in walkers})
            common.execute(world, client, actors, {'events': [], 'controls': {label: {'brake': 1.0} for label in actors}})
        world.set_weather_synchronously(carla.WeatherParameters(**bundle['boundary_observation']['weather']))
        frame = world.get_snapshot().frame
        if traffic_data is not None:
            live_traffic.restore(world, traffic_data, traffic_rpc, port, bundle['identity']['selected_files_id'], frame)
        live_props.restore(world, props, prop_decoded, prop_rpc, port, bundle['identity']['selected_files_id'], frame)
        live_walkers.restore(client, world, walkers, walker_slots, walker_decoded, walker_rpc, port, frame)
        for label, actor in sorted(actors.items()):
            if not actor.import_physics_checkpoint(decoded[label], bundle['identity']['selected_files_id'], True):
                raise RuntimeError('import rejected for {}'.format(label))
            if not actor.restore_physics_checkpoint():
                raise RuntimeError('restore rejected for {}'.format(label))
            if world.get_snapshot().frame != frame:
                raise RuntimeError('world advanced during restore')
        result['server_boundary'] = {}
        for label, actor in sorted(actors.items()):
            if not actor.save_physics_checkpoint():
                raise RuntimeError('boundary recapture rejected for {}'.format(label))
            try:
                observed = actor.export_physics_checkpoint(bundle['identity']['selected_files_id'])
                result['server_boundary'][label] = fields.compare(decoded[label], observed)
            finally:
                if not actor.import_physics_checkpoint(decoded[label], bundle['identity']['selected_files_id'], True):
                    raise RuntimeError('failed to reinstall slot for {}'.format(label))
            if world.get_snapshot().frame != frame:
                raise RuntimeError('world advanced during boundary recapture')
        traffic_lights = None
        if traffic_reference is not None:
            all_lights = multi.traffic_lights(world)
            wanted = set(traffic_reference[0])
            if not wanted or not wanted <= set(all_lights) or any(set(sample) != wanted for sample in traffic_reference):
                raise ValueError('invalid traffic light reference IDs')
            traffic_lights = {key: all_lights[key] for key in wanted}
            result['traffic_boundary'] = multi.sample_traffic_lights(traffic_lights)
        result['walker_suffix'] = []
        result['walker_max_position_error_m'] = 0.0 if walker_reference is not None else None
        result['walker_max_velocity_error_m_s'] = 0.0 if walker_reference is not None else None
        result['walker_first_outside_tolerance'] = None
        result['walker_tolerances'] = {'position_m': walker_position_tolerance, 'velocity_m_s': walker_velocity_tolerance}
        result['vehicle_light_boundary'] = multi.sample_vehicle_lights(world, actors)
        result['vehicle_light_suffix'] = []
        result['vehicle_light_first_difference'] = None
        result['traffic_suffix'] = []
        result['traffic_first_difference'] = None
        result['prop_suffix'] = []
        result['prop_first_difference'] = None
        for index, action in enumerate(frames):
            if walker_frames is not None:
                live_walkers.execute_frame(client, world, walkers, walker_frames[index], walker_specs)
            actual = journal.execute(world, client, actors, action, action_specs, vehicle_light_policy)
            result['suffix'].append(actual)
            walker_sample = {label: single.sample(world, actor) for label, actor in sorted(walkers.items())}
            result['walker_suffix'].append(walker_sample)
            if walker_reference is not None:
                expected_walkers = walker_reference[index]
                if set(expected_walkers) != set(walker_sample):
                    raise ValueError('walker reference labels differ')
                for label, observed_walker in walker_sample.items():
                    expected_walker = expected_walkers[label]
                    position_error = math.dist(observed_walker['location'], expected_walker['location'])
                    velocity_error = math.dist(observed_walker['velocity'], expected_walker['velocity'])
                    if not math.isfinite(position_error) or not math.isfinite(velocity_error):
                        raise ValueError('invalid walker reference values')
                    result['walker_max_position_error_m'] = max(result['walker_max_position_error_m'], position_error)
                    result['walker_max_velocity_error_m_s'] = max(result['walker_max_velocity_error_m_s'], velocity_error)
                    if result['walker_first_outside_tolerance'] is None and (position_error > walker_position_tolerance or velocity_error > walker_velocity_tolerance):
                        result['walker_first_outside_tolerance'] = {'suffix_frame': index + 1, 'label': label,
                            'position_error_m': position_error, 'velocity_error_m_s': velocity_error}
            if vehicle_light_reference is not None:
                vehicle_lights = multi.sample_vehicle_lights(world, actors)
                result['vehicle_light_suffix'].append(vehicle_lights)
                if result['vehicle_light_first_difference'] is None and vehicle_lights != vehicle_light_reference[index]:
                    result['vehicle_light_first_difference'] = {'suffix_frame': index + 1,
                        'expected': vehicle_light_reference[index], 'actual': vehicle_lights}
            if traffic_lights is not None:
                traffic_sample = multi.sample_traffic_lights(traffic_lights)
                result['traffic_suffix'].append(traffic_sample)
                if result['traffic_first_difference'] is None and traffic_sample != traffic_reference[index]:
                    result['traffic_first_difference'] = {'suffix_frame': index + 1,
                        'expected': traffic_reference[index], 'actual': traffic_sample}
            prop_sample = {label: single.sample(world, actor) for label, actor in sorted(props.items())}
            result['prop_suffix'].append(prop_sample)
            if prop_reference is not None and result['prop_first_difference'] is None and prop_sample != prop_reference[index]:
                result['prop_first_difference'] = {'suffix_frame': index + 1, 'expected': prop_reference[index], 'actual': prop_sample}
            if reference is not None and result['first_difference'] is None and actual != reference[index]:
                result['first_difference'] = {'suffix_frame': index + 1, 'expected': reference[index], 'actual': actual}
        if bundle.get('fixed_props') is not None:
            fixed_props.validate_current(world, target_props, (actor.id for actor in props.values()))
        if common.fingerprint(bundle['map']) != bundle['identity']:
            raise RuntimeError('selected artifacts changed during replay')
        result['exact'] = None if reference is None else result['first_difference'] is None
        result['prop_exact'] = None if prop_reference is None else result['prop_first_difference'] is None
        result['traffic_exact'] = None if traffic_reference is None else result['traffic_first_difference'] is None
        result['vehicle_light_exact'] = None if vehicle_light_reference is None else result['vehicle_light_first_difference'] is None
        result['walker_within_tolerance'] = None if walker_reference is None else result['walker_first_outside_tolerance'] is None
        return result
    except Exception as exc:
        result['failure'] = {'type': type(exc).__name__, 'message': str(exc)}
        raise
    finally:
        try:
            for actor in list(actors.values()) + list(props.values()) + list(walkers.values()):
                actor.destroy()
        finally:
            common.atomic_create(report_path, result)


def capture_reactive_traffic(client, world, traffic_manager, actors, fixed_props_guard, specs,
                             seed, prefix, initial_weather, path, tm_config=None):
    """Capture a running scenario initialized with the reactive prefix workflow."""
    import checkpoint_tm_prefix_reconstruction as reactive
    return reactive.capture_running(client, world, traffic_manager, actors, fixed_props_guard,
                                    specs, seed, prefix, initial_weather, path, tm_config)


def restore_reactive_traffic(client, checkpoint_path, tm_port):
    """Rebuild a reactive scenario; caller owns the returned actors and TM session."""
    import checkpoint_tm_prefix_reconstruction as reactive
    return reactive.reconstruct(client, checkpoint_path, tm_port)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('operation', choices=('prepare-fixed-props', 'capture', 'replay'))
    parser.add_argument('--checkpoint')
    parser.add_argument('--actors', help='JSON object mapping labels to live CARLA actor IDs; capture only')
    parser.add_argument('--bootstrap-transforms', help='optional JSON map of labels to fresh-episode spawn transforms')
    parser.add_argument('--actions', help='JSON controls list or lifecycle journal; replay only')
    parser.add_argument('--reference', help='optional JSON list of sampled suffix frames')
    parser.add_argument('--allow-uncaptured-props', action='store_true',
                        help='allow visible props whose state will not be captured')
    parser.add_argument('--fixed-props-guard', help='JSON receipt created by prepare-fixed-props')
    parser.add_argument('--props', help='capture: JSON map of dynamic prop labels to actor IDs')
    parser.add_argument('--prop-bootstrap-transforms', help='capture: JSON map of original prop spawn transforms')
    parser.add_argument('--prop-rpc', help='path to experimental native prop RPC helper')
    parser.add_argument('--prop-reference', help='optional JSON list of sampled prop suffix frames')
    parser.add_argument('--walkers', help='capture: JSON map of walker labels to actor IDs')
    parser.add_argument('--walker-bootstrap-transforms', help='capture: original walker spawn transforms')
    parser.add_argument('--walker-rpc', help='experimental walker velocity helper')
    parser.add_argument('--walker-actions', help='replay: one map of walker controls per action frame')
    parser.add_argument('--walker-reference', help='optional sampled walker states for replay comparison')
    parser.add_argument('--walker-position-tolerance', type=float, default=0.01, help='position tolerance in metres')
    parser.add_argument('--walker-velocity-tolerance', type=float, default=0.05, help='velocity tolerance in metres per second')
    parser.add_argument('--vehicle-light-reference', help='optional sampled vehicle-light states for replay diagnostics')
    parser.add_argument('--vehicle-light-policy', help='capture: JSON policy applied by the scenario from initialization')
    parser.add_argument('--traffic-rpc', help='experimental portable traffic-light RPC helper')
    parser.add_argument('--traffic-reference', help='optional sampled traffic-light suffix for diagnostics')
    parser.add_argument('--report', help='required for replay')
    parser.add_argument('--port', type=int, default=22000)
    parser.add_argument('--timeout', type=float, default=180)
    args = parser.parse_args()
    if args.operation == 'prepare-fixed-props' and not args.fixed_props_guard:
        parser.error('prepare-fixed-props requires --fixed-props-guard')
    if args.operation == 'capture' and (not args.actors or not args.checkpoint):
        parser.error('capture requires --actors and --checkpoint')
    if args.operation == 'replay' and (not args.actions or not args.report):
        parser.error('replay requires --actions and --report')
    if args.operation == 'replay' and not args.checkpoint:
        parser.error('replay requires --checkpoint')
    client = carla.Client('127.0.0.1', args.port)
    client.set_timeout(args.timeout)
    if args.operation == 'prepare-fixed-props':
        if Path(args.fixed_props_guard).exists():
            raise FileExistsError(args.fixed_props_guard)
        guard = fixed_props.prepare(client, client.get_world())
        common.atomic_create(args.fixed_props_guard, guard)
        result = {'saved': args.fixed_props_guard, 'fixed_prop_count': len(guard['records']),
                  'prepared_frame': guard['prepared_frame']}
    elif args.operation == 'capture':
        bootstrap = json.loads(Path(args.bootstrap_transforms).read_text()) if args.bootstrap_transforms else None
        guard = json.loads(Path(args.fixed_props_guard).read_text()) if args.fixed_props_guard else None
        result = capture(client.get_world(), json.loads(Path(args.actors).read_text()), args.checkpoint,
                         allow_uncaptured_props=args.allow_uncaptured_props, bootstrap_transforms=bootstrap,
                         fixed_props_guard=guard,
                         prop_mapping=json.loads(Path(args.props).read_text()) if args.props else None,
                         prop_bootstrap_transforms=json.loads(Path(args.prop_bootstrap_transforms).read_text()) if args.prop_bootstrap_transforms else None,
                         prop_rpc=args.prop_rpc, port=args.port, traffic_rpc=args.traffic_rpc,
                         vehicle_light_policy=json.loads(Path(args.vehicle_light_policy).read_text()) if args.vehicle_light_policy else None,
                         walker_mapping=json.loads(Path(args.walkers).read_text()) if args.walkers else None,
                         walker_bootstrap_transforms=json.loads(Path(args.walker_bootstrap_transforms).read_text()) if args.walker_bootstrap_transforms else None, walker_rpc=args.walker_rpc)
    else:
        actions = json.loads(Path(args.actions).read_text())
        reference = json.loads(Path(args.reference).read_text()) if args.reference else None
        settings = client.get_world().get_settings()
        try:
            result = replay(client, args.checkpoint, actions, args.report, reference, args.prop_rpc, args.port,
                            json.loads(Path(args.prop_reference).read_text()) if args.prop_reference else None,
                            json.loads(Path(args.traffic_reference).read_text()) if args.traffic_reference else None,
                            args.traffic_rpc,
                            json.loads(Path(args.vehicle_light_reference).read_text()) if args.vehicle_light_reference else None,
                            args.walker_rpc, json.loads(Path(args.walker_actions).read_text()) if args.walker_actions else None,
                            json.loads(Path(args.walker_reference).read_text()) if args.walker_reference else None,
                            args.walker_position_tolerance, args.walker_velocity_tolerance)
        finally:
            client.get_world().apply_settings(settings)
    print(json.dumps({k: v for k, v in result.items() if k not in ('suffix', 'prop_suffix', 'traffic_suffix', 'vehicle_light_suffix', 'walker_suffix')}, indent=2))
    return 1 if args.operation == 'replay' and (result['exact'] is False or result.get('prop_exact') is False or result.get('traffic_exact') is False or result.get('vehicle_light_exact') is False or result.get('walker_within_tolerance') is False) else 0


if __name__ == '__main__':
    raise SystemExit(main())
