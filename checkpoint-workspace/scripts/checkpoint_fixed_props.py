"""Opt-in, fixed map-prop mode for dedicated checkpoint experiments.

Freezing props changes simulation behavior. Prepare immediately after map load,
before the scenario advances, and prevent external changes throughout the run.
This is not a snapshot of dynamic prop physics or a general world checkpoint.
"""
import json

import carla


def observe(world):
    snapshot = world.get_snapshot()
    props = [actor for actor in world.get_actors() if actor.type_id.startswith('static.prop.')]
    result = []
    for actor in props:
        state = snapshot.find(actor.id)
        if state is None:
            raise RuntimeError('prop missing from world snapshot: {}'.format(actor.id))
        transform = state.get_transform()
        velocity, angular = state.get_velocity(), state.get_angular_velocity()
        result.append({'id': actor.id, 'type': actor.type_id, 'attributes': dict(actor.attributes),
                       'transform': {'x': transform.location.x, 'y': transform.location.y, 'z': transform.location.z,
                                     'pitch': transform.rotation.pitch, 'yaw': transform.rotation.yaw,
                                     'roll': transform.rotation.roll},
                       'velocity': [velocity.x, velocity.y, velocity.z],
                       'angular_velocity': [angular.x, angular.y, angular.z]})
    if world.get_snapshot().frame != snapshot.frame:
        raise RuntimeError('world advanced while observing props')
    return snapshot.frame, result


def canonical(records):
    """Compare visible prop attributes and poses independently of runtime IDs."""
    values = [{key: value for key, value in record.items() if key != 'id'} for record in records]
    return sorted(values, key=lambda value: json.dumps(value, sort_keys=True, separators=(',', ':')))


def prepare(client, world):
    if not world.get_settings().synchronous_mode:
        raise ValueError('fixed props require a synchronous world')
    active = [a.type_id for a in world.get_actors() if a.type_id.startswith(('vehicle.', 'walker.', 'sensor.'))]
    if active:
        raise ValueError('prepare fixed props before spawning scenario vehicles, walkers or sensors')
    frame, records = observe(world)
    replies = client.apply_batch_sync([carla.command.SetSimulatePhysics(record['id'], False) for record in records], False)
    if len(replies) != len(records) or any(reply.error for reply in replies):
        raise RuntimeError('fixed-prop command failed: {}'.format([r.error for r in replies if r.error]))
    if world.get_snapshot().frame != frame:
        raise RuntimeError('world advanced while fixing props')
    # The streamed snapshot can retain pre-disable velocities. Publish one
    # acknowledged frame before recording the fixed scene for later checks.
    if world.tick() != frame + 1:
        raise RuntimeError('unexpected world tick during fixed-prop preparation')
    prepared_frame, prepared = observe(world)
    if {r['id'] for r in prepared} != {r['id'] for r in records}:
        raise RuntimeError('prop set changed during preparation')
    if any(any(value != 0.0 for value in r['velocity'] + r['angular_velocity']) for r in prepared):
        raise RuntimeError('some props still report velocity after preparation')
    return {'mode': 'fixed_map_props_v1', 'map': world.get_map().name.rsplit('/', 1)[-1],
            'prepared_frame': prepared_frame, 'records': prepared,
            'limitation': 'physics-disable commands acknowledged; no public readback of simulate-physics flag'}


def validate_current(world, guard, extra_prop_ids=()):
    if guard.get('mode') != 'fixed_map_props_v1' or guard.get('map') != world.get_map().name.rsplit('/', 1)[-1]:
        raise ValueError('invalid fixed-prop guard')
    _, current = observe(world)
    expected = {record['id']: record for record in guard['records']}
    extras = set(extra_prop_ids)
    if extras & set(expected) or not extras <= {r['id'] for r in current}:
        raise ValueError('invalid extra prop actor IDs')
    current = [r for r in current if r['id'] not in extras]
    if len(expected) != len(guard['records']) or {r['id'] for r in current} != set(expected):
        raise ValueError('fixed prop actor set changed')
    if any(record != expected[record['id']] for record in current):
        raise ValueError('fixed prop pose, velocity, or attributes changed')
    return current
