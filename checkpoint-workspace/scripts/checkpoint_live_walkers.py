"""Experimental manually controlled walkers for live checkpoint replay.

This restores root transform, character velocity and observer velocity only.
Character movement internals, animation, AI and contact state are not captured.
"""
import base64
import hashlib
import math
import subprocess
import tempfile
from pathlib import Path

import carla
import checkpoint_scenario_bundle  # Registers the CARLA examples directory.
import vehicle_physics_checkpoint as single


def validate_selection(mapping, transforms, helper):
    if (not helper or not isinstance(mapping, dict) or not 1 <= len(mapping) <= 64 or
            any(not isinstance(k, str) or not k or type(v) is not int or v <= 0 for k, v in mapping.items()) or
            len(set(mapping.values())) != len(mapping) or not isinstance(transforms, dict) or set(transforms) != set(mapping)):
        raise ValueError('walkers require unique IDs, a helper and matching original spawn transforms')
    for label, t in transforms.items():
        if (not isinstance(t, dict) or set(t) != {'x', 'y', 'z', 'pitch', 'yaw', 'roll'} or
                any(type(v) not in (int, float) or not math.isfinite(v) for v in t.values())):
            raise ValueError('invalid walker spawn transform for {}'.format(label))


def rpc(helper, port, operation, actor_id, path):
    result = subprocess.run([helper, str(port), operation, str(actor_id), str(path)],
                            capture_output=True, text=True, timeout=45)
    if result.returncode:
        raise RuntimeError('walker {} failed for {}: {}'.format(operation, actor_id, result.stderr.strip() or result.stdout.strip()))


def capture(world, mapping, transforms, helper, port, frame):
    validate_selection(mapping, transforms, helper)
    by_id = {actor.id: actor for actor in world.get_actors()}
    slots = {}
    with tempfile.TemporaryDirectory(prefix='carla-live-walkers-') as tmp:
        for index, (label, actor_id) in enumerate(sorted(mapping.items())):
            actor = by_id.get(actor_id)
            if actor is None or not actor.type_id.startswith('walker.pedestrian.') or world.get_snapshot().find(actor_id) is None:
                raise ValueError('walker missing from snapshot: {}'.format(label))
            bp = world.get_blueprint_library().find(actor.type_id)
            attrs = {k: v for k, v in actor.attributes.items() if bp.has_attribute(k) and bp.get_attribute(k).is_modifiable}
            path = Path(tmp) / str(index)
            rpc(helper, port, 'capture', actor_id, path)
            data = path.read_bytes()
            if not 0 < len(data) <= 1024:
                raise ValueError('invalid walker velocity record')
            slots[label] = {'source_actor_id': actor_id, 'blueprint': actor.type_id,
                            'attributes': dict(actor.attributes), 'spawn_attributes': attrs,
                            'spawn_transform': transforms[label], 'boundary': single.sample(world, actor),
                            'data_base64': base64.b64encode(data).decode('ascii'),
                            'sha256': hashlib.sha256(data).hexdigest()}
            if world.get_snapshot().frame != frame:
                raise RuntimeError('world advanced during walker capture')
    return slots


def decode(slots):
    if not isinstance(slots, dict) or len(slots) > 64:
        raise ValueError('invalid walker slots')
    result = {}
    for label, slot in slots.items():
        data = base64.b64decode(slot['data_base64'], validate=True)
        if not 0 < len(data) <= 1024 or hashlib.sha256(data).hexdigest() != slot['sha256']:
            raise ValueError('invalid walker velocity record for {}'.format(label))
        result[label] = data
    return result


def spawn(world, slots):
    actors = {}
    try:
        for label, slot in sorted(slots.items()):
            if not slot['blueprint'].startswith('walker.pedestrian.'):
                raise ValueError('invalid walker blueprint')
            bp = world.get_blueprint_library().find(slot['blueprint'])
            for key, value in slot['spawn_attributes'].items():
                if not bp.has_attribute(key) or not bp.get_attribute(key).is_modifiable:
                    raise ValueError('invalid walker attribute')
                bp.set_attribute(key, value)
            t = slot['spawn_transform']
            actor = world.try_spawn_actor(bp, carla.Transform(carla.Location(x=t['x'], y=t['y'], z=t['z']),
                carla.Rotation(pitch=t['pitch'], yaw=t['yaw'], roll=t['roll'])))
            if actor is None:
                raise RuntimeError('walker spawn failed for {}'.format(label))
            actors[label] = actor
            if dict(actor.attributes) != slot['attributes']:
                raise RuntimeError('walker attributes differ for {}'.format(label))
        return actors
    except Exception:
        for actor in actors.values():
            actor.destroy()
        raise


def apply_controls(client, world, actors, controls):
    if set(controls) != set(actors):
        raise ValueError('walker controls must match saved walkers')
    frame = world.get_snapshot().frame
    commands = []
    for label, actor in sorted(actors.items()):
        control = controls[label]
        if 'transform' in control:
            t = control['transform']
            transform = carla.Transform(carla.Location(x=t['x'], y=t['y'], z=t['z']),
                                        carla.Rotation(pitch=t['pitch'], yaw=t['yaw'], roll=t['roll']))
            commands.append(carla.command.ApplyWalkerState(actor.id, transform, control['speed']))
        else:
            commands.append(carla.command.ApplyWalkerControl(actor.id, carla.WalkerControl(
                direction=carla.Vector3D(*control['direction']), speed=control['speed'], jump=control['jump'])))
    replies = client.apply_batch_sync(commands, False)
    if len(replies) != len(commands) or any(reply.error for reply in replies):
        raise RuntimeError('walker control batch failed')
    if world.get_snapshot().frame != frame:
        raise RuntimeError('world advanced during walker commands')


def validate_actions(actions, labels, count):
    if not isinstance(actions, list) or len(actions) != count:
        raise ValueError('walker controls must have one record per action frame')
    for controls in actions:
        if not isinstance(controls, dict) or set(controls) != set(labels):
            raise ValueError('walker control labels differ from saved walkers')
        for control in controls.values():
            if not isinstance(control, dict) or set(control) not in ({'direction', 'speed', 'jump'}, {'transform', 'speed'}):
                raise ValueError('invalid walker control')
            if 'transform' in control:
                validate_selection({'walker': 1}, {'walker': control['transform']}, 'helper')
                if type(control['speed']) not in (int, float) or not math.isfinite(control['speed']) or not 0 <= control['speed'] <= 20:
                    raise ValueError('invalid walker state speed')
                continue
            vector = control['direction']
            if (not isinstance(vector, list) or len(vector) != 3 or
                    any(type(v) not in (int, float) or not math.isfinite(v) or abs(v) > 1 for v in vector) or
                    type(control['speed']) not in (int, float) or not math.isfinite(control['speed']) or
                    not 0 <= control['speed'] <= 20 or type(control['jump']) is not bool):
                raise ValueError('invalid walker direction, speed or jump')
    return actions


def prepare_actions(actions, slots, count, reserved_labels=()):
    """Validate a controls list or a lifecycle journal before changing the world."""
    if isinstance(actions, list):
        validate_actions(actions, slots, count)
        return [{'events': [], 'controls': controls} for controls in actions], dict(slots)
    if not isinstance(actions, dict) or set(actions) != {'actors', 'frames'}:
        raise ValueError('walker actions must be controls or a lifecycle journal')
    extra, frames = actions['actors'], actions['frames']
    if not isinstance(extra, dict) or len(extra) + len(slots) > 64 or not isinstance(frames, list) or len(frames) != count:
        raise ValueError('invalid walker lifecycle definitions or frame count')
    if set(extra) & (set(slots) | set(reserved_labels)):
        raise ValueError('walker lifecycle labels must be unique across actors')
    specs = dict(slots)
    for label, spec in extra.items():
        if not isinstance(label, str) or not label or not isinstance(spec, dict) or set(spec) != {'blueprint', 'attributes', 'spawn_attributes', 'spawn_transform'}:
            raise ValueError('invalid walker lifecycle definition')
        if not isinstance(spec['blueprint'], str) or not spec['blueprint'].startswith('walker.pedestrian.'):
            raise ValueError('invalid walker blueprint')
        if any(not isinstance(spec[k], dict) or any(not isinstance(a, str) or not isinstance(b, str) for a, b in spec[k].items()) for k in ('attributes', 'spawn_attributes')):
            raise ValueError('invalid walker attributes')
        validate_selection({label: 1}, {label: spec['spawn_transform']}, 'helper')
        specs[label] = spec
    alive = set(slots)
    for index, frame in enumerate(frames):
        if not isinstance(frame, dict) or set(frame) != {'events', 'controls'} or not isinstance(frame['events'], list):
            raise ValueError('invalid walker lifecycle frame {}'.format(index))
        for event in frame['events']:
            if not isinstance(event, dict) or set(event) != {'action', 'label'} or event['label'] not in specs:
                raise ValueError('invalid walker lifecycle event at frame {}'.format(index))
            label, action = event['label'], event['action']
            if action == 'spawn' and label not in alive:
                alive.add(label)
            elif action == 'destroy' and label in alive:
                alive.remove(label)
            else:
                raise ValueError('invalid walker lifecycle transition at frame {}'.format(index))
        validate_actions([frame['controls']], alive, 1)
    return frames, specs


def execute_frame(client, world, actors, frame, specs):
    for event in frame['events']:
        label = event['label']
        if event['action'] == 'spawn':
            actors.update(spawn(world, {label: specs[label]}))
        else:
            if not actors[label].destroy():
                raise RuntimeError('walker destroy failed for {}'.format(label))
            del actors[label]
    apply_controls(client, world, actors, frame['controls'])


def validate_reference(reference, labels, count):
    if not isinstance(reference, list) or len(reference) != count:
        raise ValueError('walker reference must have one record per frame')
    for frame in reference:
        if not isinstance(frame, dict) or set(frame) != set(labels):
            raise ValueError('walker reference labels differ from saved walkers')
        for sample in frame.values():
            if not isinstance(sample, dict):
                raise ValueError('invalid walker reference sample')
            for field in ('location', 'velocity'):
                values = sample.get(field)
                if (not isinstance(values, list) or len(values) != 3 or any(
                        type(value) not in (int, float) or not math.isfinite(value) for value in values)):
                    raise ValueError('invalid walker reference {}'.format(field))
    return reference


def restore(client, world, actors, slots, decoded, helper, port, frame):
    with tempfile.TemporaryDirectory(prefix='carla-live-walkers-') as tmp:
        for index, (label, actor) in enumerate(sorted(actors.items())):
            boundary = slots[label]['boundary']
            x, y, z = boundary['location']
            pitch, yaw, roll = boundary['rotation']
            transform = carla.Transform(carla.Location(x=x, y=y, z=z), carla.Rotation(pitch=pitch, yaw=yaw, roll=roll))
            replies = client.apply_batch_sync([carla.command.ApplyTransform(actor.id, transform)], False)
            if len(replies) != 1 or replies[0].error:
                raise RuntimeError('walker transform restore failed for {}'.format(label))
            path = Path(tmp) / str(index)
            path.write_bytes(decoded[label])
            rpc(helper, port, 'restore', actor.id, path)
            if world.get_snapshot().frame != frame:
                raise RuntimeError('world advanced during walker restore')
