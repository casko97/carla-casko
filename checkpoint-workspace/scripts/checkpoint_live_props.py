"""Experimental live dynamic-prop capture for the manual-vehicle harness."""
import base64
import hashlib
import math
import subprocess
import tempfile
from pathlib import Path

import carla
import vehicle_physics_checkpoint as single


def rpc(helper, port, operation, actor_id, path=None, build_id=None):
    args = [helper, str(port), operation, str(actor_id)]
    if path is not None:
        args += [str(path), build_id]
    result = subprocess.run(args, capture_output=True, text=True, timeout=45)
    if result.returncode:
        raise RuntimeError('prop {} failed for {}: {}'.format(operation, actor_id, result.stderr.strip() or result.stdout.strip()))


def validate_selection(mapping, transforms, helper):
    if not helper or not isinstance(mapping, dict) or not mapping or len(mapping) > 64:
        raise ValueError('dynamic props require a helper and a nonempty label-to-ID map')
    if (any(not isinstance(k, str) or not k or type(v) is not int or v <= 0 for k, v in mapping.items()) or
            len(set(mapping.values())) != len(mapping) or not isinstance(transforms, dict) or set(transforms) != set(mapping)):
        raise ValueError('dynamic prop IDs and original spawn transforms must have matching unique labels')
    for label, t in transforms.items():
        if (not isinstance(t, dict) or set(t) != {'x', 'y', 'z', 'pitch', 'yaw', 'roll'} or
                any(type(v) not in (int, float) or not math.isfinite(v) for v in t.values())):
            raise ValueError('invalid prop spawn transform for {}'.format(label))


def capture(world, mapping, transforms, helper, port, build_id, frame):
    validate_selection(mapping, transforms, helper)
    actors = {a.id: a for a in world.get_actors()}
    slots = {}
    with tempfile.TemporaryDirectory(prefix='carla-live-props-') as tmp:
        for i, (label, actor_id) in enumerate(sorted(mapping.items())):
            actor = actors.get(actor_id)
            if actor is None or not actor.type_id.startswith('static.prop.') or world.get_snapshot().find(actor_id) is None:
                raise ValueError('dynamic prop not present in snapshot: {}'.format(label))
            t = transforms[label]
            blueprint = world.get_blueprint_library().find(actor.type_id)
            attributes = {k: v for k, v in actor.attributes.items()
                          if blueprint.has_attribute(k) and blueprint.get_attribute(k).is_modifiable}
            path = Path(tmp) / str(i)
            rpc(helper, port, 'save', actor_id)
            rpc(helper, port, 'export', actor_id, path, build_id)
            data = path.read_bytes()
            if not 0 < len(data) <= 1024 * 1024:
                raise ValueError('invalid prop checkpoint size')
            slots[label] = {'source_actor_id': actor_id, 'blueprint': actor.type_id,
                            'attributes': dict(actor.attributes), 'spawn_attributes': attributes,
                            'spawn_transform': t, 'data_base64': base64.b64encode(data).decode('ascii'),
                            'sha256': hashlib.sha256(data).hexdigest(), 'boundary': single.sample(world, actor)}
            if world.get_snapshot().frame != frame:
                raise RuntimeError('world advanced during prop capture')
    return slots


def decode(slots):
    result = {}
    if not isinstance(slots, dict) or len(slots) > 64:
        raise ValueError('invalid prop slots')
    for label, slot in slots.items():
        data = base64.b64decode(slot['data_base64'], validate=True)
        if not 0 < len(data) <= 1024 * 1024 or hashlib.sha256(data).hexdigest() != slot['sha256']:
            raise ValueError('invalid prop checkpoint data for {}'.format(label))
        result[label] = data
    return result


def spawn(client, world, slots):
    actors = {}
    try:
        for label, slot in sorted(slots.items()):
            bp = world.get_blueprint_library().find(slot['blueprint'])
            if not slot['blueprint'].startswith('static.prop.'):
                raise ValueError('invalid prop blueprint')
            for key, value in slot['spawn_attributes'].items():
                if not bp.has_attribute(key) or not bp.get_attribute(key).is_modifiable:
                    raise ValueError('invalid prop attribute')
                bp.set_attribute(key, value)
            t = slot['spawn_transform']
            actor = world.try_spawn_actor(bp, carla.Transform(carla.Location(x=t['x'], y=t['y'], z=t['z']),
                carla.Rotation(pitch=t['pitch'], yaw=t['yaw'], roll=t['roll'])))
            if actor is None:
                raise RuntimeError('prop spawn failed for {}'.format(label))
            actors[label] = actor
            if dict(actor.attributes) != slot['attributes']:
                raise RuntimeError('prop attributes differ for {}'.format(label))
        replies = client.apply_batch_sync([carla.command.SetSimulatePhysics(a.id, True) for a in actors.values()], False)
        if len(replies) != len(actors) or any(r.error for r in replies):
            raise RuntimeError('could not enable prop physics')
        if actors:
            world.tick()
        return actors
    except Exception:
        for actor in actors.values():
            actor.destroy()
        raise


def restore(world, actors, decoded, helper, port, build_id, frame):
    with tempfile.TemporaryDirectory(prefix='carla-live-props-') as tmp:
        for i, (label, actor) in enumerate(sorted(actors.items())):
            path = Path(tmp) / str(i)
            path.write_bytes(decoded[label])
            rpc(helper, port, 'import', actor.id, path, build_id)
            rpc(helper, port, 'restore', actor.id)
            if world.get_snapshot().frame != frame:
                raise RuntimeError('world advanced during prop restore')
