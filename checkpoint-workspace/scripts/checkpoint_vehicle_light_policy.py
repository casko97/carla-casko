"""Optional deterministic vehicle lights derived from manual controls.

Use the policy throughout the original scenario and replay under one tick owner.
Other light writers, including Traffic Manager, must be disabled.
"""
import carla

POLICY = 'fixed-plus-control-v1'
BRAKE = 0x8
REVERSE = 0x40
ALL_LIGHTS = 0x7ff


def validate(policy):
    if policy is None:
        return None
    if (not isinstance(policy, dict) or set(policy) != {'mode', 'fixed_mask'} or
            policy['mode'] != POLICY or type(policy['fixed_mask']) is not int or
            policy['fixed_mask'] < 0 or policy['fixed_mask'] & ~ALL_LIGHTS or
            policy['fixed_mask'] & (BRAKE | REVERSE)):
        raise ValueError('invalid derived vehicle-light policy')
    return policy


def state(policy, control):
    validate(policy)
    if policy is None:
        raise ValueError('light policy is disabled')
    result = policy['fixed_mask']
    if control.get('brake', 0.0) > 0.5:
        result |= BRAKE
    if control.get('reverse', False):
        result |= REVERSE
    return result


def apply(client, world, actors, controls, policy):
    validate(policy)
    if policy is None:
        return
    if set(actors) != set(controls):
        raise ValueError('vehicle controls and light-policy actor sets differ')
    frame = world.get_snapshot().frame
    commands = [carla.command.SetVehicleLightState(actors[label].id, state(policy, controls[label]))
                for label in sorted(actors)]
    replies = client.apply_batch_sync(commands, False)
    if len(replies) != len(commands) or any(reply.error for reply in replies):
        raise RuntimeError('vehicle-light batch failed; some commands may have been applied')
    if world.get_snapshot().frame != frame:
        raise RuntimeError('world advanced while applying vehicle lights')


def check_current(world, actors, policy):
    validate(policy)
    if policy is None:
        return
    frame = world.get_snapshot().frame
    for label, actor in sorted(actors.items()):
        control = actor.get_control()
        expected = state(policy, {'brake': control.brake, 'reverse': control.reverse})
        if int(actor.get_light_state()) != expected:
            raise ValueError('vehicle {} does not match the selected light policy'.format(label))
    if world.get_snapshot().frame != frame:
        raise RuntimeError('world advanced while checking vehicle lights')
