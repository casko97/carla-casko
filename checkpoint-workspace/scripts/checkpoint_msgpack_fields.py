"""Compare encoded fields in the experimental v2 vehicle checkpoint.

This reads only MessagePack framing. Values are compared as encoded bytes,
including nested values; it does not claim numerical equivalence across builds.
"""
import hashlib
import struct

FIELDS = ('version', 'physx_major', 'physx_minor', 'blueprint', 'map', 'build_id',
          'capture_frame', 'physx_scene_flags', 'fixed_delta_seconds', 'max_substep_delta_time',
          'max_substeps', 'substepping', 'physics_configuration', 'position_cm',
          'rotation_quaternion', 'scale', 'linear_velocity_cm_s', 'angular_velocity_rad_s',
          'awake', 'wake_counter', 'input_control', 'last_control', 'input_priority',
          'light_state', 'rollover_tracker', 'failure_state', 'rollover_timer_exists',
          'rollover_timer_paused', 'rollover_timer_remaining', 'rollover_timer_rate',
          'drive', 'wheels', 'movement_floats', 'movement_bools', 'body_position_physx',
          'body_rotation_quaternion', 'body_linear_velocity_physx', 'body_angular_velocity_physx',
          'observer_velocity_m_s')
FIXED_SLEEP_SUFFIX = '|fixed-sleep-threshold-zero-v1'


def fields(data):
    if len(data) > 1024 * 1024:
        raise ValueError('checkpoint too large')

    def read(pos, size):
        if pos + size > len(data):
            raise ValueError('truncated MessagePack')
        return data[pos:pos + size]

    def length(pos, width):
        return int.from_bytes(read(pos, width), 'big'), pos + width

    def skip(pos, depth=0):
        if depth > 64:
            raise ValueError('MessagePack nesting too deep')
        code = read(pos, 1)[0]
        pos += 1
        if code <= 0x7f or code >= 0xe0 or code in (0xc0, 0xc2, 0xc3):
            return pos
        if 0xa0 <= code <= 0xbf:
            size = code & 31
            read(pos, size)
            return pos + size
        if 0x90 <= code <= 0x9f:
            count = code & 15
        elif 0x80 <= code <= 0x8f:
            count = (code & 15) * 2
        elif code in (0xdc, 0xdd, 0xde, 0xdf):
            count, pos = length(pos, 2 if code in (0xdc, 0xde) else 4)
            if code in (0xde, 0xdf):
                count *= 2
        elif code in (0xc4, 0xc5, 0xc6, 0xd9, 0xda, 0xdb):
            width = {0xc4: 1, 0xc5: 2, 0xc6: 4, 0xd9: 1, 0xda: 2, 0xdb: 4}[code]
            size, pos = length(pos, width)
            read(pos, size)
            return pos + size
        else:
            sizes = {0xca: 4, 0xcb: 8, 0xcc: 1, 0xcd: 2, 0xce: 4, 0xcf: 8,
                     0xd0: 1, 0xd1: 2, 0xd2: 4, 0xd3: 8}
            if code not in sizes:
                raise ValueError('unsupported MessagePack marker: {}'.format(hex(code)))
            size = sizes[code]
            read(pos, size)
            return pos + size
        if count > len(data) - pos:
            raise ValueError('invalid MessagePack container size')
        for _ in range(count):
            pos = skip(pos, depth + 1)
        return pos

    marker = read(0, 1)[0]
    if 0x90 <= marker <= 0x9f:
        count, pos = marker & 15, 1
    elif marker in (0xdc, 0xdd):
        count, pos = length(1, 2 if marker == 0xdc else 4)
    else:
        raise ValueError('checkpoint is not an array')
    if count != len(FIELDS):
        raise ValueError('unexpected checkpoint field count')
    result = {}
    for name in FIELDS:
        end = skip(pos)
        result[name] = data[pos:end]
        pos = end
    if pos != len(data) or result['version'] != b'\x02':
        raise ValueError('unexpected checkpoint version or trailing data')
    return result


def compare(expected, actual):
    left, right = fields(expected), fields(actual)
    differences = {}
    for name in FIELDS:
        if name != 'capture_frame' and left[name] != right[name]:
            differences[name] = {'expected_sha256': hashlib.sha256(left[name]).hexdigest(),
                                 'actual_sha256': hashlib.sha256(right[name]).hexdigest()}
            if name == 'wake_counter':
                def float_value(encoded):
                    if len(encoded) == 5 and encoded[0] == 0xca:
                        return struct.unpack('>f', encoded[1:])[0]
                    if len(encoded) == 9 and encoded[0] == 0xcb:
                        return struct.unpack('>d', encoded[1:])[0]
                    return None
                differences[name]['expected_value'] = float_value(left[name])
                differences[name]['actual_value'] = float_value(right[name])
            if name in ('position_cm', 'rotation_quaternion', 'scale', 'linear_velocity_cm_s',
                        'angular_velocity_rad_s', 'body_position_physx', 'body_rotation_quaternion',
                        'body_linear_velocity_physx', 'body_angular_velocity_physx', 'observer_velocity_m_s'):
                differences[name]['expected_msgpack_hex'] = left[name].hex()
                differences[name]['actual_msgpack_hex'] = right[name].hex()
                def float_array(encoded):
                    count, pos, values = encoded[0] & 15, 1, []
                    if encoded[0] < 0x90 or encoded[0] > 0x9f:
                        return None
                    for _ in range(count):
                        marker = encoded[pos]
                        if marker not in (0xca, 0xcb):
                            return None
                        size = 4 if marker == 0xca else 8
                        values.append(struct.unpack('>f' if size == 4 else '>d', encoded[pos + 1:pos + 1 + size])[0])
                        pos += size + 1
                    return values if pos == len(encoded) else None
                differences[name]['expected_values'] = float_array(left[name])
                differences[name]['actual_values'] = float_array(right[name])
    return differences


def checkpoint_mode(data, build_id):
    """Read the server's mode marker from the encoded checkpoint build ID."""
    encoded = fields(data)['build_id']
    marker = encoded[0]
    if 0xa0 <= marker <= 0xbf:
        length, start = marker & 31, 1
    elif marker in (0xd9, 0xda, 0xdb):
        width = {0xd9: 1, 0xda: 2, 0xdb: 4}[marker]
        length, start = int.from_bytes(encoded[1:1 + width], 'big'), 1 + width
    else:
        raise ValueError('checkpoint build ID is not a string')
    if len(encoded) != start + length:
        raise ValueError('invalid checkpoint build ID length')
    actual = encoded[start:].decode('utf-8')
    if actual == build_id:
        return 'default'
    if actual == build_id + FIXED_SLEEP_SUFFIX:
        return 'fixed_sleep_threshold_zero'
    raise ValueError('checkpoint build ID or mode is unexpected')
