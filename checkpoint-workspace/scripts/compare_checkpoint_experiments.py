#!/usr/bin/env python3
"""Compare uninterrupted traces from two multi-vehicle checkpoint reports.

The settings and server identity must be established by the experimenter; these
reports do not by themselves prove which executable or launch flags were used.
"""
import argparse
import hashlib
import json
import math
from pathlib import Path


def load(path):
    raw = Path(path).read_bytes()
    return json.loads(raw), hashlib.sha256(raw).hexdigest()


def compare_sequences(a, b):
    if len(a) != len(b):
        raise ValueError('trace lengths differ')
    first = next((i + 1 for i, (x, y) in enumerate(zip(a, b)) if x != y), None)
    return {'frames': len(a), 'exact': first is None, 'first_difference_frame': first}


def full_trace(report, key):
    trace = report[key]
    return trace['prefix'] + trace['baseline']


def compare_reports(a, b):
    if a.get('failure') or b.get('failure'):
        raise ValueError('cannot compare failed experiments')
    if a['scenario'] != b['scenario'] or a['checkpoint_after'] != b['checkpoint_after']:
        raise ValueError('scenario or checkpoint boundary differs')
    for key in ('traffic_lights_enabled', 'weather_enabled', 'traffic_manager_enabled',
                'collision_events_enabled', 'vehicle_lights_enabled', 'tm_seed'):
        if a.get(key) != b.get(key):
            raise ValueError('experiment option differs: {}'.format(key))
    left, right = full_trace(a, 'traces'), full_trace(b, 'traces')
    result = {'trajectory': compare_sequences(left, right), 'actors': {}, 'components': {},
              'scope': 'sampled uninterrupted traces; not a complete simulator or quality evaluation'}
    labels = [spec['id'] for spec in a['scenario']['actors']]
    for label in labels:
        result['actors'][label] = {}
        for field in ('location', 'rotation', 'velocity', 'angular_velocity', 'acceleration'):
            maximum, first = 0.0, None
            for index, (x, y) in enumerate(zip(left, right), 1):
                av, bv = x[label][field], y[label][field]
                if len(av) != len(bv) or any(not math.isfinite(v) for v in av + bv):
                    raise ValueError('invalid {} values for {}'.format(field, label))
                errors = [abs(p - q) for p, q in zip(av, bv)]
                if first is None and any(error != 0 for error in errors):
                    first = index
                maximum = max(maximum, *errors)
            result['actors'][label][field] = {'first_difference_frame': first,
                                               'max_absolute_component_difference': maximum}
    for key in ('tm_control_traces', 'light_traces', 'weather_traces', 'vehicle_light_traces', 'collision_traces'):
        ltrace, rtrace = full_trace(a, key), full_trace(b, key)
        if ltrace or rtrace:
            result['components'][key] = compare_sequences(ltrace, rtrace)
            if key == 'collision_traces':
                def count(trace):
                    return sum(len(events) for frame in trace for events in frame.values())
                result['components'][key]['event_records'] = [count(ltrace), count(rtrace)]
                result['components'][key]['diagnostic_only'] = True
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('reference')
    parser.add_argument('candidate')
    parser.add_argument('--output', required=True, help='new JSON file; existing files are preserved')
    args = parser.parse_args()
    a, ah = load(args.reference)
    b, bh = load(args.candidate)
    result = compare_reports(a, b)
    result['inputs'] = [{'path': args.reference, 'sha256': ah}, {'path': args.candidate, 'sha256': bh}]
    with open(args.output, 'x', encoding='utf-8') as output:
        json.dump(result, output, indent=2, sort_keys=True, allow_nan=False)
        output.write('\n')
    print('Compared {} frames; first trajectory difference: {}'.format(
        result['trajectory']['frames'], result['trajectory']['first_difference_frame']))


if __name__ == '__main__':
    main()
