#!/usr/bin/env python3
"""Probe rejected portable traffic-light imports on a dedicated server."""
import argparse
import base64
import json
import subprocess
from pathlib import Path

import carla
import checkpoint_scenario_bundle as common
import multi_vehicle_physics_checkpoint as multi
import vehicle_physics_checkpoint as single


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--checkpoint', required=True)
    parser.add_argument('--rpc', required=True)
    parser.add_argument('--mutator', required=True)
    parser.add_argument('--output-dir', required=True)
    parser.add_argument('--port', type=int, default=27000)
    args = parser.parse_args()
    out = Path(args.output_dir)
    out.mkdir(parents=True, exist_ok=False)
    bundle = json.loads(Path(args.checkpoint).read_text())
    data = base64.b64decode(bundle['traffic_lights']['data_base64'], validate=True)
    source = out / 'traffic.msgpack'
    source.write_bytes(data)
    bad = out / 'bad-topology.msgpack'
    subprocess.run([args.mutator, str(source), str(bad)], check=True, timeout=30)
    client = carla.Client('127.0.0.1', args.port)
    client.set_timeout(180)
    settings = client.get_world().get_settings()
    report = {'checks': {}, 'failure': None}
    try:
        world = single.load_world(client, bundle['map'])
        lights = multi.traffic_lights(world)
        frame = world.get_snapshot().frame
        before = multi.sample_traffic_lights(lights)
        for name, file, build_id in (
                ('wrong_build', source, 'different-build'),
                ('bad_topology', bad, bundle['identity']['selected_files_id'])):
            proc = subprocess.run([args.rpc, str(args.port), 'import', str(file), build_id],
                                  capture_output=True, text=True, timeout=45)
            report['checks'][name + '_rejected'] = proc.returncode == 1
            report['checks'][name + '_no_tick'] = world.get_snapshot().frame == frame
            report['checks'][name + '_visible_state_unchanged'] = multi.sample_traffic_lights(lights) == before
            if not all(report['checks'].values()):
                raise RuntimeError('{} failed: {} {}'.format(name, proc.returncode, proc.stderr))
        proc = subprocess.run([args.rpc, str(args.port), 'import', str(source), bundle['identity']['selected_files_id']],
                              capture_output=True, text=True, timeout=45)
        report['checks']['compatible_import_accepted'] = proc.returncode == 0
        report['checks']['compatible_import_no_tick'] = world.get_snapshot().frame == frame
        if not all(report['checks'].values()):
            raise RuntimeError('compatible import failed: {} {}'.format(proc.returncode, proc.stderr))
        print(json.dumps(report['checks'], indent=2))
    except Exception as exc:
        report['failure'] = {'type': type(exc).__name__, 'message': str(exc)}
        raise
    finally:
        common.atomic_create(out / 'report.json', report)
        client.get_world().apply_settings(settings)


if __name__ == '__main__':
    main()
