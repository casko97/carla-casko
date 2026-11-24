"""Portable traffic-light RPC adapter for the experimental live harness."""
import base64
import hashlib
import subprocess
import tempfile
from pathlib import Path


def rpc(helper, port, operation, path, build_id):
    result = subprocess.run([helper, str(port), operation, str(path), build_id],
                            capture_output=True, text=True, timeout=45)
    if result.returncode:
        raise RuntimeError('traffic-light {} failed: {}'.format(operation, result.stderr.strip() or result.stdout.strip()))


def capture(world, helper, port, build_id, frame):
    with tempfile.TemporaryDirectory(prefix='carla-live-lights-') as tmp:
        path = Path(tmp) / 'checkpoint'
        rpc(helper, port, 'export', path, build_id)
        data = path.read_bytes()
    if not 0 < len(data) <= 4 * 1024 * 1024 or world.get_snapshot().frame != frame:
        raise RuntimeError('invalid traffic-light record or world advanced')
    return {'data_base64': base64.b64encode(data).decode('ascii'), 'sha256': hashlib.sha256(data).hexdigest()}


def decode(slot):
    data = base64.b64decode(slot['data_base64'], validate=True)
    if not 0 < len(data) <= 4 * 1024 * 1024 or hashlib.sha256(data).hexdigest() != slot['sha256']:
        raise ValueError('invalid traffic-light checkpoint data')
    return data


def restore(world, data, helper, port, build_id, frame):
    with tempfile.TemporaryDirectory(prefix='carla-live-lights-') as tmp:
        path = Path(tmp) / 'checkpoint'
        path.write_bytes(data)
        rpc(helper, port, 'import', path, build_id)
    if world.get_snapshot().frame != frame:
        raise RuntimeError('world advanced during traffic-light restore')
