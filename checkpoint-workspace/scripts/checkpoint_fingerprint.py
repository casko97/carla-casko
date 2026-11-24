#!/usr/bin/env python3
"""Fingerprint selected local checkpoint artifacts; not a complete asset manifest."""
import argparse
import hashlib
import json
import subprocess
from datetime import datetime, timezone
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PROJECT = ROOT / 'deterministic-cpt'
ENGINE = ROOT.parent / 'carla-casko-engine'


def digest(path):
    before = path.stat()
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            h.update(block)
    after = path.stat()
    if (before.st_ino, before.st_size, before.st_mtime_ns) != (after.st_ino, after.st_size, after.st_mtime_ns):
        raise RuntimeError('file changed while hashing: {}'.format(path))
    return {'path': str(path), 'bytes': before.st_size, 'sha256': h.hexdigest()}


def source(path):
    def git(*args):
        return subprocess.check_output(['git', '-C', str(path), *args])
    diff = git('diff', '--binary', 'HEAD', '--')
    status = git('status', '--porcelain=v1', '-z').decode('utf-8', 'surrogateescape')
    return {'head': git('rev-parse', 'HEAD').decode().strip(),
            'tracked_diff_sha256': hashlib.sha256(diff).hexdigest(),
            'status_entries': [v for v in status.split('\0') if v]}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--output', required=True)
    args = p.parse_args()
    maps = PROJECT / 'Unreal/CarlaUE4/Content/Carla/Maps'
    files = [ENGINE / 'Engine/Binaries/Linux/UE4Editor',
             PROJECT / 'Unreal/CarlaUE4/Plugins/Carla/Binaries/Linux/libUE4Editor-Carla.so',
             PROJECT / 'PythonAPI/carla/dist/carla-0.9.15-py3.8-linux-x86_64.egg',
             ROOT / 'scripts/checkpoint_lifecycle_replay.py',
             PROJECT / 'PythonAPI/examples/vehicle_physics_checkpoint.py',
             maps / 'Town01.umap', maps / 'Town01_BuiltData.uasset',
             maps / 'OpenDrive/Town01.xodr', maps / 'TM/Town01.bin', maps / 'Nav/Town01.bin']
    present = [digest(path) for path in files if path.is_file()]
    missing = [str(path) for path in files if not path.is_file()]
    data = {'generated_utc': datetime.now(timezone.utc).isoformat(),
            'scope': 'Selected on-disk files only; not all engine modules, assets, runtime-loaded bytes, or untracked source contents.',
            'source': {'project': source(PROJECT), 'engine': source(ENGINE)},
            'files': present, 'missing': missing}
    Path(args.output).write_text(json.dumps(data, indent=2, sort_keys=True) + '\n')
    print('Hashed {} selected files; {} missing. No complete build identity claimed.'.format(len(present), len(missing)))


if __name__ == '__main__':
    main()
