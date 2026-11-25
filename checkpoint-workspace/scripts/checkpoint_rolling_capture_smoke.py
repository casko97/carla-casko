"""Capture-only rolling buffer smoke test on a disposable custom server."""
import argparse
import base64
import hashlib
import json
import sys
from pathlib import Path

root = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(root / 'scripts'))
sys.path.insert(1, str(root / 'deterministic-cpt/checkpoint-workspace/scripts'))
sys.path.insert(2, str(root / 'deterministic-cpt/PythonAPI/examples'))
import carla
import checkpoint_rolling as rolling
import checkpoint_scenario_bundle as common
import checkpoint_tm_prefix_reconstruction as reactive
import multi_vehicle_physics_checkpoint as multi

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--port', type=int, default=36000)
parser.add_argument('--tm-port', type=int, default=8600)
parser.add_argument('--report', required=True)
args = parser.parse_args()
if Path(args.report).exists():
    raise FileExistsError(args.report)
client = carla.Client('127.0.0.1', args.port)
client.set_timeout(360)
original = client.get_world().get_settings()
actors, tm = {}, None
try:
    world, guard, specs, actors, tm = reactive.setup(client, 'Town01', args.tm_port, 17)
    weather = common.get_weather(world)
    buffer = rolling.ReactivePrefixBuffer(world)
    action = {'brake': 1.0}
    for _ in range(140):
        multi.step(client, world, actors, {'expert': action})
        before = world.get_snapshot().frame
        result = buffer.observe_reactive(action, client, tm, actors, guard, specs, 17, weather)
        assert world.get_snapshot().frame == before
    trigger = world.get_snapshot()
    selected = buffer.buffer.select_and_pin(trigger.frame, trigger.timestamp.elapsed_seconds)
    saved = selected.recipe_bytes
    for _ in range(80):
        multi.step(client, world, actors, {'expert': action})
        before = world.get_snapshot().frame
        buffer.observe_reactive(action, client, tm, actors, guard, specs, 17, weather)
        assert world.get_snapshot().frame == before
    assert selected.recipe_bytes == saved
    assert selected.recipe()['prefix'] and selected in buffer.buffer.checkpoints
    vehicles = selected.recipe()['rolling_vehicle_checkpoints']
    assert set(vehicles) == set(actors)
    for item in vehicles.values():
        payload = base64.b64decode(item['data_base64'], validate=True)
        assert hashlib.sha256(payload).hexdigest() == item['sha256']
    report = {'selected_frame': selected.frame, 'trigger_frame': trigger.frame,
              'selected_time': selected.elapsed_seconds, 'trigger_time': trigger.timestamp.elapsed_seconds,
              'actual_lookback_seconds': trigger.timestamp.elapsed_seconds - selected.elapsed_seconds,
              'retained_entries': len(buffer.buffer.checkpoints), 'recipe_bytes': buffer.buffer.bytes_used,
              'selected_sha256': selected.sha256, 'selected_prefix_frames': len(selected.recipe()['prefix']),
              'portable_vehicle_checkpoints': len(vehicles),
              'pinned_recipe_unchanged': True, 'capture_advanced_world': False,
              'scope': 'restricted Town01 scenario; capture and retention only; no restore comparison'}
    with open(args.report, 'x', encoding='utf-8') as stream:
        json.dump(report, stream, indent=2, allow_nan=False)
        stream.write('\n')
    print(json.dumps(report, indent=2), flush=True)
finally:
    reactive.close(tm, actors, args.tm_port)
    client.get_world().apply_settings(original)
