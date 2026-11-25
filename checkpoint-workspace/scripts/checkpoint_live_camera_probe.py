#!/usr/bin/env python3
"""Capture real CARLA RGB frames during same-episode checkpoint branches."""
import argparse
import hashlib
import json
import math
import queue
import subprocess
import sys
import traceback
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path[:0] = [str(ROOT / 'scripts'), str(ROOT / 'deterministic-cpt/PythonAPI/examples')]
import carla
import cv2
import numpy as np
import checkpoint_scenario_bundle as common
import checkpoint_tm_prefix_reconstruction as reactive
from checkpoint_tm_same_episode import SameEpisodeTrafficCheckpoint
from checkpoint_tm_reactive_probe import record_step
import multi_vehicle_physics_checkpoint as multi


def image_for_frame(q, frame):
    while True:
        image = q.get(timeout=60)
        if image.frame == frame:
            return image
        if image.frame > frame:
            raise RuntimeError('camera missed frame {}'.format(frame))


def draw_overlay(frame, text, tick, total):
    cv2.rectangle(frame, (0, 0), (960, 92), (0, 0, 0), -1)
    cv2.putText(frame, text, (20, 31), cv2.FONT_HERSHEY_SIMPLEX, .8, (255, 255, 255), 2)
    cv2.putText(frame, 'LIVE CARLA RGB CAMERA - FRAME SYNCHRONIZED', (20, 59),
                cv2.FONT_HERSHEY_SIMPLEX, .52, (175, 230, 255), 1)
    cv2.putText(frame, 'tick {}/{}   t={:.2f}s'.format(tick + 1, total, (tick + 1) * .05),
                (20, 82), cv2.FONT_HERSHEY_SIMPLEX, .55, (240, 240, 240), 1)


def record_branch(client, world, actors, output, label, actions, transform):
    camera = None
    writer = None
    recorder_started = False
    try:
        bp = world.get_blueprint_library().find('sensor.camera.rgb')
        bp.set_attribute('image_size_x', '960')
        bp.set_attribute('image_size_y', '720')
        bp.set_attribute('fov', '80')
        camera = world.spawn_actor(bp, transform)
        frames = queue.Queue()
        camera.listen(frames.put)
        path = output / (label + '.mp4')
        writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*'mp4v'), 20, (960, 720))
        if not writer.isOpened():
            raise RuntimeError('cannot create video')
        response = client.start_recorder(str(output / (label + '.log')), True)
        if 'error' in response.lower():
            raise RuntimeError(response)
        recorder_started = True
        results, digests = [], []
        for i, action in enumerate(actions):
            states = record_step(client, world, actors, action)
            frame = world.get_snapshot().frame
            image = image_for_frame(frames, frame)
            pixels = np.frombuffer(image.raw_data, dtype=np.uint8).reshape((720, 960, 4))[:, :, :3].copy()
            digests.append(hashlib.sha256(pixels.tobytes()).hexdigest())
            draw_overlay(pixels, 'ORIGINAL SIMULATION' if label == 'original' else 'RESTORED SIMULATION', i, len(actions))
            writer.write(pixels)
            states['ego_control'] = multi.sample_tm_controls(actors, ['expert'])['expert']
            states['lights'] = multi.sample_traffic_lights(multi.traffic_lights(world))
            states['camera_frame'] = image.frame
            states['world_frame'] = frame
            results.append(states)
        return results, digests
    finally:
        if recorder_started:
            client.stop_recorder()
        if writer is not None:
            writer.release()
        if camera is not None:
            camera.stop()
            camera.destroy()


def comparable(frame):
    return {key: value for key, value in frame.items() if key not in ('camera_frame', 'world_frame')}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--port', type=int, default=41000)
    p.add_argument('--tm-port', type=int, default=9100)
    p.add_argument('--output', required=True)
    p.add_argument('--ticks', type=int, default=120)
    args = p.parse_args()
    output = Path(args.output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    client = carla.Client('127.0.0.1', args.port)
    client.set_timeout(360)
    original_settings = client.get_world().get_settings()
    actors, tm = {}, None
    report = {'scope': 'real RGB frames from controlled same-episode CARLA simulation; fixed props; no ScenarioRunner or policy model',
              'failure': None}
    try:
        world, guard, specs, actors, tm = reactive.setup(client, 'Town01', args.tm_port, 17)
        for _ in range(60):
            record_step(client, world, actors, {'brake': 1.0})
        checkpoint = SameEpisodeTrafficCheckpoint(client, world, tm, actors, guard)
        report['checkpoint_frame'] = checkpoint.capture()
        point_a = actors['expert'].get_location()
        point_b = actors['traffic'].get_location()
        transform = carla.Transform(carla.Location(x=(point_a.x + point_b.x) / 2,
                                                    y=(point_a.y + point_b.y) / 2 + 4,
                                                    z=40), carla.Rotation(pitch=-90))
        actions = [{'throttle': .4} for _ in range(args.ticks)]
        baseline, baseline_images = record_branch(client, world, actors, output, 'original', actions, transform)
        # CARLA's client-side actor list may still reflect the last streamed
        # snapshot after sensor destruction. Advance once so the coordinator
        # can confirm that no sensors remain before restoring its slots.
        world.tick()
        if any(a.type_id.startswith('sensor.') for a in world.get_actors()):
            raise RuntimeError('camera remained visible after destruction and cleanup tick')
        checkpoint.restore()
        restored, restored_images = record_branch(client, world, actors, output, 'restored', actions, transform)
        report['original'] = baseline
        report['restored'] = restored
        report['camera_frame_aligned'] = all(x['camera_frame'] == x['world_frame'] for x in baseline + restored)
        report['exact_sampled_state_match'] = all(comparable(a) == comparable(b) for a, b in zip(baseline, restored))
        report['first_state_difference_tick'] = next((i + 1 for i, (a, b) in enumerate(zip(baseline, restored))
                                                       if comparable(a) != comparable(b)), None)
        report['max_position_error_m'] = {label: max(math.dist(a['states'][label]['location'],
            b['states'][label]['location']) for a, b in zip(baseline, restored)) for label in actors}
        report['camera_raw_frames_matching'] = sum(a == b for a, b in zip(baseline_images, restored_images))
        report['camera_frame_count'] = len(baseline_images)
        report['original_camera_sha256'] = baseline_images
        report['restored_camera_sha256'] = restored_images
        subprocess.run(['ffmpeg', '-v', 'error', '-n', '-i', str(output / 'original.mp4'),
                        '-i', str(output / 'restored.mp4'), '-filter_complex', 'hstack=inputs=2',
                        '-c:v', 'libx264', '-crf', '18', '-pix_fmt', 'yuv420p',
                        str(output / 'side_by_side.mp4')], check=True)
    except Exception as exc:
        report['failure'] = {'type': type(exc).__name__, 'message': str(exc), 'traceback': traceback.format_exc()}
        raise
    finally:
        try:
            reactive.close(tm, actors, args.tm_port)
            client.get_world().apply_settings(original_settings)
        finally:
            (output / 'report.json').write_text(json.dumps(report, indent=2, allow_nan=False))
            print(json.dumps({k: v for k, v in report.items() if k not in ('original', 'restored',
                'original_camera_sha256', 'restored_camera_sha256')}, indent=2), flush=True)


if __name__ == '__main__':
    main()
