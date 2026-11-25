#!/usr/bin/env python3
"""Render saved checkpoint-test trajectories; this is not original camera data."""
import argparse
import json
import queue
import subprocess
from pathlib import Path

import carla
import cv2
import numpy as np


def image_for_frame(images, frame):
    while True:
        image = images.get(timeout=60)
        if image.frame == frame:
            return image
        if image.frame > frame:
            raise RuntimeError('camera frame {} passed requested {}'.format(image.frame, frame))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--report', required=True)
    parser.add_argument('--recipe', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--port', type=int, default=39000)
    args = parser.parse_args()
    report = json.loads(Path(args.report).read_text())
    recipe = json.loads(Path(args.recipe).read_text())
    baseline = [entry['observation'] for entry in report['baseline']]
    candidate = report['candidate']
    if len(baseline) != len(candidate) or not baseline:
        raise ValueError('trajectory lengths differ')
    output = Path(args.output)
    output.mkdir(parents=True, exist_ok=False)
    client = carla.Client('127.0.0.1', args.port)
    client.set_timeout(120)
    initial = client.get_world()
    settings = initial.get_settings()
    settings.synchronous_mode = True
    settings.fixed_delta_seconds = .05
    settings.no_rendering_mode = False
    initial.apply_settings(settings)
    world = client.load_world('Town01', reset_settings=False)
    weather = world.get_weather()
    for key, value in recipe['initial_weather'].items():
        setattr(weather, key, value)
    world.set_weather(weather)
    actors, camera = {}, None
    try:
        points = [row['states'][name]['location'] for row in baseline + candidate for name in ('expert', 'traffic')]
        center_x = (min(p[0] for p in points) + max(p[0] for p in points)) / 2
        center_y = (min(p[1] for p in points) + max(p[1] for p in points)) / 2
        library = world.get_blueprint_library()
        for name, color in [('expert', '40,120,255'), ('traffic', '255,130,25')]:
            bp = library.find('vehicle.lincoln.mkz_2020')
            bp.set_attribute('color', color)
            loc = baseline[0]['states'][name]['location']
            pose = carla.Transform(carla.Location(x=loc[0], y=loc[1], z=loc[2] + 3))
            actor = world.try_spawn_actor(bp, pose)
            if actor is None:
                raise RuntimeError('could not spawn ' + name)
            actors[name] = actor
            actor.set_simulate_physics(False)
        camera_bp = library.find('sensor.camera.rgb')
        camera_bp.set_attribute('image_size_x', '960')
        camera_bp.set_attribute('image_size_y', '720')
        camera_bp.set_attribute('fov', '80')
        camera = world.spawn_actor(camera_bp, carla.Transform(
            carla.Location(x=center_x, y=center_y, z=40), carla.Rotation(pitch=-90)))
        images = queue.Queue()
        camera.listen(images.put)
        for title, sequence, filename in [('ORIGINAL TRACE', baseline, 'original.mp4'),
                                          ('RECONSTRUCTED TRACE', candidate, 'reconstructed.mp4')]:
            path = output / filename
            writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*'mp4v'), 20, (960, 720))
            if not writer.isOpened():
                raise RuntimeError('could not open video writer')
            try:
                for index, row in enumerate(sequence):
                    commands = []
                    for name, actor in actors.items():
                        state = row['states'][name]
                        x, y, z = state['location']
                        pitch, yaw, roll = state['rotation']
                        commands.append(carla.command.ApplyTransform(actor.id, carla.Transform(
                            carla.Location(x=x, y=y, z=z), carla.Rotation(pitch=pitch, yaw=yaw, roll=roll))))
                    responses = client.apply_batch_sync(commands, False)
                    if len(responses) != len(commands) or any(result.error for result in responses):
                        raise RuntimeError('pose batch failed')
                    frame = world.tick()
                    image = image_for_frame(images, frame)
                    pixels = np.frombuffer(image.raw_data, dtype=np.uint8).reshape((720, 960, 4))[:, :, :3].copy()
                    cv2.rectangle(pixels, (0, 0), (960, 100), (0, 0, 0), -1)
                    cv2.putText(pixels, title, (20, 33), cv2.FONT_HERSHEY_SIMPLEX, .85, (255, 255, 255), 2)
                    cv2.putText(pixels, 'RENDERED FROM SAVED POSITIONS - NOT ORIGINAL CAMERA', (20, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, .52, (180, 220, 255), 1)
                    cv2.putText(pixels, 'Frame {}/{}   t={:.2f}s   blue: ego  orange: traffic'.format(
                        index + 1, len(sequence), (index + 1) * .05), (20, 86),
                        cv2.FONT_HERSHEY_SIMPLEX, .58, (240, 240, 240), 1)
                    writer.write(pixels)
            finally:
                writer.release()
            print('saved', path, flush=True)
        subprocess.run(['ffmpeg', '-v', 'error', '-n', '-i', str(output / 'original.mp4'),
                        '-i', str(output / 'reconstructed.mp4'), '-filter_complex', 'hstack=inputs=2',
                        '-c:v', 'libx264', '-crf', '18', '-pix_fmt', 'yuv420p',
                        str(output / 'side_by_side.mp4')], check=True)
        print('saved', output / 'side_by_side.mp4', flush=True)
    finally:
        if camera:
            camera.stop()
            camera.destroy()
        for actor in actors.values():
            actor.destroy()
        settings = world.get_settings()
        settings.synchronous_mode = False
        settings.fixed_delta_seconds = None
        world.apply_settings(settings)


if __name__ == '__main__':
    main()
