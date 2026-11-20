#!/usr/bin/env python3
"""Smoke-test a running local CARLA server with this workspace's Python API."""
import argparse
import queue
import carla

parser = argparse.ArgumentParser()
parser.add_argument('--port', type=int, default=2000)
args = parser.parse_args()
client = carla.Client('127.0.0.1', args.port)
client.set_timeout(30.0)
print('Client:', client.get_client_version(), flush=True)
print('Server:', client.get_server_version(), flush=True)
world = client.get_world()
print('Map:', world.get_map().name, flush=True)
blueprints = world.get_blueprint_library().filter('vehicle.*')
spawn_points = world.get_map().get_spawn_points()
if not blueprints or not spawn_points:
    raise RuntimeError('Map has no vehicle blueprints or spawn points')
vehicle = None
camera = None
try:
    for transform in spawn_points[:20]:
        vehicle = world.try_spawn_actor(blueprints[0], transform)
        if vehicle is not None:
            break
    if vehicle is None:
        raise RuntimeError('Could not spawn a vehicle in the first 20 spawn points')
    print('Spawned:', vehicle.type_id, 'id', vehicle.id, flush=True)
    frame = world.wait_for_tick(30.0).frame
    print('Observed simulation frame:', frame, flush=True)
    camera_bp = world.get_blueprint_library().find('sensor.camera.rgb')
    camera_bp.set_attribute('image_size_x', '320')
    camera_bp.set_attribute('image_size_y', '240')
    camera = world.spawn_actor(camera_bp, carla.Transform(carla.Location(x=1.5, z=2.4)), attach_to=vehicle)
    images = queue.Queue()
    camera.listen(images.put)
    image = images.get(timeout=90)
    if (image.width, image.height, len(image.raw_data)) != (320, 240, 320 * 240 * 4):
        raise RuntimeError('Camera returned an unexpected image size')
    print('RGB camera:', image.width, 'x', image.height, 'frame', image.frame, 'bytes', len(image.raw_data), flush=True)
finally:
    if camera is not None:
        camera.stop()
        camera.destroy()
    if vehicle is not None:
        vehicle.destroy()
        print('Destroyed test vehicle', flush=True)
print('Smoke test passed', flush=True)
