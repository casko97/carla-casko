"""Record actions executed by the sole tick owner after a live checkpoint.

This adapter only records actions passed through it. It cannot intercept other
clients, Traffic Manager, sensors, or asynchronous simulator mutations.
"""
import copy
import hashlib
import json
from pathlib import Path

import checkpoint_live_vehicles as live
import checkpoint_journal_bundle as journal
import checkpoint_scenario_bundle as common
import multi_vehicle_physics_checkpoint as multi
import vehicle_physics_checkpoint as single


class LiveActionRecorder:
    def __init__(self, client, world, checkpoint_path, actors, props=None, actor_specs=None, traffic_ids=()):
        self.client, self.world = client, world
        self.checkpoint_path = Path(checkpoint_path)
        data = self.checkpoint_path.read_bytes()
        self.checkpoint_hash = hashlib.sha256(data).hexdigest()
        self.bundle = json.loads(data)
        if self.bundle.get('schema') != live.SCHEMA or self.bundle.get('identity') != common.fingerprint(self.bundle.get('map')):
            raise ValueError('checkpoint or selected artifacts differ')
        if self.bundle.get('walkers'):
            raise ValueError('this recorder does not record walker controls or lifecycle')
        self.vehicle_light_policy = live.light_policy.validate(self.bundle.get('vehicle_light_policy'))
        self.weather = copy.deepcopy(self.bundle['boundary_observation']['weather'])
        live.check_settings(world)
        if world.get_map().name.rsplit('/', 1)[-1] != self.bundle.get('map'):
            raise ValueError('world map does not match checkpoint')
        self.actors = actors  # lifecycle actions deliberately update caller's dictionary
        self.props = dict(props or {})
        self.extra_specs = copy.deepcopy(actor_specs or [])
        if set(actors) != set(self.bundle['actors']) or any(
                actors[label].id != slot['source_actor_id'] for label, slot in self.bundle['actors'].items()):
            raise ValueError('actors do not match the source checkpoint')
        if set(self.props) != set(self.bundle.get('dynamic_props', {})) or any(
                self.props[label].id != slot['source_actor_id'] for label, slot in self.bundle.get('dynamic_props', {}).items()):
            raise ValueError('props do not match the source checkpoint')
        self.frame = world.get_snapshot().frame
        if self.frame != self.bundle['source_frame']:
            raise ValueError('recorder must start at the captured source frame')
        self.lights = {}
        if traffic_ids:
            all_lights = multi.traffic_lights(world)
            if len(set(traffic_ids)) != len(traffic_ids) or not set(traffic_ids) <= set(all_lights):
                raise ValueError('traffic IDs must be unique visible OpenDRIVE IDs')
            self.lights = {label: all_lights[label] for label in traffic_ids}
        self.frames, self.vehicle_samples, self.prop_samples, self.traffic_samples = [], [], [], []
        self.vehicle_light_samples = []
        self.failed, self.closed = False, False
        self._check_world()

    def _check_world(self):
        snapshot = self.world.get_snapshot()
        if snapshot.frame != self.frame:
            raise RuntimeError('world advanced outside the recorder')
        if common.get_weather(self.world) != self.weather:
            raise RuntimeError('weather changed outside the recorder')
        world_actors = list(self.world.get_actors())
        vehicles = {a.id for a in world_actors if a.type_id.startswith('vehicle.')}
        if vehicles != {a.id for a in self.actors.values()}:
            raise RuntimeError('visible vehicle set changed outside the recorder')
        if any(a.type_id.startswith(('walker.', 'controller.ai.', 'sensor.')) for a in world_actors):
            raise RuntimeError('walkers, controllers and sensors are not recorded by this adapter')
        if any(snapshot.find(a.id) is None for a in list(self.actors.values()) + list(self.props.values())):
            raise RuntimeError('a recorded actor is absent from the source snapshot')
        if self.world.get_snapshot().frame != self.frame:
            raise RuntimeError('world advanced during recorder preflight')

    def step(self, action):
        if self.failed or self.closed:
            raise RuntimeError('recorder is failed or closed')
        # Copy before executing so caller changes cannot alter the recorded command.
        record = copy.deepcopy(action)
        _, specs = live.prepare_actions(self.bundle, {'actors': self.extra_specs, 'frames': self.frames + [record]})
        self._check_world()
        try:
            vehicle_sample = journal.execute(self.world, self.client, self.actors, record, specs, self.vehicle_light_policy)
            if self.world.get_snapshot().frame != self.frame + 1:
                raise RuntimeError('unexpected frame during recorded action')
            self.frame += 1
            prop_sample = {label: single.sample(self.world, actor) for label, actor in sorted(self.props.items())}
            light_sample = multi.sample_traffic_lights(self.lights) if self.lights else None
            vehicle_light_sample = (multi.sample_vehicle_lights(self.world, self.actors)
                                    if self.vehicle_light_policy is not None else None)
            self._check_world()
        except Exception:
            # A failed command batch or post-tick observation may already have changed the world.
            self.failed = True
            raise
        self.frames.append(record)
        self.vehicle_samples.append(vehicle_sample)
        self.prop_samples.append(prop_sample)
        if self.lights:
            self.traffic_samples.append(light_sample)
        if self.vehicle_light_policy is not None:
            self.vehicle_light_samples.append(vehicle_light_sample)
        return vehicle_sample

    def finish(self, output_dir):
        if self.failed or self.closed or not self.frames:
            raise RuntimeError('cannot finish a failed, closed, or empty recorder')
        self._check_world()
        if hashlib.sha256(self.checkpoint_path.read_bytes()).hexdigest() != self.checkpoint_hash:
            raise RuntimeError('checkpoint file changed while recording')
        if common.fingerprint(self.bundle.get('map')) != self.bundle['identity']:
            raise RuntimeError('selected artifacts changed while recording')
        output = Path(output_dir)
        output.mkdir(parents=True, exist_ok=False)
        actions = {'actors': self.extra_specs, 'frames': self.frames}
        common.atomic_create(output / 'actions.json', actions)
        common.atomic_create(output / 'vehicle_reference.json', self.vehicle_samples)
        common.atomic_create(output / 'prop_reference.json', self.prop_samples)
        if self.lights:
            common.atomic_create(output / 'traffic_reference.json', self.traffic_samples)
        if self.vehicle_light_policy is not None:
            common.atomic_create(output / 'vehicle_light_reference.json', self.vehicle_light_samples)
        manifest = {'schema': 'carla-live-action-recording-v1', 'checkpoint_sha256': self.checkpoint_hash,
                    'source_frame': self.bundle['source_frame'], 'last_frame': self.frame,
                    'frames': len(self.frames), 'traffic_ids': sorted(self.lights),
                    'vehicle_light_policy': self.vehicle_light_policy,
                    'limitations': ['only commands passed to this recorder are recorded',
                                    'sole tick owner and no external mutations required',
                                    'weather must remain fixed at the checkpoint value',
                                    'samples are not full simulator state']}
        # Manifest is written last; its presence indicates the other files were written.
        common.atomic_create(output / 'manifest.json', manifest)
        self.closed = True
        return manifest
