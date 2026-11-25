"""In-memory coordinator for reactive Traffic Manager branching.

One dedicated synchronous episode and a sole tick owner are required. This
does not survive a server or client restart, a map reload, or actor lifecycle
changes, and does not capture arbitrary scenario state. Slot RPCs are not an
atomic transaction; an error can leave partial state changes in the world.
"""
import copy

import checkpoint_fixed_props as fixed_props
import checkpoint_live_vehicles as live
import checkpoint_scenario_bundle as common
import multi_vehicle_physics_checkpoint as multi


class SameEpisodeTrafficCheckpoint:
    def __init__(self, client, world, traffic_manager, actors, fixed_props_guard):
        if (not isinstance(actors, dict) or not actors or
                any(not isinstance(name, str) or not name for name in actors) or
                len({actor.id for actor in actors.values()}) != len(actors)):
            raise ValueError('actors must be a nonempty label-to-unique-vehicle mapping')
        self.client, self.world, self.tm = client, world, traffic_manager
        self.actors = dict(actors)
        self.actor_ids = {name: actor.id for name, actor in actors.items()}
        self.guard = copy.deepcopy(fixed_props_guard)
        self.episode_id = world.id
        self.weather = None
        self.state = 'new'

    def _check_world(self):
        if self.client.get_world().id != self.episode_id or self.world.id != self.episode_id:
            raise RuntimeError('world episode changed')
        live.check_settings(self.world)
        before = self.world.get_snapshot().frame
        visible = list(self.world.get_actors())
        vehicles = {actor.id for actor in visible if actor.type_id.startswith('vehicle.')}
        if vehicles != set(self.actor_ids.values()):
            raise RuntimeError('visible vehicle set differs from checkpoint actors')
        if any(actor.type_id.startswith(('walker.', 'controller.ai.', 'sensor.')) for actor in visible):
            raise RuntimeError('walkers, AI walkers and sensors are outside this coordinator')
        if any(self.world.get_snapshot().find(actor_id) is None for actor_id in self.actor_ids.values()):
            raise RuntimeError('checkpoint vehicle missing from snapshot')
        fixed_props.validate_current(self.world, self.guard)
        weather = common.get_weather(self.world)
        if self.weather is not None and weather != self.weather:
            raise RuntimeError('weather changed since checkpoint')
        if self.world.get_snapshot().frame != before:
            raise RuntimeError('world advanced during checkpoint preflight')
        return before, weather

    def capture(self):
        if self.state != 'new':
            raise RuntimeError('coordinator cannot capture again')
        try:
            frame, weather = self._check_world()
            if (multi.checkpoint_actors(self.world, self.actors) != frame or
                    multi.checkpoint_traffic_lights(self.world) != frame or
                    multi.checkpoint_traffic_manager(self.world, self.tm) != frame):
                raise RuntimeError('world advanced during checkpoint')
            self.weather = weather
            self.state = 'ready'
            return frame
        except Exception:
            self.state = 'failed'
            raise

    def restore(self):
        if self.state != 'ready':
            raise RuntimeError('coordinator has no usable checkpoint')
        try:
            frame, _ = self._check_world()
            if (multi.checkpoint_actors(self.world, self.actors, restore=True) != frame or
                    multi.checkpoint_traffic_lights(self.world, restore=True) != frame or
                    multi.checkpoint_traffic_manager(self.world, self.tm, restore=True) != frame):
                raise RuntimeError('world advanced during restore')
            return frame
        except Exception:
            # Even a failed operation may already have mutated some slots or actors.
            self.state = 'failed'
            raise
