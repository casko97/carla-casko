"""Bounded, immutable prefix-reconstruction recipes for a synchronous episode.

This stores recipes, not native TM snapshots. Reconstructing one requires the
restricted reactive prefix-reconstruction backend and a dedicated server.
"""
from collections import Counter, deque
import base64
from dataclasses import dataclass
import hashlib
import json
import math


@dataclass(frozen=True)
class RollingCheckpoint:
    frame: int
    elapsed_seconds: float
    episode_id: int
    recipe_bytes: bytes

    @property
    def sha256(self):
        return hashlib.sha256(self.recipe_bytes).hexdigest()

    def recipe(self):
        return json.loads(self.recipe_bytes)


def _seconds(value, name, *, positive=False):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError('{} must be finite'.format(name))
    if value < 0 or (positive and value == 0):
        raise ValueError('{} must be {}'.format(name, 'positive' if positive else 'non-negative'))
    return float(value)


class RollingCheckpointBuffer:
    """Capture recipes every interval and pin the latest before a trigger time.

    The caller owns every world tick and mutation and must call observe after
    every completed tick. The factory must generate a complete validated recipe
    for that boundary, including the prefix; no mutable native slot is retained.
    """

    def __init__(self, interval_seconds=1.0, before_seconds=5.0,
                 max_checkpoints=16, max_bytes=128 * 1024 * 1024):
        self.interval_seconds = _seconds(interval_seconds, 'interval_seconds', positive=True)
        self.before_seconds = _seconds(before_seconds, 'before_seconds')
        for name, value in (('max_checkpoints', max_checkpoints), ('max_bytes', max_bytes)):
            if type(value) is not int or value <= 0:
                raise ValueError('{} must be a positive integer'.format(name))
        self.max_checkpoints, self.max_bytes = max_checkpoints, max_bytes
        self._entries = deque()
        self._pins = Counter()
        self._bytes = 0
        self._last_frame = None
        self._last_time = None
        self._last_capture_time = None
        self._episode_id = None
        self._failed = False

    @property
    def bytes_used(self):
        return self._bytes

    @property
    def checkpoints(self):
        return tuple(self._entries)

    def observe(self, world, recipe_factory):
        """Observe one tick, capturing a recipe if due. Returns entry or None."""
        if self._failed:
            raise RuntimeError('buffer is unusable after capture failure')
        try:
            before = world.get_snapshot()
            frame = before.frame
            timestamp = _seconds(before.timestamp.elapsed_seconds, 'simulation time')
            episode_id = world.id
            if type(frame) is not int or frame < 0:
                raise ValueError('invalid simulation frame')
            if self._episode_id is not None and episode_id != self._episode_id:
                raise RuntimeError('world episode changed')
            if self._last_frame is not None and (frame != self._last_frame + 1 or timestamp <= self._last_time):
                raise RuntimeError('ticks must be consecutive with increasing simulation time')
            due = self._last_capture_time is None or timestamp - self._last_capture_time + 1e-9 >= self.interval_seconds
            if not due:
                self._episode_id, self._last_frame, self._last_time = episode_id, frame, timestamp
                return None
            recipe = recipe_factory()
            after = world.get_snapshot()
            if world.id != episode_id or after.frame != frame or after.timestamp.elapsed_seconds != timestamp:
                raise RuntimeError('world advanced or episode changed during capture')
            if not isinstance(recipe, dict) or recipe.get('schema') != 'carla-tm-prefix-reconstruction-v1':
                raise ValueError('factory did not produce a reactive prefix reconstruction recipe')
            encoded = json.dumps(recipe, sort_keys=True, separators=(',', ':'), allow_nan=False).encode('utf-8')
            candidate = RollingCheckpoint(frame, timestamp, episode_id, encoded)
            if len(encoded) > self.max_bytes:
                raise RuntimeError('checkpoint exceeds buffer byte limit')
            # Select evictions before mutating the buffer. Pinned entries may
            # not be evicted, even if all capacity is exhausted.
            prospective = list(self._entries)
            used = self._bytes
            while len(prospective) >= self.max_checkpoints or used + len(encoded) > self.max_bytes:
                victim = next((item for item in prospective if not self._pins[item.frame]), None)
                if victim is None:
                    raise RuntimeError('checkpoint buffer is full of pinned entries')
                prospective.remove(victim)
                used -= len(victim.recipe_bytes)
            prospective.append(candidate)
            self._entries = deque(prospective)
            self._bytes = used + len(encoded)
            self._episode_id, self._last_frame, self._last_time = episode_id, frame, timestamp
            self._last_capture_time = timestamp
            return candidate
        except Exception:
            # The factory may have partially mutated native checkpoint slots.
            # Existing encoded recipes remain readable and selectable.
            self._failed = True
            raise

    def select_and_pin(self, trigger_frame, trigger_seconds, before_seconds=None):
        """Pin latest available recipe no later than trigger minus offset."""
        trigger_seconds = _seconds(trigger_seconds, 'trigger_seconds')
        offset = self.before_seconds if before_seconds is None else _seconds(before_seconds, 'before_seconds')
        if type(trigger_frame) is not int or trigger_frame < 0:
            raise ValueError('invalid trigger frame')
        if self._last_frame is None or trigger_frame > self._last_frame or trigger_seconds > self._last_time + 1e-9:
            raise ValueError('trigger has not been observed')
        cutoff = trigger_seconds - offset
        candidates = [item for item in self._entries if item.frame <= trigger_frame and item.elapsed_seconds <= cutoff + 1e-9]
        if not candidates:
            raise LookupError('no checkpoint is available at the requested pre-trigger time')
        result = candidates[-1]
        self._pins[result.frame] += 1
        return result

    def release(self, checkpoint):
        if not any(checkpoint is item for item in self._entries) or not self._pins[checkpoint.frame]:
            raise ValueError('checkpoint is not pinned by this buffer')
        self._pins[checkpoint.frame] -= 1
        if not self._pins[checkpoint.frame]:
            del self._pins[checkpoint.frame]


class ReactivePrefixBuffer:
    """Journal ego actions from setup and capture independent TM recipes.

    Construct immediately after the restricted scenario setup and before the
    first controlled tick. For every later tick pass the action actually sent
    before it. The caller is responsible for preventing all unrecorded world,
    actor and TM mutations. Recipe preparation must be the existing validated
    prepare_running_checkpoint backend; this wrapper verifies its prefix.
    """

    def __init__(self, world, *, interval_seconds=1.0, before_seconds=5.0,
                 max_checkpoints=16, max_bytes=128 * 1024 * 1024,
                 max_prefix_frames=10000, max_prefix_bytes=8 * 1024 * 1024):
        if type(max_prefix_frames) is not int or not 1 <= max_prefix_frames <= 10000:
            raise ValueError('max_prefix_frames must be an integer between 1 and 10000')
        if type(max_prefix_bytes) is not int or max_prefix_bytes <= 0:
            raise ValueError('max_prefix_bytes must be a positive integer')
        self.world = world
        self.episode_id = world.id
        self.start_frame = world.get_snapshot().frame
        self.buffer = RollingCheckpointBuffer(interval_seconds, before_seconds, max_checkpoints, max_bytes)
        self.max_prefix_frames = max_prefix_frames
        self.max_prefix_bytes = max_prefix_bytes
        self.prefix_bytes = 0
        self._actions = []
        self._failed = False

    def observe(self, applied_control, prepare_recipe):
        """Record one action after its tick and capture a validated recipe if due."""
        if self._failed:
            raise RuntimeError('reactive prefix buffer is unusable after failure')
        try:
            frame = self.world.get_snapshot().frame
            if self.world.id != self.episode_id or frame != self.start_frame + len(self._actions) + 1:
                raise RuntimeError('episode changed or a controlled tick was missed')
            if not isinstance(applied_control, dict):
                raise ValueError('control must be a dictionary')
            data = json.dumps(applied_control, sort_keys=True, separators=(',', ':'), allow_nan=False).encode('utf-8')
            if len(self._actions) >= self.max_prefix_frames or self.prefix_bytes + len(data) > self.max_prefix_bytes:
                raise RuntimeError('recorded prefix exceeds its configured limit')
            self._actions.append(data)
            self.prefix_bytes += len(data)

            def make_recipe():
                prefix = [json.loads(action) for action in self._actions]
                recipe = prepare_recipe(prefix)
                if not isinstance(recipe, dict) or recipe.get('prefix') != prefix:
                    raise ValueError('recipe does not contain the complete recorded prefix')
                return recipe

            result = self.buffer.observe(self.world, make_recipe)
            if self.world.get_snapshot().frame != frame or self.world.id != self.episode_id:
                raise RuntimeError('world advanced during observation')
            return result
        except Exception:
            self._failed = True
            raise

    def observe_reactive(self, applied_control, client, traffic_manager, actors, guard,
                         specs, seed, initial_weather, tm_config=None):
        """Capture using the existing restricted reactive checkpoint backend."""
        import checkpoint_tm_prefix_reconstruction as reactive

        def prepare(prefix):
            recipe, _slot = reactive.prepare_running_checkpoint(
                client, self.world, traffic_manager, actors, guard, specs,
                seed, prefix, initial_weather, tm_config)
            # The reactive recipe reconstructs TM state by prefix replay. Save
            # the independent native vehicle snapshots as well, so later
            # captures cannot replace the only copy of those physics fields.
            build_id = recipe['identity']['selected_files_id']
            vehicles = {}
            for label, actor in sorted(actors.items()):
                data = bytes(actor.export_physics_checkpoint(build_id))
                if not 0 < len(data) <= 1024 * 1024:
                    raise RuntimeError('invalid exported vehicle checkpoint for {}'.format(label))
                vehicles[label] = {'actor_id': actor.id,
                                   'sha256': hashlib.sha256(data).hexdigest(),
                                   'data_base64': base64.b64encode(data).decode('ascii')}
            recipe['rolling_vehicle_checkpoints'] = vehicles
            return recipe

        return self.observe(applied_control, prepare)
