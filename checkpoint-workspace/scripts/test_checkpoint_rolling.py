import json
from types import SimpleNamespace
import unittest
from unittest import mock

from checkpoint_rolling import ReactivePrefixBuffer, RollingCheckpointBuffer


class World:
    def __init__(self):
        self.id = 42
        self.frame = 0

    def get_snapshot(self):
        return SimpleNamespace(frame=self.frame, timestamp=SimpleNamespace(elapsed_seconds=self.frame * .05))


class RollingBufferTests(unittest.TestCase):
    def setUp(self):
        self.world = World()
        self.buffer = RollingCheckpointBuffer()
        self.prefix = []

    def recipe(self):
        return {'schema': 'carla-tm-prefix-reconstruction-v1', 'prefix': self.prefix,
                'boundary': {'frame': self.world.frame}}

    def tick_to(self, frame):
        while self.world.frame <= frame:
            self.prefix.append({'steer': self.world.frame / 1000})
            self.buffer.observe(self.world, self.recipe)
            self.world.frame += 1

    def test_periodic_pretrigger_selection_pinning_and_copy(self):
        self.tick_to(207)  # 10.35 seconds; expected 5.0-second checkpoint
        selected = self.buffer.select_and_pin(207, 10.35)
        self.assertEqual((selected.frame, selected.elapsed_seconds), (100, 5.0))
        pinned_bytes = selected.recipe_bytes
        recipe = selected.recipe()
        recipe['prefix'].append({'steer': 999})
        self.tick_to(620)  # Evict other old entries.
        self.assertIn(selected, self.buffer.checkpoints)
        self.assertEqual(selected.recipe_bytes, pinned_bytes)
        self.assertEqual(selected.recipe()['boundary']['frame'], 100)
        self.assertNotEqual(selected.recipe()['prefix'][-1]['steer'], 999)
        self.buffer.release(selected)
        self.tick_to(660)
        self.assertNotIn(selected, self.buffer.checkpoints)

    def test_multiple_events_reference_count_pins(self):
        self.tick_to(120)
        a = self.buffer.select_and_pin(120, 6.0)
        b = self.buffer.select_and_pin(120, 6.0)
        self.assertIs(a, b)
        self.buffer.release(a)
        self.tick_to(460)
        self.assertIn(b, self.buffer.checkpoints)
        self.buffer.release(b)
        with self.assertRaisesRegex(ValueError, 'not pinned'):
            self.buffer.release(b)

    def test_insufficient_and_late_trigger_history(self):
        self.tick_to(20)
        with self.assertRaisesRegex(LookupError, 'no checkpoint'):
            self.buffer.select_and_pin(20, 1.0)
        with self.assertRaisesRegex(ValueError, 'not been observed'):
            self.buffer.select_and_pin(21, 1.05)
        self.tick_to(200)
        self.assertEqual(self.buffer.select_and_pin(120, 6.0).frame, 20)

    def test_pinned_capacity_failure_preserves_previous_recipe(self):
        self.buffer = RollingCheckpointBuffer(interval_seconds=.05, before_seconds=0,
                                              max_checkpoints=1, max_bytes=1000)
        self.tick_to(0)
        old = self.buffer.select_and_pin(0, 0)
        with self.assertRaisesRegex(RuntimeError, 'full of pinned'):
            self.tick_to(1)
        self.assertEqual(self.buffer.checkpoints, (old,))
        self.assertEqual(json.loads(old.recipe_bytes)['boundary']['frame'], 0)
        with self.assertRaisesRegex(RuntimeError, 'unusable'):
            self.buffer.observe(self.world, self.recipe)

    def test_byte_limit_and_nonfinite_data_fail_closed(self):
        self.buffer = RollingCheckpointBuffer(max_bytes=10)
        with self.assertRaisesRegex(RuntimeError, 'byte limit'):
            self.tick_to(0)
        self.assertEqual(self.buffer.checkpoints, ())
        self.buffer = RollingCheckpointBuffer()
        with self.assertRaises(ValueError):
            self.buffer.observe(self.world, lambda: {'schema': 'carla-tm-prefix-reconstruction-v1', 'x': float('nan')})

    def test_tick_or_episode_changed_during_capture_fails(self):
        def advances_world():
            self.world.frame += 1
            return self.recipe()
        with self.assertRaisesRegex(RuntimeError, 'world advanced'):
            self.buffer.observe(self.world, advances_world)
        self.assertEqual(self.buffer.checkpoints, ())
        self.buffer = RollingCheckpointBuffer()
        self.buffer.observe(self.world, self.recipe)
        self.world.frame += 1
        self.world.id += 1
        with self.assertRaisesRegex(RuntimeError, 'episode changed'):
            self.buffer.observe(self.world, self.recipe)

    def test_skipped_tick_and_invalid_config_are_rejected(self):
        self.buffer.observe(self.world, self.recipe)
        self.world.frame += 2
        with self.assertRaisesRegex(RuntimeError, 'consecutive'):
            self.buffer.observe(self.world, self.recipe)
        for kwargs in ({'interval_seconds': 0}, {'before_seconds': -1}, {'max_bytes': True},
                       {'interval_seconds': float('nan')}):
            with self.assertRaises(ValueError):
                RollingCheckpointBuffer(**kwargs)


class ReactivePrefixTests(unittest.TestCase):
    def setUp(self):
        self.world = World()
        self.buffer = ReactivePrefixBuffer(self.world, interval_seconds=.05, before_seconds=.05)

    def prepare(self, prefix):
        return {'schema': 'carla-tm-prefix-reconstruction-v1', 'prefix': prefix,
                'boundary': {'frame': self.world.frame}}

    def test_recipe_has_immutable_complete_action_prefix(self):
        self.world.frame = 1
        control = {'throttle': .4}
        first = self.buffer.observe(control, self.prepare)
        control['throttle'] = .8
        self.world.frame = 2
        second = self.buffer.observe({'brake': 1}, self.prepare)
        self.assertEqual(first.recipe()['prefix'], [{'throttle': .4}])
        self.assertEqual(second.recipe()['prefix'], [{'throttle': .4}, {'brake': 1}])
        selected = self.buffer.buffer.select_and_pin(2, .1)
        self.assertIs(selected, first)

    def test_missing_tick_and_prefix_mismatch_fail(self):
        self.world.frame = 2
        with self.assertRaisesRegex(RuntimeError, 'missed'):
            self.buffer.observe({}, self.prepare)
        self.buffer = ReactivePrefixBuffer(self.world, interval_seconds=.05)
        self.world.frame += 1
        with self.assertRaisesRegex(ValueError, 'complete recorded prefix'):
            self.buffer.observe({}, lambda prefix: {'schema': 'carla-tm-prefix-reconstruction-v1', 'prefix': []})
        with self.assertRaisesRegex(RuntimeError, 'unusable'):
            self.buffer.observe({}, self.prepare)

    def test_prefix_bounds_fail_without_overwriting_snapshot(self):
        self.buffer = ReactivePrefixBuffer(self.world, interval_seconds=.05, max_prefix_frames=1)
        self.world.frame += 1
        snapshot = self.buffer.observe({'throttle': .4}, self.prepare)
        self.world.frame += 1
        with self.assertRaisesRegex(RuntimeError, 'prefix exceeds'):
            self.buffer.observe({'throttle': .4}, self.prepare)
        self.assertEqual(self.buffer.buffer.checkpoints, (snapshot,))

    def test_adapter_calls_reactive_backend_and_ignores_mutable_slot(self):
        def prepare(*args):
            result = self.prepare(args[7])
            result['identity'] = {'selected_files_id': 'build-id'}
            return result, object()
        backend = SimpleNamespace(prepare_running_checkpoint=mock.Mock(
            side_effect=prepare))
        actor = SimpleNamespace(id=1, export_physics_checkpoint=mock.Mock(return_value=b'vehicle-state'))
        self.world.frame = 1
        with mock.patch.dict('sys.modules', {'checkpoint_tm_prefix_reconstruction': backend}):
            result = self.buffer.observe_reactive({'throttle': .4}, 'client', 'tm', {'expert': actor},
                                                  'guard', 'specs', 17, {'fog': 0})
        self.assertEqual(result.recipe()['prefix'], [{'throttle': .4}])
        self.assertEqual(result.recipe()['rolling_vehicle_checkpoints']['expert']['data_base64'], 'dmVoaWNsZS1zdGF0ZQ==')
        actor.export_physics_checkpoint.assert_called_once_with('build-id')
        backend.prepare_running_checkpoint.assert_called_once()


if __name__ == '__main__':
    unittest.main()
