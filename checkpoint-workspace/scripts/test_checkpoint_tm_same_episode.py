"""Coordinator preconditions and partial-failure behavior."""
import unittest
from types import SimpleNamespace
from unittest import mock

import checkpoint_tm_same_episode as checkpoint


class TrafficCoordinatorTests(unittest.TestCase):
    def setUp(self):
        self.actor = SimpleNamespace(id=7, type_id='vehicle.test')
        self.world = mock.Mock()
        self.world.id = 41
        self.world.get_actors.return_value = [self.actor]
        self.world.get_snapshot.return_value = SimpleNamespace(frame=10, find=lambda _: self.actor)
        self.client = mock.Mock()
        self.client.get_world.return_value = self.world
        self.patches = [mock.patch.object(checkpoint.live, 'check_settings'),
                        mock.patch.object(checkpoint.fixed_props, 'validate_current'),
                        mock.patch.object(checkpoint.common, 'get_weather', return_value={'cloudiness': 0}),
                        mock.patch.object(checkpoint.multi, 'checkpoint_actors', return_value=10),
                        mock.patch.object(checkpoint.multi, 'checkpoint_traffic_lights', return_value=10),
                        mock.patch.object(checkpoint.multi, 'checkpoint_traffic_manager', return_value=10)]
        self.mocks = [p.start() for p in self.patches]
        for p in self.patches:
            self.addCleanup(p.stop)

    def make(self):
        return checkpoint.SameEpisodeTrafficCheckpoint(self.client, self.world, mock.Mock(), {'ego': self.actor}, {})

    def test_capture_and_repeated_restore(self):
        c = self.make()
        self.assertEqual(c.capture(), 10)
        self.assertEqual(c.restore(), 10)
        self.assertEqual(c.restore(), 10)
        self.assertEqual(c.state, 'ready')

    def test_changed_episode_fails_before_mutating_actor_slots(self):
        c = self.make()
        c.capture()
        self.client.get_world.return_value = SimpleNamespace(id=42)
        with self.assertRaisesRegex(RuntimeError, 'episode changed'):
            c.restore()
        self.assertEqual(self.mocks[3].call_count, 1)
        self.assertEqual(c.state, 'failed')

    def test_partial_failure_disables_further_restore(self):
        c = self.make()
        c.capture()
        self.mocks[5].side_effect = RuntimeError('TM restore rejected')
        with self.assertRaisesRegex(RuntimeError, 'TM restore rejected'):
            c.restore()
        with self.assertRaisesRegex(RuntimeError, 'no usable checkpoint'):
            c.restore()

    def test_weather_change_fails_before_mutating_actor_slots(self):
        c = self.make()
        c.capture()
        self.mocks[2].return_value = {'cloudiness': 1}
        with self.assertRaisesRegex(RuntimeError, 'weather changed'):
            c.restore()
        self.assertEqual(self.mocks[3].call_count, 1)


if __name__ == '__main__':
    unittest.main()
