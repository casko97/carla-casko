"""Verify the live recorder's ownership and failure invariants."""
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import checkpoint_live_recorder as recorder
import checkpoint_live_vehicles as live


class FakeWorld:
    def __init__(self, actor):
        self.frame = 17
        self.actor = actor
        self.map_name = 'Town01'
        self.weather = {name: 0.0 for name in recorder.common.WEATHER_FIELDS}
    def get_snapshot(self):
        return SimpleNamespace(frame=self.frame, find=lambda actor_id: self.actor if actor_id == self.actor.id else None)
    def get_actors(self):
        return [self.actor]
    def get_map(self):
        return SimpleNamespace(name='/Game/Carla/Maps/' + self.map_name)
    def get_weather(self):
        return SimpleNamespace(**self.weather)


class RecorderTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.actor = SimpleNamespace(id=12, type_id='vehicle.lincoln.mkz_2020')
        self.world = FakeWorld(self.actor)
        self.checkpoint = Path(self.tmp.name) / 'checkpoint.json'
        self.bundle = {'schema': live.SCHEMA, 'identity': {'id': 'test'}, 'map': 'Town01', 'source_frame': 17,
                       'actors': {'ego': {'source_actor_id': 12}}, 'dynamic_props': {},
                       'boundary_observation': {'weather': dict(self.world.weather)},
                       'spawn_specs': [{'id': 'ego', 'blueprint': 'vehicle.lincoln.mkz_2020', 'spawn_index': 0}]}
        self.checkpoint.write_text(json.dumps(self.bundle))
        self.fp = mock.patch.object(recorder.common, 'fingerprint', return_value={'id': 'test'})
        self.settings = mock.patch.object(live, 'check_settings')
        self.fp.start(); self.settings.start()
        self.addCleanup(self.fp.stop); self.addCleanup(self.settings.stop)

    def make(self):
        return recorder.LiveActionRecorder(None, self.world, self.checkpoint, {'ego': self.actor})

    def test_records_executed_copy_and_writes_manifest_last(self):
        r = self.make()
        def execute(world, client, actors, action, specs, policy):
            world.frame += 1
            return {'ego': {'executed': action['controls']['ego']['throttle']}}
        action = {'events': [], 'controls': {'ego': {'throttle': 0.2}}}
        with mock.patch.object(recorder.journal, 'execute', side_effect=execute):
            r.step(action)
        action['controls']['ego']['throttle'] = 0.8
        output = Path(self.tmp.name) / 'recording'
        result = r.finish(output)
        saved = json.loads((output / 'actions.json').read_text())
        self.assertEqual(saved['frames'][0]['controls']['ego']['throttle'], 0.2)
        self.assertEqual(result['last_frame'], 18)
        self.assertTrue((output / 'manifest.json').exists())
        with self.assertRaises(RuntimeError):
            r.step(action)

    def test_selected_map_and_light_policy_are_used(self):
        policy = {'mode': 'fixed-plus-control-v1', 'fixed_mask': 3}
        self.bundle['map'] = 'Town02'
        self.world.map_name = 'Town02'
        self.bundle['vehicle_light_policy'] = policy
        self.checkpoint.write_text(json.dumps(self.bundle))
        with mock.patch.object(recorder.common, 'fingerprint', return_value={'id': 'test'}) as fingerprint:
            r = self.make()
            def execute(world, client, actors, action, specs, passed_policy):
                self.assertEqual(passed_policy, policy)
                world.frame += 1
                return {}
            with mock.patch.object(recorder.journal, 'execute', side_effect=execute), \
                 mock.patch.object(recorder.multi, 'sample_vehicle_lights', return_value={'ego': 3}):
                r.step({'events': [], 'controls': {'ego': {'throttle': 0.2}}})
            r.finish(Path(self.tmp.name) / 'town02')
            self.assertEqual(json.loads((Path(self.tmp.name) / 'town02' / 'vehicle_light_reference.json').read_text()), [{'ego': 3}])
            self.assertEqual([c.args for c in fingerprint.call_args_list], [('Town02',), ('Town02',)])

    def test_rejects_different_live_map(self):
        self.world.map_name = 'Town02'
        with self.assertRaisesRegex(ValueError, 'world map'):
            self.make()

    def test_rejects_walkers_in_checkpoint_or_world(self):
        self.bundle['walkers'] = {'ped': {}}
        self.checkpoint.write_text(json.dumps(self.bundle))
        with self.assertRaisesRegex(ValueError, 'walker controls'):
            self.make()
        self.bundle.pop('walkers')
        self.checkpoint.write_text(json.dumps(self.bundle))
        self.world.get_actors = lambda: [self.actor, SimpleNamespace(id=13, type_id='walker.pedestrian.0001')]
        with self.assertRaisesRegex(RuntimeError, 'walkers'):
            self.make()

    def test_weather_change_prevents_step_and_publication(self):
        r = self.make()
        self.world.weather['cloudiness'] = 20.0
        action = {'events': [], 'controls': {'ego': {'throttle': 0.2}}}
        with mock.patch.object(recorder.journal, 'execute') as execute:
            with self.assertRaisesRegex(RuntimeError, 'weather changed'):
                r.step(action)
            execute.assert_not_called()
        with self.assertRaises(RuntimeError):
            r.finish(Path(self.tmp.name) / 'changed-weather')

    def test_weather_change_during_step_poison_recording(self):
        r = self.make()
        def execute(world, client, actors, action, specs, policy):
            world.frame += 1
            world.weather['precipitation'] = 20.0
            return {}
        with mock.patch.object(recorder.journal, 'execute', side_effect=execute):
            with self.assertRaisesRegex(RuntimeError, 'weather changed'):
                r.step({'events': [], 'controls': {'ego': {'throttle': 0.2}}})
        self.world.weather['precipitation'] = 0.0
        with self.assertRaisesRegex(RuntimeError, 'failed'):
            r.finish(Path(self.tmp.name) / 'failed-weather')

    def test_external_tick_and_failed_execution_cannot_be_published(self):
        action = {'events': [], 'controls': {'ego': {'throttle': 0.2}}}
        r = self.make()
        self.world.frame += 1
        with self.assertRaisesRegex(RuntimeError, 'outside the recorder'):
            r.step(action)
        with self.assertRaises(RuntimeError):
            r.finish(Path(self.tmp.name) / 'external')
        self.world.frame -= 1
        with mock.patch.object(recorder.journal, 'execute', side_effect=RuntimeError('partial failure')):
            with self.assertRaisesRegex(RuntimeError, 'partial failure'):
                r.step(action)
        with self.assertRaises(RuntimeError):
            r.finish(Path(self.tmp.name) / 'failed')


if __name__ == '__main__':
    unittest.main()
