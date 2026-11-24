"""Focused preflight checks for the experimental live prop integration."""
import base64
import hashlib
import unittest
from unittest import mock

import checkpoint_live_vehicles as live
import checkpoint_live_props as props
import checkpoint_live_traffic as traffic
import checkpoint_fixed_props as fixed


class LivePropPreflightTests(unittest.TestCase):
    def test_bad_prop_selection_does_not_touch_world_or_vehicle_slots(self):
        world = mock.Mock()
        bad = [({'cone': 2}, None, '/helper'),
               ({'cone': 2, 'other': 2}, {'cone': {}, 'other': {}}, '/helper'),
               ({'cone': 2}, {'cone': {'x': float('nan'), 'y': 0, 'z': 0,
                                      'pitch': 0, 'yaw': 0, 'roll': 0}}, '/helper'),
               ({'cone': 2}, {'cone': {'x': 0, 'y': 0, 'z': 0,
                                      'pitch': 0, 'yaw': 0, 'roll': 0}}, None)]
        for mapping, transforms, helper in bad:
            with self.subTest(mapping=mapping, transforms=transforms, helper=helper):
                with self.assertRaises(ValueError):
                    live.capture(world, {'ego': 1}, '/tmp/not-a-real-checkpoint-file',
                                 prop_mapping=mapping, prop_bootstrap_transforms=transforms, prop_rpc=helper)
        world.assert_not_called()
        self.assertEqual(world.mock_calls, [])

    def test_changed_prop_bytes_are_rejected(self):
        data = b'prop record'
        slot = {'data_base64': base64.b64encode(data).decode(), 'sha256': hashlib.sha256(data).hexdigest()}
        self.assertEqual(props.decode({'cone': slot}), {'cone': data})
        slot['data_base64'] = base64.b64encode(b'altered').decode()
        with self.assertRaises(ValueError):
            props.decode({'cone': slot})

    def test_changed_traffic_checkpoint_bytes_are_rejected(self):
        data = b'traffic record'
        slot = {'data_base64': base64.b64encode(data).decode(), 'sha256': hashlib.sha256(data).hexdigest()}
        self.assertEqual(traffic.decode(slot), data)
        slot['data_base64'] = base64.b64encode(b'altered').decode()
        with self.assertRaises(ValueError):
            traffic.decode(slot)

    def test_lifecycle_actions_validate_the_live_set(self):
        spec = {'id': 'ego', 'blueprint': 'vehicle.lincoln.mkz_2020', 'spawn_index': 0}
        later = {'id': 'later', 'blueprint': 'vehicle.lincoln.mkz_2020', 'spawn_index': 1}
        bundle = {'map': 'Town01', 'actors': {'ego': {}}, 'spawn_specs': [spec], 'dynamic_props': {'cone': {}}}
        actions = {'actors': [later], 'frames': [
            {'events': [{'action': 'spawn', 'label': 'later'}], 'controls': {'ego': {}, 'later': {}}},
            {'events': [{'action': 'destroy', 'label': 'later'}], 'controls': {'ego': {}}}]}
        frames, definitions = live.prepare_actions(bundle, actions)
        self.assertEqual(len(frames), 2)
        self.assertEqual(set(definitions), {'ego', 'later'})
        actions['frames'][1]['controls']['later'] = {}
        with self.assertRaises(ValueError):
            live.prepare_actions(bundle, actions)
        actions['frames'][1]['controls'].pop('later')
        actions['actors'][0]['id'] = 'cone'
        with self.assertRaises(ValueError):
            live.prepare_actions(bundle, actions)
        with self.assertRaises(ValueError):
            live.prepare_actions(bundle, [{'ego': {}}, {'ego': {'steer': 2}}])

    def test_fixed_guard_only_exempts_declared_dynamic_props(self):
        world = mock.Mock()
        world.get_map.return_value.name = 'Town01'
        guard = {'mode': 'fixed_map_props_v1', 'map': 'Town01', 'records': [{'id': 1, 'pose': 0}]}
        with mock.patch.object(fixed, 'observe', return_value=(7, [{'id': 1, 'pose': 0}, {'id': 2, 'pose': 9}])):
            self.assertEqual(fixed.validate_current(world, guard, [2]), guard['records'])
            with self.assertRaises(ValueError):
                fixed.validate_current(world, guard)
            with self.assertRaises(ValueError):
                fixed.validate_current(world, guard, [1, 2])


if __name__ == '__main__':
    unittest.main()
