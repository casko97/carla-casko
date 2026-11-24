"""Validate manual walker controls and checkpoint data before world mutation."""
import base64
import hashlib
import unittest
from types import SimpleNamespace
from unittest import mock

import checkpoint_live_walkers as walkers


class WalkerValidationTests(unittest.TestCase):
    def test_navigation_state_commands_are_validated_and_applied(self):
        transform = {'x': 1, 'y': 2, 'z': 3, 'pitch': 0, 'yaw': 90, 'roll': 0}
        state = {'transform': transform, 'speed': 1.4}
        walkers.validate_actions([{'ped': state}], {'ped'}, 1)
        actor = SimpleNamespace(id=7)
        world = mock.Mock()
        world.get_snapshot.return_value = SimpleNamespace(frame=10)
        client = mock.Mock()
        client.apply_batch_sync.return_value = [SimpleNamespace(error=None)]
        with mock.patch.object(walkers.carla.command, 'ApplyWalkerState', return_value='state') as command:
            walkers.apply_controls(client, world, {'ped': actor}, {'ped': state})
        self.assertEqual(command.call_args.args[0], 7)
        self.assertEqual(command.call_args.args[2], 1.4)
        self.assertEqual(command.call_args.args[1].location.x, 1)
        client.apply_batch_sync.assert_called_once_with(['state'], False)
        for bad in ({'transform': dict(transform, x=float('nan')), 'speed': 1},
                    {'transform': transform, 'speed': -1},
                    {'transform': transform, 'speed': 1, 'jump': False}):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                walkers.validate_actions([{'ped': bad}], {'ped'}, 1)

    def test_lifecycle_preflight_tracks_alive_set(self):
        control = {'direction': [1, 0, 0], 'speed': 1.4, 'jump': False}
        slots = {'ped': {}}
        frames = [{'events': [{'action': 'destroy', 'label': 'ped'}], 'controls': {}},
                  {'events': [{'action': 'spawn', 'label': 'ped'}], 'controls': {'ped': control}}]
        prepared, _ = walkers.prepare_actions({'actors': {}, 'frames': frames}, slots, 2)
        self.assertEqual(prepared, frames)
        for bad in ([{'events': [{'action': 'spawn', 'label': 'ped'}], 'controls': {'ped': control}}],
                    [{'events': [{'action': 'destroy', 'label': 'ped'}], 'controls': {'ped': control}}],
                    [{'events': [{'action': 'destroy', 'label': 'unknown'}], 'controls': {}}]):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                walkers.prepare_actions({'actors': {}, 'frames': bad}, slots, 1)

    def test_new_walker_definition_cannot_overlap_other_actors(self):
        t = {'x': 0, 'y': 1, 'z': 2, 'pitch': 0, 'yaw': 0, 'roll': 0}
        spec = {'blueprint': 'walker.pedestrian.0001', 'attributes': {}, 'spawn_attributes': {}, 'spawn_transform': t}
        control = {'direction': [1, 0, 0], 'speed': 1.4, 'jump': False}
        journal = {'actors': {'new': spec}, 'frames': [{'events': [{'action': 'spawn', 'label': 'new'}], 'controls': {'new': control}}]}
        walkers.prepare_actions(journal, {}, 1)
        with self.assertRaises(ValueError):
            walkers.prepare_actions(journal, {}, 1, {'new'})

    def test_validates_frame_count_actor_set_and_controls(self):
        good = [{'ped': {'direction': [1.0, 0.0, 0.0], 'speed': 1.4, 'jump': False}}]
        self.assertEqual(walkers.validate_actions(good, {'ped'}, 1), good)
        for bad in ([], [{}], [{'ped': {'direction': [2, 0, 0], 'speed': 1, 'jump': False}}],
                    [{'ped': {'direction': [1, 0, 0], 'speed': float('nan'), 'jump': False}}],
                    [{'ped': {'direction': [1, 0, 0], 'speed': 1, 'jump': 1}}]):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                walkers.validate_actions(bad, {'ped'}, 1)

    def test_rejects_tampered_velocity(self):
        data = b'walker velocity'
        slot = {'data_base64': base64.b64encode(data).decode(), 'sha256': hashlib.sha256(data).hexdigest()}
        self.assertEqual(walkers.decode({'ped': slot}), {'ped': data})
        slot['data_base64'] = base64.b64encode(b'tampered').decode()
        with self.assertRaises(ValueError):
            walkers.decode({'ped': slot})

    def test_reference_validation_rejects_invalid_vectors(self):
        sample = {'ped': {'location': [1, 2, 3], 'velocity': [0, 0, 1]}}
        self.assertEqual(walkers.validate_reference([sample], {'ped'}, 1), [sample])
        with self.assertRaises(ValueError):
            walkers.validate_reference([{'ped': {'location': [1, 2], 'velocity': [0, 0, 1]}}], {'ped'}, 1)
        with self.assertRaises(ValueError):
            walkers.validate_reference([{'ped': {'location': [1, 2, 3], 'velocity': [0, 0, float('inf')]}}], {'ped'}, 1)

    def test_spawn_transform_and_id_validation(self):
        t = {'x': 0, 'y': 1, 'z': 2, 'pitch': 0, 'yaw': 0, 'roll': 0}
        walkers.validate_selection({'ped': 8}, {'ped': t}, '/helper')
        with self.assertRaises(ValueError):
            walkers.validate_selection({'ped': 8}, {'ped': dict(t, x=float('nan'))}, '/helper')
        with self.assertRaises(ValueError):
            walkers.validate_selection({'ped': 8, 'p2': 8}, {'ped': t, 'p2': t}, '/helper')


if __name__ == '__main__':
    unittest.main()
