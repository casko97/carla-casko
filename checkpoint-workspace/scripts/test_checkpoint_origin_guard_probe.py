"""Check the origin probe's control flow and origin-response validation."""
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest import mock

import checkpoint_origin_guard_probe as probe


class OriginProbeTests(unittest.TestCase):
    def test_records_only_expected_shifted_origin_rejection(self):
        class Vehicle:
            shifted = False
            saved_shifted = False
            def save_physics_checkpoint(self):
                self.saved_shifted = self.shifted
                return True
            def restore_physics_checkpoint(self):
                return self.saved_shifted == self.shifted
            def export_physics_checkpoint(self, _):
                if self.saved_shifted:
                    raise RuntimeError('vehicle has no exportable physics checkpoint')
                return b'checkpoint'
            def set_transform(self, _):
                self.shifted = True
            def destroy(self):
                return True

        vehicle = Vehicle()
        blueprint = mock.Mock()
        world = mock.Mock()
        world.get_settings.return_value = object()
        world.get_blueprint_library.return_value.find.return_value = blueprint
        world.try_spawn_actor.return_value = vehicle
        client = mock.Mock()
        client.get_world.return_value = world
        with tempfile.TemporaryDirectory() as directory:
            report = Path(directory) / 'report.json'
            arguments = ['probe', '--map', 'Town01', '--origin-rpc', '/helper', '--report', str(report)]
            with mock.patch.object(probe.carla, 'Client', return_value=client), \
                 mock.patch.object(probe.single, 'load_world', return_value=world), \
                 mock.patch.object(probe, 'origin', side_effect=lambda *_: [100, 0, 0] if vehicle.shifted else [0, 0, 0]), \
                 mock.patch('sys.argv', arguments):
                self.assertEqual(probe.main(), 0)
            result = json.loads(report.read_text())
        self.assertEqual(result['initial_origin_cm'], [0, 0, 0])
        self.assertEqual(result['shifted_origin_cm'], [100, 0, 0])
        self.assertTrue(result['zero_origin_restore_accepted'])
        self.assertFalse(result['old_slot_restore_accepted'])
        self.assertTrue(result['shifted_export_rejected'])
        self.assertIsNone(result['failure'])

    def test_origin_query_rejects_invalid_output(self):
        with mock.patch.object(probe.subprocess, 'run', return_value=SimpleNamespace(returncode=0, stdout='[0, "0", 0]')):
            with self.assertRaisesRegex(RuntimeError, 'invalid origin'):
                probe.origin('/helper', 36000)


if __name__ == '__main__':
    unittest.main()
