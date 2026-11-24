"""Map identity validation for live checkpoint compatibility."""
import json
import tempfile
import unittest
from pathlib import Path
from unittest import mock
import checkpoint_scenario_bundle as common
import checkpoint_journal_bundle as journal
import checkpoint_live_vehicles as live


class MapValidationTests(unittest.TestCase):
    def test_known_maps_have_expected_distinct_local_artifacts(self):
        first = common.map_artifact_paths('Town01')
        second = common.map_artifact_paths('Town02')
        self.assertEqual(set(first), {'map', 'map_built_data', 'opendrive', 'traffic_map', 'nav'})
        self.assertNotEqual(first['map'], second['map'])
        self.assertTrue(all(path.is_file() for path in second.values()))

    def test_rejects_paths_missing_maps_and_non_strings(self):
        for name in ('../Town01', '/Game/Carla/Maps/Town01', 'TownMissing', '', None):
            with self.subTest(name=name), self.assertRaises(ValueError):
                common.map_artifact_paths(name)

    def test_mismatched_map_identity_rejected_before_client_use(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / 'checkpoint.json'
            path.write_text(json.dumps({'schema': live.SCHEMA, 'map': 'Town02', 'identity': {'id': 'Town01'}}))
            client = mock.Mock()
            with mock.patch.object(common, 'fingerprint', return_value={'id': 'Town02'}):
                with self.assertRaisesRegex(ValueError, 'artifacts differ'):
                    live.replay(client, path, [], Path(temp) / 'report.json')
            self.assertEqual(client.mock_calls, [])
            self.assertFalse((Path(temp) / 'report.json').exists())

    def test_lifecycle_journal_accepts_another_flat_map(self):
        scenario = {'map': 'Town02', 'actors': [{'id': 'ego', 'blueprint': 'vehicle.lincoln.mkz_2020', 'spawn_index': 0}],
                    'initial_actors': ['ego'], 'frames': [{'events': [], 'controls': {'ego': {}}},
                                                        {'events': [], 'controls': {'ego': {}}}]}
        definitions, alive = journal.validate_journal(scenario, 1)
        self.assertEqual(alive, {'ego'})
        self.assertEqual(set(definitions), {'ego'})


if __name__ == '__main__':
    unittest.main()
