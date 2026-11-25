"""Preflight checks for the restricted reactive prefix recipe."""
import copy
import unittest
from unittest import mock

import checkpoint_tm_prefix_reconstruction as prefix


class PrefixRecipeTests(unittest.TestCase):
    def setUp(self):
        self.recipe = {'schema': prefix.SCHEMA, 'map': 'Town01', 'identity': {'id': 'test'}, 'tm_seed': 17,
                       'initial_weather': {name: 0.0 for name in prefix.common.WEATHER_FIELDS},
                       'actors': [{'id': 'expert', 'blueprint': 'vehicle.lincoln.mkz_2020', 'spawn_index': 1},
                                  {'id': 'traffic', 'blueprint': 'vehicle.lincoln.mkz_2020', 'spawn_index': 0,
                                   'controller': 'traffic_manager'}],
                       'fixed_props': [], 'prefix': [{'brake': 1.0} for _ in range(60)], 'boundary': {},
                       'source_branches': {'hold': [{} for _ in range(120)], 'move': [{} for _ in range(120)]}}
        patch = mock.patch.object(prefix.common, 'fingerprint', return_value={'id': 'test'})
        patch.start()
        self.addCleanup(patch.stop)

    def test_accepts_restricted_recipe(self):
        self.assertIs(prefix.validate_recipe(self.recipe), self.recipe)

    def test_rejects_invalid_seed_prefix_or_actor_roles(self):
        for change in ('seed', 'prefix', 'role'):
            recipe = copy.deepcopy(self.recipe)
            if change == 'seed':
                recipe['tm_seed'] = -1
            elif change == 'prefix':
                recipe['prefix'][0] = {'throttle': 1.1}
            else:
                recipe['actors'][1]['controller'] = 'manual'
            with self.subTest(change=change), self.assertRaises(ValueError):
                prefix.validate_recipe(recipe)

    def test_multiple_traffic_vehicles_require_explicit_complete_configuration(self):
        recipe = copy.deepcopy(self.recipe)
        recipe['actors'].append({'id': 'traffic2', 'blueprint': 'vehicle.lincoln.mkz_2020',
                                 'spawn_index': 2, 'controller': 'traffic_manager'})
        with self.assertRaises(ValueError):
            prefix.validate_recipe(recipe)
        recipe['tm_config'] = {
            'traffic': {'auto_lane_change': False, 'distance_to_leading_vehicle': 6.0},
            'traffic2': {'auto_lane_change': False, 'distance_to_leading_vehicle': 5.0,
                         'speed_difference': 10.0}}
        self.assertIs(prefix.validate_recipe(recipe), recipe)
        recipe['tm_config']['traffic2']['distance_to_leading_vehicle'] = float('nan')
        with self.assertRaises(ValueError):
            prefix.validate_recipe(recipe)


if __name__ == '__main__':
    unittest.main()
