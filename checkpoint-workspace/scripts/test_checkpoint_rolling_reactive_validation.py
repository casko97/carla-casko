"""Comparison reports for the rolling checkpoint controlled experiment."""
import copy
import unittest

from checkpoint_rolling_reactive_validation import compare, reaction


def frame(expert_x=0.0, traffic_x=10.0, throttle=0.0):
    return {'states': {'expert': {'location': [expert_x, 0, 0], 'velocity': [0, 0, 0]},
                       'traffic': {'location': [traffic_x, 0, 0], 'velocity': [0, 0, 0]}},
            'ego_control': {'throttle': .4},
            'traffic_control': {'throttle': throttle, 'brake': 0., 'steer': 0.},
            'traffic_lights': {'light': {'state': 'Green'}}, 'center_distance_m': traffic_x - expert_x}


class ControlledValidationTests(unittest.TestCase):
    def test_compares_every_frame_and_reports_first_difference(self):
        reference = [frame(), frame(), frame()]
        candidate = copy.deepcopy(reference)
        candidate[1]['states']['expert']['location'][0] = .1
        result = compare(reference, candidate)
        self.assertFalse(result['exact_sampled_match'])
        self.assertEqual(result['first_difference']['tick'], 2)
        self.assertEqual(result['max_position_error_m']['expert'], .1)

    def test_detects_non_position_difference(self):
        a, b = [frame()], [frame()]
        b[0]['traffic_lights']['light']['state'] = 'Red'
        result = compare(a, b)
        self.assertFalse(result['exact_sampled_match'])
        self.assertIn('traffic_lights', result['first_difference']['field'])
        self.assertEqual(result['max_position_error_m']['traffic'], 0.)

    def test_rejects_missing_frames_and_tracks_traffic_reaction(self):
        with self.assertRaises(ValueError):
            compare([frame()], [])
        changed = frame(traffic_x=12., throttle=.5)
        result = reaction([frame()], [changed])
        self.assertEqual(result['max_traffic_position_difference_m'], 2.)
        self.assertEqual(result['max_traffic_control_difference'], .5)


if __name__ == '__main__':
    unittest.main()
