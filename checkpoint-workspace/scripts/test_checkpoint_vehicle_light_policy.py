"""Check the deterministic vehicle-light rule and policy validation."""
import unittest
from unittest import mock

import checkpoint_vehicle_light_policy as lights


class VehicleLightPolicyTests(unittest.TestCase):
    def test_control_derived_lights_and_fixed_mask(self):
        policy = {'mode': lights.POLICY, 'fixed_mask': 3}
        self.assertEqual(lights.state(policy, {}), 3)
        self.assertEqual(lights.state(policy, {'brake': 0.5}), 3)
        self.assertEqual(lights.state(policy, {'brake': 0.6}), 3 | lights.BRAKE)
        self.assertEqual(lights.state(policy, {'reverse': True}), 3 | lights.REVERSE)
        self.assertEqual(lights.state(policy, {'brake': 1.0, 'reverse': True}), 3 | lights.BRAKE | lights.REVERSE)

    def test_rejects_unsupported_and_dynamic_fixed_bits(self):
        for mask in (lights.BRAKE, lights.REVERSE, 0x800, -1, True):
            with self.subTest(mask=mask), self.assertRaises(ValueError):
                lights.validate({'mode': lights.POLICY, 'fixed_mask': mask})
        with self.assertRaises(ValueError):
            lights.validate({'mode': 'other', 'fixed_mask': 0})

    def test_capture_rejects_lights_inconsistent_with_last_control(self):
        world, actor = mock.Mock(), mock.Mock()
        world.get_snapshot.return_value.frame = 7
        actor.get_control.return_value.brake = 1.0
        actor.get_control.return_value.reverse = False
        actor.get_light_state.return_value = 3
        with self.assertRaisesRegex(ValueError, 'does not match'):
            lights.check_current(world, {'ego': actor}, {'mode': lights.POLICY, 'fixed_mask': 3})
        actor.get_light_state.return_value = 3 | lights.BRAKE
        lights.check_current(world, {'ego': actor}, {'mode': lights.POLICY, 'fixed_mask': 3})

    def test_rejects_mismatched_actor_controls_before_batch(self):
        client, world = mock.Mock(), mock.Mock()
        with self.assertRaises(ValueError):
            lights.apply(client, world, {'ego': object()}, {}, {'mode': lights.POLICY, 'fixed_mask': 0})
        self.assertEqual(client.mock_calls, [])
        self.assertEqual(world.mock_calls, [])


if __name__ == '__main__':
    unittest.main()
