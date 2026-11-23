"""Unit tests for multi-vehicle journal handling; no CARLA server required."""

import copy
import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch


examples = Path(__file__).resolve().parents[2] / "examples"
single_spec = importlib.util.spec_from_file_location("vehicle_physics_checkpoint", examples / "vehicle_physics_checkpoint.py")
single = importlib.util.module_from_spec(single_spec)
spec = importlib.util.spec_from_file_location("multi_vehicle_physics_checkpoint", examples / "multi_vehicle_physics_checkpoint.py")
module = importlib.util.module_from_spec(spec)
with patch.dict(sys.modules, {"carla": types.ModuleType("carla"), "vehicle_physics_checkpoint": single}):
    single_spec.loader.exec_module(single)
    spec.loader.exec_module(module)


class MultiVehicleJournalTest(unittest.TestCase):
    def test_traffic_lights_use_unique_ids_and_detect_first_difference(self):
        def light(identifier, state):
            return types.SimpleNamespace(get_opendrive_id=lambda: identifier, get_state=lambda: state,
                get_elapsed_time=lambda: 1.0, get_green_time=lambda: 10.0, get_yellow_time=lambda: 3.0,
                get_red_time=lambda: 2.0, is_frozen=lambda: False)
        lights = [light("A", "Green"), light("B", "Red")]
        world = types.SimpleNamespace(get_actors=lambda: types.SimpleNamespace(filter=lambda pattern: lights))
        selected = module.traffic_lights(world)
        baseline = module.sample_traffic_lights(selected)
        changed = copy.deepcopy(baseline)
        changed["A"]["elapsed"] = 2.0
        result = module.compare_traffic_lights([baseline], [changed])
        self.assertEqual(result["first_difference"]["relative_frame"], 1)
        self.assertFalse(result["exact"])
        lights.append(light("A", "Yellow"))
        with self.assertRaisesRegex(RuntimeError, "unique"):
            module.traffic_lights(world)

    def test_traffic_light_checkpoint_rejects_failed_or_extra_tick(self):
        world = types.SimpleNamespace(get_snapshot=lambda: types.SimpleNamespace(frame=8),
                                      save_traffic_lights_checkpoint=lambda: False)
        with self.assertRaisesRegex(RuntimeError, "checkpoint rejected"):
            module.checkpoint_traffic_lights(world)
        frames = iter([8, 9])
        world = types.SimpleNamespace(get_snapshot=lambda: types.SimpleNamespace(frame=next(frames)),
                                      restore_traffic_lights_checkpoint=lambda: True)
        with self.assertRaisesRegex(RuntimeError, "world advanced"):
            module.checkpoint_traffic_lights(world, restore=True)

    def test_scenario_rejects_missing_actor_commands_and_invalid_control(self):
        scenario = module.demo_scenario()
        self.assertEqual(module.validate_scenario(scenario, 40), ["first", "second"])
        for mutation in ("missing", "duplicate", "nonfinite", "out_of_range", "extra"):
            with self.subTest(mutation=mutation):
                altered = copy.deepcopy(scenario)
                if mutation == "missing":
                    del altered["frames"][0]["first"]
                elif mutation == "duplicate":
                    altered["actors"][1]["id"] = altered["actors"][0]["id"]
                elif mutation == "nonfinite":
                    altered["frames"][0]["first"]["steer"] = float("nan")
                elif mutation == "out_of_range":
                    altered["frames"][0]["first"]["brake"] = -1.0
                else:
                    altered["frames"][0]["first"]["unexpected"] = 3
                with self.assertRaises(ValueError):
                    module.validate_scenario(altered, 40)
        for checkpoint in (0, len(scenario["frames"])):
            with self.assertRaises(ValueError):
                module.validate_scenario(scenario, checkpoint)

    def test_control_batch_keeps_actor_order_and_ticks_once(self):
        command = types.SimpleNamespace(ApplyVehicleControl=lambda actor_id, control: (actor_id, control))
        fake_carla = types.SimpleNamespace(command=command, VehicleControl=lambda **values: values)
        actors = {"first": types.SimpleNamespace(id=11), "second": types.SimpleNamespace(id=22)}
        calls = []
        client = types.SimpleNamespace(apply_batch_sync=lambda batch, due_tick: (
            calls.append((batch, due_tick)) or [types.SimpleNamespace(error=""), types.SimpleNamespace(error="")]))
        world = types.SimpleNamespace(get_snapshot=lambda: types.SimpleNamespace(frame=5), tick=lambda: 6)
        with patch.object(module, "carla", fake_carla), patch.object(single, "sample", side_effect=lambda w, a: {"id": a.id}):
            result = module.step(client, world, actors, {"first": {"throttle": 0.5}, "second": {"brake": 1.0}})
        self.assertEqual(calls, [([(11, {"throttle": 0.5}), (22, {"brake": 1.0})], False)])
        self.assertEqual(result, {"first": {"id": 11}, "second": {"id": 22}})

    def test_scene_comparison_reports_actor_specific_divergence(self):
        ref = [{"a": {"location": [0.0]}, "b": {"location": [1.0]}}]
        actual = [{"a": {"location": [0.0]}, "b": {"location": [2.0]}}]
        result = module.compare_scene(ref, actual, ["a", "b"])
        self.assertFalse(result["exact"])
        self.assertTrue(result["actors"]["a"]["exact"])
        self.assertEqual(result["actors"]["b"]["first_difference"]["suffix_frame"], 1)

    def test_checkpoint_rejects_failures_and_extra_ticks(self):
        frames = iter([10, 10, 11])
        world = types.SimpleNamespace(get_snapshot=lambda: types.SimpleNamespace(frame=next(frames)))
        actor = types.SimpleNamespace(save_physics_checkpoint=lambda: True)
        with self.assertRaisesRegex(RuntimeError, "world advanced"):
            module.checkpoint_actors(world, {"a": actor, "b": actor})
        world = types.SimpleNamespace(get_snapshot=lambda: types.SimpleNamespace(frame=10))
        actor = types.SimpleNamespace(restore_physics_checkpoint=lambda: False)
        with self.assertRaisesRegex(RuntimeError, "restore checkpoint rejected"):
            module.checkpoint_actors(world, {"a": actor}, restore=True)


if __name__ == "__main__":
    unittest.main()
