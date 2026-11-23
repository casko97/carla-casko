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
    def test_vehicle_light_traces_detect_changes(self):
        values = {"a": 1, "b": 2}
        actors = {label: types.SimpleNamespace(get_light_state=lambda key=label: values[key]) for label in values}
        world = types.SimpleNamespace(get_snapshot=lambda: types.SimpleNamespace(frame=7))
        expected = module.sample_vehicle_lights(world, actors)
        values["b"] = 4
        actual = module.sample_vehicle_lights(world, actors)
        self.assertEqual(expected, {"a": 1, "b": 2})
        self.assertFalse(module.compare_component_traces([expected], [actual])["exact"])
        frames = iter([7, 8])
        world = types.SimpleNamespace(get_snapshot=lambda: types.SimpleNamespace(frame=next(frames)))
        with self.assertRaisesRegex(RuntimeError, "world advanced"):
            module.sample_vehicle_lights(world, actors)

    def test_tm_actor_controls_are_not_sent_as_manual_commands(self):
        scenario = module.demo_scenario()
        scenario["actors"][1]["controller"] = "traffic_manager"
        for frame in scenario["frames"]:
            del frame["second"]
        self.assertEqual(module.validate_scenario(scenario, 40), ["first", "second"])
        invalid = copy.deepcopy(scenario)
        invalid["frames"][0]["second"] = {"throttle": 1.0}
        with self.assertRaises(ValueError):
            module.validate_scenario(invalid, 40)
        calls = []
        fake_carla = types.SimpleNamespace(command=types.SimpleNamespace(
            ApplyVehicleControl=lambda actor_id, control: (actor_id, control)), VehicleControl=lambda **kw: kw)
        client = types.SimpleNamespace(apply_batch_sync=lambda batch, due_tick: calls.append(batch) or
                                       [types.SimpleNamespace(error="")])
        actors = {"first": types.SimpleNamespace(id=11), "second": types.SimpleNamespace(id=22)}
        world = types.SimpleNamespace(get_snapshot=lambda: types.SimpleNamespace(frame=5), tick=lambda: 6)
        with patch.object(module, "carla", fake_carla), patch.object(single, "sample", side_effect=lambda w, a: {"id": a.id}):
            result = module.step(client, world, actors, {"first": {"throttle": 0.5}})
        self.assertEqual(calls, [[(11, {"throttle": 0.5})]])
        self.assertEqual(result, {"first": {"id": 11}, "second": {"id": 22}})
        with patch.object(module, "carla", fake_carla), patch.object(single, "sample", side_effect=lambda w, a: {"id": a.id}):
            module.step(client, world, actors, {})
        self.assertEqual(len(calls), 1)

    def test_tm_checkpoint_retries_without_advancing_world(self):
        attempts = iter([False, True])
        tm = types.SimpleNamespace(save_checkpoint=lambda: next(attempts))
        world = types.SimpleNamespace(get_snapshot=lambda: types.SimpleNamespace(frame=7))
        with patch.object(module.time, "sleep") as sleep:
            self.assertEqual(module.checkpoint_traffic_manager(world, tm), 7)
        sleep.assert_called_once()
        frames = iter([7, 8])
        world = types.SimpleNamespace(get_snapshot=lambda: types.SimpleNamespace(frame=next(frames)))
        tm = types.SimpleNamespace(restore_checkpoint=lambda: True)
        with self.assertRaisesRegex(RuntimeError, "world advanced"):
            module.checkpoint_traffic_manager(world, tm, restore=True)

    def test_weather_changes_are_validated_and_replayed_at_absolute_frame(self):
        scenario = module.demo_scenario()
        parameters = {field: float(index) for index, field in enumerate(module.WEATHER_FIELDS)}
        scenario["weather_changes"] = [{"frame": 40, "parameters": parameters}]
        module.validate_scenario(scenario, 40)
        for mutation in ("duplicate", "out_of_range", "missing", "nonfinite"):
            altered = copy.deepcopy(scenario)
            if mutation == "duplicate":
                altered["weather_changes"].append(copy.deepcopy(altered["weather_changes"][0]))
            elif mutation == "out_of_range":
                altered["weather_changes"][0]["frame"] = len(altered["frames"])
            elif mutation == "missing":
                del altered["weather_changes"][0]["parameters"]["wetness"]
            else:
                altered["weather_changes"][0]["parameters"]["wetness"] = float("nan")
            with self.subTest(mutation=mutation), self.assertRaises(ValueError):
                module.validate_scenario(altered, 40)
        calls = []
        world = types.SimpleNamespace(get_snapshot=lambda: types.SimpleNamespace(frame=7),
                                      set_weather_synchronously=lambda weather: calls.append(("weather", weather)))
        fake_carla = types.SimpleNamespace(WeatherParameters=lambda **kw: kw)
        trace = []
        with patch.object(module, "carla", fake_carla), patch.object(module, "step", side_effect=lambda *args: calls.append(("step", args[-1])) or {}):
            module.run_frames(None, world, {}, [{"a": 1}, {"a": 2}], trace,
                              weather_changes={40: parameters}, start_frame=40)
        self.assertEqual(calls, [("weather", parameters), ("step", {"a": 1}), ("step", {"a": 2})])
        self.assertEqual(len(trace), 2)

    def test_explicit_spawn_transform_and_attributes(self):
        scenario = module.demo_scenario()
        scenario["actors"][0] = {"id": "first", "blueprint": "vehicle.example",
            "transform": {"x": 1, "y": 2, "z": 3, "pitch": 4, "yaw": 5, "roll": 6},
            "attributes": {"color": "red", "sticky_control": "false"}}
        module.validate_scenario(scenario, 40)
        attributes = {}
        blueprint = types.SimpleNamespace(has_attribute=lambda name: name in ("color", "sticky_control"),
                                           set_attribute=lambda name, value: attributes.update({name: value}))
        spawned = []
        actor = types.SimpleNamespace(destroy=lambda: None)
        world = types.SimpleNamespace(get_blueprint_library=lambda: types.SimpleNamespace(find=lambda _: blueprint),
                                      try_spawn_actor=lambda bp, transform: spawned.append((bp, transform)) or actor)
        fake_carla = types.SimpleNamespace(Location=lambda **kw: kw, Rotation=lambda **kw: kw,
                                           Transform=lambda loc, rot: (loc, rot))
        with patch.object(module, "carla", fake_carla):
            result = module.spawn_actors(world, scenario["actors"][:1])
        self.assertIs(result["first"], actor)
        self.assertEqual(spawned[0][1], ({"x": 1, "y": 2, "z": 3}, {"pitch": 4, "yaw": 5, "roll": 6}))
        self.assertEqual(attributes, {"color": "red", "sticky_control": "false"})
        for change in ("both", "nonfinite", "sticky", "unknown"):
            altered = copy.deepcopy(scenario)
            spec = altered["actors"][0]
            if change == "both":
                spec["spawn_index"] = 0
            elif change == "nonfinite":
                spec["transform"]["x"] = float("nan")
            elif change == "sticky":
                spec["attributes"]["sticky_control"] = "true"
            else:
                spec["attributes"]["color"] = 1
            with self.subTest(change=change), self.assertRaises(ValueError):
                module.validate_scenario(altered, 40)

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
        result = module.compare_component_traces([baseline], [changed])
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

    def test_weather_checkpoint_and_complete_parameter_sample(self):
        weather = types.SimpleNamespace(**{field: float(index) for index, field in enumerate(module.WEATHER_FIELDS)})
        world = types.SimpleNamespace(get_snapshot=lambda: types.SimpleNamespace(frame=8),
                                      get_weather=lambda: weather, save_weather_checkpoint=lambda: True,
                                      restore_weather_checkpoint=lambda: False)
        self.assertEqual(module.checkpoint_weather(world), 8)
        self.assertEqual(module.sample_weather(world)["dust_storm"], float(len(module.WEATHER_FIELDS) - 1))
        with self.assertRaisesRegex(RuntimeError, "weather restore rejected"):
            module.checkpoint_weather(world, restore=True)

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
