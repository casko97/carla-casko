"""Pure-Python checks for checkpoint rollout comparison; no server required."""

import importlib.util
import json
from pathlib import Path
import sys
import tempfile
import types
import unittest
from unittest.mock import patch


example = Path(__file__).resolve().parents[2] / "examples" / "vehicle_physics_checkpoint.py"
spec = importlib.util.spec_from_file_location("vehicle_physics_checkpoint", example)
module = importlib.util.module_from_spec(spec)
with patch.dict(sys.modules, {"carla": types.ModuleType("carla")}):
    spec.loader.exec_module(module)


class CompareRolloutsTest(unittest.TestCase):
    def test_export_writes_the_server_bytes_with_build_id(self):
        seen = []
        vehicle = types.SimpleNamespace(export_physics_checkpoint=lambda build_id: seen.append(build_id) or b"checkpoint")
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "vehicle.msgpack"
            result = module.export_checkpoint(vehicle, "build-1", target)
            self.assertEqual(target.read_bytes(), b"checkpoint")
        self.assertEqual(seen, ["build-1"])
        self.assertEqual(result["bytes"], 10)

    def test_manifest_captures_actual_settings_and_vehicle_configuration(self):
        pos = types.SimpleNamespace(x=1.0, y=2.0, z=3.0)
        rot = types.SimpleNamespace(pitch=4.0, yaw=5.0, roll=6.0)
        transform = types.SimpleNamespace(location=pos, rotation=rot)
        map_obj = types.SimpleNamespace(name="Town03", get_spawn_points=lambda: [transform])
        settings = types.SimpleNamespace(synchronous_mode=True, fixed_delta_seconds=0.05,
                                         substepping=True, max_substep_delta_time=0.01,
                                         max_substeps=10, deterministic_ragdolls=True,
                                         no_rendering_mode=False)
        world = types.SimpleNamespace(get_settings=lambda: settings, get_map=lambda: map_obj,
                                      get_physx_scene_flags=lambda: 1 << 20)
        client = types.SimpleNamespace(get_client_version=lambda: "client", get_server_version=lambda: "server")
        vehicle = types.SimpleNamespace(type_id="vehicle.test", attributes={"color": "red"},
                                        get_physics_control=lambda: "physics")
        result = module.collect_manifest(client, world, vehicle, 0, "build-hash")
        self.assertEqual(result["world_settings"]["max_substeps"], 10)
        self.assertEqual(result["spawn_location"], [1.0, 2.0, 3.0])
        self.assertEqual(result["spawn_rotation"], [4.0, 5.0, 6.0])
        self.assertEqual(result["vehicle_attributes"], {"color": "red"})
        self.assertEqual(result["physics_control"], "physics")
        self.assertEqual(result["physx_scene_flags"], 1 << 20)
        self.assertTrue(result["physx_scene_flags_physx_3_4"]["enhanced_determinism"])
        self.assertFalse(result["physx_scene_flags_physx_3_4"]["gpu_dynamics"])
        self.assertEqual(result["build_id"], "build-hash")

    def test_maneuver_journal_includes_both_gears_steering_and_braking(self):
        journal = module.make_journal(2, 3, 8, "maneuver")
        self.assertEqual([len(journal[k]) for k in ("warmup", "prefix", "suffix")], [2, 3, 8])
        self.assertEqual({item["gear"] for item in journal["suffix"]}, {1, 2})
        self.assertTrue(any(item.get("steer", 0) > 0 for item in journal["suffix"]))
        self.assertTrue(any(item.get("steer", 0) < 0 for item in journal["suffix"]))
        self.assertTrue(any(item.get("brake", 0) > 0 for item in journal["suffix"]))
        self.assertTrue(all(item["manual_gear_shift"] for item in journal["suffix"]))
        with self.assertRaises(ValueError):
            module.make_journal(0, 0, 3, "maneuver")

    def test_journal_sends_each_control_in_order(self):
        journal = module.make_journal(0, 0, 8, "maneuver")["suffix"]
        sent = []
        fake_carla = types.SimpleNamespace(VehicleControl=lambda **kwargs: kwargs)
        with patch.object(module, "carla", fake_carla), patch.object(
                module, "step", side_effect=lambda client, world, actor, control: sent.append(control)):
            module.run_journal(None, None, None, journal)
        self.assertEqual(sent, journal)
        self.assertIsNot(sent[0], journal[0])

    def test_partial_trace_is_retained_when_a_step_fails(self):
        trace = []
        fake_carla = types.SimpleNamespace(VehicleControl=lambda **kwargs: kwargs)
        with patch.object(module, "carla", fake_carla), patch.object(
                module, "step", side_effect=[{"location": [1.0, 0.0, 0.0]}, RuntimeError("failed")]):
            with self.assertRaisesRegex(RuntimeError, "failed"):
                module.run_journal(None, None, None, [{"throttle": 1.0}, {"brake": 1.0}], trace)
        self.assertEqual(trace, [{"location": [1.0, 0.0, 0.0]}])

    def test_report_contains_failure_stage_and_journal(self):
        client = types.SimpleNamespace(set_timeout=lambda timeout: None,
                                       get_world=lambda: types.SimpleNamespace(get_settings=lambda: object()))
        fake_carla = types.SimpleNamespace(Client=lambda host, port: client)
        with tempfile.TemporaryDirectory() as directory:
            report_path = Path(directory) / "report.json"
            args = ["vehicle_physics_checkpoint.py", "--report", str(report_path)]
            with patch.object(module, "carla", fake_carla), patch.object(sys, "argv", args), patch.object(
                    module, "load_world", side_effect=RuntimeError("map failed")):
                with self.assertRaisesRegex(RuntimeError, "map failed"):
                    module.main()
            result = json.loads(report_path.read_text(encoding="utf-8"))
        self.assertEqual(result["failure"]["stage"], "load_world")
        self.assertEqual(result["failure"]["message"], "map failed")
        self.assertEqual(len(result["command_journal"]["suffix"]), 80)

    def test_step_rejects_a_failed_batch_and_an_unexpected_tick(self):
        command = types.SimpleNamespace(ApplyVehicleControl=lambda actor_id, control: (actor_id, control))
        fake_carla = types.SimpleNamespace(command=command)
        vehicle = types.SimpleNamespace(id=7)
        with patch.object(module, "carla", fake_carla):
            client = types.SimpleNamespace(apply_batch_sync=lambda batch, due_tick: [types.SimpleNamespace(error="failed")])
            with self.assertRaisesRegex(RuntimeError, "control command failed"):
                module.step(client, None, vehicle, object())
            client = types.SimpleNamespace(apply_batch_sync=lambda batch, due_tick: [types.SimpleNamespace(error="")])
            world = types.SimpleNamespace(get_snapshot=lambda: types.SimpleNamespace(frame=10), tick=lambda: 12)
            with self.assertRaisesRegex(RuntimeError, "unexpected world tick"):
                module.step(client, world, vehicle, object())

    def test_reports_first_difference_and_maximum_error(self):
        baseline = [{"location": [0.0, 1.0, 2.0]}, {"location": [2.0, 3.0, 4.0]}]
        actual = [{"location": [0.0, 1.25, 2.0]}, {"location": [2.0, 4.0, 4.0]}]
        result = module.compare(baseline, actual)
        self.assertFalse(result["exact"])
        self.assertEqual(result["first_difference"]["suffix_frame"], 1)
        self.assertEqual(result["first_difference"]["component"], 1)
        self.assertEqual(result["max_absolute_error"]["location"], 1.0)

    def test_identical_and_empty_rollouts(self):
        self.assertTrue(module.compare([], [])["exact"])
        samples = [{"velocity": [0.0, 1.0, -2.0]}]
        self.assertTrue(module.compare(samples, samples)["exact"])

    def test_rejects_missing_fields_dimensions_and_nonfinite_values(self):
        for baseline, actual in [
            ([{"v": [1.0]}], []),
            ([{"v": [1.0]}], [{"x": [1.0]}]),
            ([{"v": [1.0, 2.0]}], [{"v": [1.0]}]),
            ([{"v": [float("nan")]}], [{"v": [0.0]}]),
        ]:
            with self.subTest(baseline=baseline, actual=actual):
                with self.assertRaises(ValueError):
                    module.compare(baseline, actual)


if __name__ == "__main__":
    unittest.main()
