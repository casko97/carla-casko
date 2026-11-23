#!/usr/bin/env python3
"""Replay a frame-indexed, multi-vehicle journal around an in-memory checkpoint.

Use only on a dedicated server built from this branch. This is a controlled
physics test with optional Traffic Manager state, not a ScenarioRunner checkpoint.
"""

import argparse
import json
import math
import sys
import time

import carla
import vehicle_physics_checkpoint as single


CONTROL_FIELDS = {"throttle", "steer", "brake", "hand_brake", "reverse", "manual_gear_shift", "gear"}
WEATHER_FIELDS = ("cloudiness", "precipitation", "precipitation_deposits", "wind_intensity",
                  "sun_azimuth_angle", "sun_altitude_angle", "fog_density", "fog_distance",
                  "fog_falloff", "wetness", "scattering_intensity", "mie_scattering_scale",
                  "rayleigh_scattering_scale", "dust_storm")


def validate_scenario(scenario, checkpoint_after):
    if not isinstance(scenario, dict) or not isinstance(scenario.get("map"), str):
        raise ValueError("scenario must have a map string")
    actors = scenario.get("actors")
    frames = scenario.get("frames")
    if not isinstance(actors, list) or not actors or not isinstance(frames, list):
        raise ValueError("actors must be non-empty and frames must be a list")
    if not 1 <= checkpoint_after < len(frames):
        raise ValueError("checkpoint-after must be at least one and leave a non-empty suffix")
    weather_changes = scenario.get("weather_changes", [])
    if not isinstance(weather_changes, list):
        raise ValueError("weather_changes must be a list")
    weather_frames = set()
    for change in weather_changes:
        if not isinstance(change, dict) or set(change) != {"frame", "parameters"}:
            raise ValueError("each weather change must contain frame and parameters")
        frame = change["frame"]
        if type(frame) is not int or not 0 <= frame < len(frames) or frame in weather_frames:
            raise ValueError("weather change frames must be unique valid frame indices")
        weather_frames.add(frame)
        parameters = change["parameters"]
        if (not isinstance(parameters, dict) or set(parameters) != set(WEATHER_FIELDS) or
                any(type(value) not in (int, float) or not math.isfinite(value) for value in parameters.values())):
            raise ValueError("weather parameters must contain all finite weather fields")
    labels = []
    for actor in actors:
        if not isinstance(actor, dict) or not isinstance(actor.get("id"), str) or not actor["id"]:
            raise ValueError("each actor requires a non-empty id")
        if actor["id"] in labels:
            raise ValueError("actor ids must be unique")
        if not isinstance(actor.get("blueprint"), str) or not actor["blueprint"].startswith("vehicle."):
            raise ValueError("each actor requires a vehicle blueprint")
        if set(actor) - {"id", "blueprint", "spawn_index", "transform", "attributes", "controller"} or ("spawn_index" in actor) == ("transform" in actor):
            raise ValueError("each actor requires exactly one of spawn_index or transform")
        if actor.get("controller", "manual") not in ("manual", "traffic_manager"):
            raise ValueError("controller must be manual or traffic_manager")
        if "spawn_index" in actor and (type(actor["spawn_index"]) is not int or actor["spawn_index"] < 0):
            raise ValueError("spawn_index must be a non-negative integer")
        if "transform" in actor:
            transform = actor["transform"]
            if (not isinstance(transform, dict) or set(transform) != {"x", "y", "z", "pitch", "yaw", "roll"}
                    or any(type(value) not in (int, float) or not math.isfinite(value) for value in transform.values())):
                raise ValueError("transform must contain finite x, y, z, pitch, yaw and roll")
        attributes = actor.get("attributes", {})
        if not isinstance(attributes, dict) or any(not isinstance(key, str) or not isinstance(value, str)
                                                   for key, value in attributes.items()):
            raise ValueError("attributes must map strings to strings")
        if "sticky_control" in attributes and attributes["sticky_control"].lower() != "false":
            raise ValueError("sticky_control must be false")
        labels.append(actor["id"])
    manual_labels = {actor["id"] for actor in actors if actor.get("controller", "manual") == "manual"}
    for frame_index, frame in enumerate(frames):
        if not isinstance(frame, dict) or set(frame) != manual_labels:
            raise ValueError("frame {} must contain exactly one command per manual actor".format(frame_index))
        for label, control in frame.items():
            if not isinstance(control, dict) or set(control) - CONTROL_FIELDS:
                raise ValueError("invalid control fields for {} at frame {}".format(label, frame_index))
            for key, value in control.items():
                if key in ("hand_brake", "reverse", "manual_gear_shift"):
                    if type(value) is not bool:
                        raise ValueError("{} must be a boolean".format(key))
                elif key == "gear":
                    if type(value) is not int:
                        raise ValueError("gear must be an integer")
                else:
                    if type(value) not in (float, int) or not math.isfinite(value):
                        raise ValueError("{} must be a finite number".format(key))
                    low = -1.0 if key == "steer" else 0.0
                    if not low <= value <= 1.0:
                        raise ValueError("{} is outside the valid control range".format(key))
    return labels


def demo_scenario():
    actors = [{"id": "first", "blueprint": "vehicle.lincoln.mkz_2020", "spawn_index": 0},
              {"id": "second", "blueprint": "vehicle.lincoln.mkz_2020", "spawn_index": 1}]
    frames = []
    for index in range(100):
        if index < 20:
            frames.append({"first": {"brake": 1.0}, "second": {"brake": 1.0}})
        else:
            frames.append({"first": {"throttle": 0.3, "steer": 0.05},
                           "second": {"throttle": 0.25, "steer": -0.05}})
    return {"map": "Town03", "actors": actors, "frames": frames}


def spawn_transform(world, spec):
    if "spawn_index" in spec:
        points = world.get_map().get_spawn_points()
        if spec["spawn_index"] >= len(points):
            raise ValueError("spawn index is outside the map's spawn-point list")
        return points[spec["spawn_index"]]
    value = spec["transform"]
    return carla.Transform(carla.Location(x=value["x"], y=value["y"], z=value["z"]),
                           carla.Rotation(pitch=value["pitch"], yaw=value["yaw"], roll=value["roll"]))


def spawn_actors(world, actor_specs):
    actors = {}
    try:
        for spec in actor_specs:
            blueprint = world.get_blueprint_library().find(spec["blueprint"])
            for name, value in spec.get("attributes", {}).items():
                if not blueprint.has_attribute(name):
                    raise ValueError("unknown blueprint attribute: {}".format(name))
                blueprint.set_attribute(name, value)
            if blueprint.has_attribute("sticky_control"):
                blueprint.set_attribute("sticky_control", "false")
            actor = world.try_spawn_actor(blueprint, spawn_transform(world, spec))
            if actor is None:
                raise RuntimeError("vehicle spawn failed for {}".format(spec["id"]))
            actors[spec["id"]] = actor
    except Exception:
        for actor in actors.values():
            actor.destroy()
        raise
    return actors


def step(client, world, actors, commands):
    labels = [label for label in actors if label in commands]
    batch = [carla.command.ApplyVehicleControl(actors[label].id, carla.VehicleControl(**commands[label]))
             for label in labels]
    if batch:
        responses = client.apply_batch_sync(batch, False)
        if len(responses) != len(labels) or any(response.error for response in responses):
            raise RuntimeError("control batch failed: {}".format(responses))
    previous_frame = world.get_snapshot().frame
    frame = world.tick()
    if frame != previous_frame + 1:
        raise RuntimeError("unexpected world tick: another client may be ticking")
    return {label: single.sample(world, actor) for label, actor in actors.items()}


def setup_traffic_manager(client, actors, specs, port, seed):
    tm = client.get_trafficmanager(port)
    registered = []
    try:
        tm.set_synchronous_mode(True)
        tm.set_random_device_seed(seed)
        for spec in specs:
            if spec.get("controller", "manual") == "traffic_manager":
                actor = actors[spec["id"]]
                actor.set_autopilot(True, port)
                registered.append(actor)
    except Exception:
        for actor in registered:
            actor.set_autopilot(False, port)
        tm.set_synchronous_mode(False)
        raise
    return tm


def sample_tm_controls(actors, tm_labels):
    result = {}
    for label in tm_labels:
        control = actors[label].get_control()
        result[label] = {field: getattr(control, field) for field in sorted(CONTROL_FIELDS)}
    return result


def checkpoint_traffic_manager(world, tm, restore=False, timeout=2.0):
    before = world.get_snapshot().frame
    deadline = time.monotonic() + timeout
    while True:
        accepted = tm.restore_checkpoint() if restore else tm.save_checkpoint()
        if world.get_snapshot().frame != before:
            raise RuntimeError("world advanced during Traffic Manager checkpoint operation")
        if accepted:
            return before
        if time.monotonic() >= deadline:
            raise RuntimeError("Traffic Manager {} rejected".format("restore" if restore else "checkpoint"))
        time.sleep(0.01)


def traffic_lights(world):
    result = {}
    for light in world.get_actors().filter("traffic.traffic_light"):
        key = light.get_opendrive_id()
        if not key or key in result:
            raise RuntimeError("traffic lights must have unique non-empty OpenDRIVE IDs")
        result[key] = light
    if not result:
        raise RuntimeError("no traffic lights found")
    return result


def sample_traffic_lights(lights):
    return {key: {"state": str(light.get_state()), "elapsed": light.get_elapsed_time(),
                  "green": light.get_green_time(), "yellow": light.get_yellow_time(),
                  "red": light.get_red_time(), "frozen": light.is_frozen()}
            for key, light in sorted(lights.items())}


def sample_weather(world):
    weather = world.get_weather()
    return {field: getattr(weather, field) for field in WEATHER_FIELDS}


def compare_component_traces(reference, candidate):
    if len(reference) != len(candidate):
        raise ValueError("component traces have different lengths")
    for index, (expected, actual) in enumerate(zip(reference, candidate), 1):
        if expected != actual:
            return {"exact": False, "first_difference": {"relative_frame": index,
                    "expected": expected, "actual": actual}}
    return {"exact": True, "first_difference": None}


def run_frames(client, world, actors, frames, trace, lights=None, light_trace=None, weather_trace=None,
               weather_changes=None, start_frame=0, tm_labels=(), tm_trace=None):
    weather_changes = weather_changes or {}
    for frame_index, commands in enumerate(frames, start_frame):
        if frame_index in weather_changes:
            before = world.get_snapshot().frame
            world.set_weather_synchronously(carla.WeatherParameters(**weather_changes[frame_index]))
            if world.get_snapshot().frame != before:
                raise RuntimeError("world advanced during weather change")
        trace.append(step(client, world, actors, commands))
        if tm_trace is not None:
            tm_trace.append(sample_tm_controls(actors, tm_labels))
        if lights is not None:
            light_trace.append(sample_traffic_lights(lights))
        if weather_trace is not None:
            weather_trace.append(sample_weather(world))


def compare_scene(reference, candidate, labels):
    if len(reference) != len(candidate):
        raise ValueError("rollouts have different lengths")
    for frame in reference + candidate:
        if set(frame) != set(labels):
            raise ValueError("trace actors do not match the scenario")
    per_actor = {label: single.compare([frame[label] for frame in reference],
                                       [frame[label] for frame in candidate]) for label in labels}
    return {"exact": all(result["exact"] for result in per_actor.values()), "actors": per_actor}


def checkpoint_actors(world, actors, restore=False):
    before = world.get_snapshot().frame
    for label, actor in actors.items():
        accepted = actor.restore_physics_checkpoint() if restore else actor.save_physics_checkpoint()
        if not accepted:
            raise RuntimeError("{} checkpoint rejected for {}".format("restore" if restore else "save", label))
        if world.get_snapshot().frame != before:
            raise RuntimeError("world advanced during checkpoint operations")
    return before


def checkpoint_traffic_lights(world, restore=False):
    before = world.get_snapshot().frame
    accepted = (world.restore_traffic_lights_checkpoint() if restore
                else world.save_traffic_lights_checkpoint())
    if not accepted:
        raise RuntimeError("traffic-light {} rejected".format("restore" if restore else "checkpoint"))
    if world.get_snapshot().frame != before:
        raise RuntimeError("world advanced during traffic-light checkpoint operation")
    return before


def checkpoint_weather(world, restore=False):
    before = world.get_snapshot().frame
    accepted = world.restore_weather_checkpoint() if restore else world.save_weather_checkpoint()
    if not accepted:
        raise RuntimeError("weather {} rejected".format("restore" if restore else "checkpoint"))
    if world.get_snapshot().frame != before:
        raise RuntimeError("world advanced during weather checkpoint operation")
    return before


def manifest(client, world, actors, actor_specs, build_id):
    return {label: single.collect_manifest(client, world, actors[label], spec.get("spawn_index"), build_id,
                                           spawn_transform(world, spec))
            for label, spec in ((spec["id"], spec) for spec in actor_specs)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", help="JSON file containing map, actors and frame commands; omit for demo")
    parser.add_argument("--checkpoint-after", type=int, default=40, help="number of frames before saving")
    parser.add_argument("--repetitions", type=int, default=3)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--build-id", help="user-supplied server, engine and content build identifiers")
    parser.add_argument("--traffic-lights", action="store_true", help="save and compare traffic-light group state")
    parser.add_argument("--weather", action="store_true", help="save and compare weather parameters")
    parser.add_argument("--traffic-manager", action="store_true", help="save and restore Traffic Manager state")
    parser.add_argument("--tm-port", type=int, default=8000)
    parser.add_argument("--tm-seed", type=int, default=0)
    parser.add_argument("--report", required=True, help="write the journal, traces and results as JSON")
    args = parser.parse_args()
    if args.repetitions < 1:
        parser.error("repetitions must be positive")
    if not 1 <= args.tm_port <= 65535 or not 0 <= args.tm_seed < 2**64:
        parser.error("invalid Traffic Manager port or seed")
    if args.scenario:
        with open(args.scenario, encoding="utf-8") as source:
            scenario = json.load(source)
    else:
        scenario = demo_scenario()
    try:
        labels = validate_scenario(scenario, args.checkpoint_after)
    except ValueError as error:
        parser.error(str(error))
    weather_changes = {change["frame"]: change["parameters"] for change in scenario.get("weather_changes", [])}
    if weather_changes and not args.weather:
        parser.error("weather_changes requires --weather")
    tm_labels = [actor["id"] for actor in scenario["actors"] if actor.get("controller", "manual") == "traffic_manager"]
    if bool(tm_labels) != args.traffic_manager:
        parser.error("traffic_manager actors and --traffic-manager must be specified together")

    traces = {"prefix": [], "baseline": [], "restores": [], "replay_prefix": [], "replay_suffix": []}
    light_traces = {"prefix": [], "baseline": [], "restores": [], "replay_prefix": [], "replay_suffix": []}
    weather_traces = {"prefix": [], "baseline": [], "restores": [], "replay_prefix": [], "replay_suffix": []}
    tm_traces = {"prefix": [], "baseline": [], "restores": [], "replay_prefix": [], "replay_suffix": []}
    report = {"scenario": scenario, "checkpoint_after": args.checkpoint_after, "repetitions": args.repetitions,
              "traffic_lights_enabled": args.traffic_lights, "manifest": None, "replay_manifest": None,
              "weather_enabled": args.weather, "traces": traces, "light_traces": light_traces,
              "weather_traces": weather_traces, "weather_checkpoint": None,
              "weather_after_restores": [], "traffic_manager_enabled": args.traffic_manager,
              "tm_port": args.tm_port if args.traffic_manager else None,
              "tm_seed": args.tm_seed if args.traffic_manager else None,
              "tm_control_traces": tm_traces, "summary": None, "failure": None}
    client = carla.Client(args.host, args.port)
    client.set_timeout(30.0)
    original_settings = client.get_world().get_settings()
    world = None
    actors = {}
    tm = None
    stage = "load_world"
    try:
        world = single.load_world(client, scenario["map"])
        stage = "spawn"
        actors = spawn_actors(world, scenario["actors"])
        if args.traffic_manager:
            stage = "setup_traffic_manager"
            tm = setup_traffic_manager(client, actors, scenario["actors"], args.tm_port, args.tm_seed)
        stage = "find_traffic_lights"
        lights = traffic_lights(world) if args.traffic_lights else None
        stage = "prefix"
        run_frames(client, world, actors, scenario["frames"][:args.checkpoint_after], traces["prefix"], lights, light_traces["prefix"], weather_traces["prefix"] if args.weather else None, weather_changes, tm_labels=tm_labels, tm_trace=tm_traces["prefix"] if tm else None)
        report["manifest"] = manifest(client, world, actors, scenario["actors"], args.build_id)
        stage = "save_checkpoint"
        report["checkpoint_frame"] = checkpoint_actors(world, actors)
        if args.traffic_lights:
            stage = "save_traffic_lights"
            if checkpoint_traffic_lights(world) != report["checkpoint_frame"]:
                raise RuntimeError("world advanced during traffic-light checkpoint")
        if args.weather:
            stage = "save_weather"
            if checkpoint_weather(world) != report["checkpoint_frame"]:
                raise RuntimeError("world advanced during weather checkpoint")
            report["weather_checkpoint"] = sample_weather(world)
        if tm:
            stage = "save_traffic_manager"
            if checkpoint_traffic_manager(world, tm) != report["checkpoint_frame"]:
                raise RuntimeError("world advanced during Traffic Manager checkpoint")
        suffix = scenario["frames"][args.checkpoint_after:]
        stage = "baseline"
        run_frames(client, world, actors, suffix, traces["baseline"], lights, light_traces["baseline"], weather_traces["baseline"] if args.weather else None, weather_changes, args.checkpoint_after, tm_labels, tm_traces["baseline"] if tm else None)
        restores = []
        light_restores = []
        weather_restores = []
        tm_restores = []
        for index in range(args.repetitions):
            stage = "restore_{}".format(index)
            restored_frame = checkpoint_actors(world, actors, restore=True)
            if args.traffic_lights:
                stage = "restore_traffic_lights_{}".format(index)
                if checkpoint_traffic_lights(world, restore=True) != restored_frame:
                    raise RuntimeError("world advanced between vehicle and traffic-light restores")
            if args.weather:
                stage = "restore_weather_{}".format(index)
                if checkpoint_weather(world, restore=True) != restored_frame:
                    raise RuntimeError("world advanced between vehicle and weather restores")
                report["weather_after_restores"].append(sample_weather(world))
            if tm:
                stage = "restore_traffic_manager_{}".format(index)
                if checkpoint_traffic_manager(world, tm, restore=True) != restored_frame:
                    raise RuntimeError("world advanced between vehicle and Traffic Manager restores")
            traces["restores"].append([])
            light_traces["restores"].append([])
            weather_traces["restores"].append([])
            tm_traces["restores"].append([])
            stage = "restore_suffix_{}".format(index)
            run_frames(client, world, actors, suffix, traces["restores"][-1], lights, light_traces["restores"][-1], weather_traces["restores"][-1] if args.weather else None, weather_changes, args.checkpoint_after, tm_labels, tm_traces["restores"][-1] if tm else None)
            restores.append(compare_scene(traces["baseline"], traces["restores"][-1], labels))
            if args.traffic_lights:
                light_restores.append(compare_component_traces(light_traces["baseline"], light_traces["restores"][-1]))
            if args.weather:
                weather_restores.append(compare_component_traces(weather_traces["baseline"], weather_traces["restores"][-1]))
            if tm:
                tm_restores.append(compare_component_traces(tm_traces["baseline"], tm_traces["restores"][-1]))
        stage = "replay_load_world"
        world = single.load_world(client, scenario["map"])
        actors = {}
        stage = "replay_spawn"
        actors = spawn_actors(world, scenario["actors"])
        if args.traffic_manager:
            stage = "replay_setup_traffic_manager"
            tm = setup_traffic_manager(client, actors, scenario["actors"], args.tm_port, args.tm_seed)
        stage = "replay_find_traffic_lights"
        lights = traffic_lights(world) if args.traffic_lights else None
        stage = "replay_prefix"
        run_frames(client, world, actors, scenario["frames"][:args.checkpoint_after], traces["replay_prefix"], lights, light_traces["replay_prefix"], weather_traces["replay_prefix"] if args.weather else None, weather_changes, tm_labels=tm_labels, tm_trace=tm_traces["replay_prefix"] if tm else None)
        report["replay_manifest"] = manifest(client, world, actors, scenario["actors"], args.build_id)
        stage = "replay_suffix"
        run_frames(client, world, actors, suffix, traces["replay_suffix"], lights, light_traces["replay_suffix"], weather_traces["replay_suffix"] if args.weather else None, weather_changes, args.checkpoint_after, tm_labels, tm_traces["replay_suffix"] if tm else None)
        stage = "compare"
        summary = {"restores": restores, "prefix_replay": compare_scene(traces["prefix"], traces["replay_prefix"], labels),
                   "suffix_replay": compare_scene(traces["baseline"], traces["replay_suffix"], labels),
                   "manifest_matches_replay": report["manifest"] == report["replay_manifest"]}
        if args.traffic_lights:
            summary["traffic_lights"] = {"restores": light_restores,
                "prefix_replay": compare_component_traces(light_traces["prefix"], light_traces["replay_prefix"]),
                "suffix_replay": compare_component_traces(light_traces["baseline"], light_traces["replay_suffix"])}
        if args.weather:
            summary["weather"] = {"restores": weather_restores,
                "checkpoint_restores_match": all(item == report["weather_checkpoint"]
                                                 for item in report["weather_after_restores"]),
                "prefix_replay": compare_component_traces(weather_traces["prefix"], weather_traces["replay_prefix"]),
                "suffix_replay": compare_component_traces(weather_traces["baseline"], weather_traces["replay_suffix"])}
        if tm:
            summary["traffic_manager"] = {"restores": tm_restores,
                "prefix_replay": compare_component_traces(tm_traces["prefix"], tm_traces["replay_prefix"]),
                "suffix_replay": compare_component_traces(tm_traces["baseline"], tm_traces["replay_suffix"])}
        report["summary"] = summary
        print(json.dumps(summary, indent=2, sort_keys=True))
        lights_match = not args.traffic_lights or (all(item["exact"] for item in light_restores) and
            summary["traffic_lights"]["prefix_replay"]["exact"] and summary["traffic_lights"]["suffix_replay"]["exact"])
        weather_match = not args.weather or (summary["weather"]["checkpoint_restores_match"] and
            all(item["exact"] for item in weather_restores) and
            summary["weather"]["prefix_replay"]["exact"] and summary["weather"]["suffix_replay"]["exact"])
        tm_match = not tm or (all(item["exact"] for item in tm_restores) and
            summary["traffic_manager"]["prefix_replay"]["exact"] and summary["traffic_manager"]["suffix_replay"]["exact"])
        return 0 if all(item["exact"] for item in restores) and summary["prefix_replay"]["exact"] and summary["suffix_replay"]["exact"] and summary["manifest_matches_replay"] and lights_match and weather_match and tm_match else 1
    except Exception as error:
        report["failure"] = {"stage": stage, "type": type(error).__name__, "message": str(error)}
        raise
    finally:
        original_error = sys.exc_info()[0] is not None
        cleanup_errors = []
        def cleanup(action, description):
            try:
                action()
            except Exception as error:
                cleanup_errors.append({"operation": description, "type": type(error).__name__, "message": str(error)})
        if tm:
            for label in tm_labels:
                if label in actors:
                    cleanup(lambda actor=actors[label]: actor.set_autopilot(False, args.tm_port),
                            "unregister {}".format(label))
            cleanup(lambda: tm.set_synchronous_mode(False), "disable Traffic Manager synchronous mode")
        for label, actor in actors.items():
            cleanup(actor.destroy, "destroy {}".format(label))
        if world is not None:
            cleanup(lambda: world.apply_settings(original_settings), "restore world settings")
        report["cleanup_errors"] = cleanup_errors
        with open(args.report, "w", encoding="utf-8") as output:
            json.dump(report, output, indent=2, sort_keys=True)
            output.write("\n")
        if cleanup_errors and not original_error:
            raise RuntimeError("cleanup failed; see report for details")


if __name__ == "__main__":
    sys.exit(main())
