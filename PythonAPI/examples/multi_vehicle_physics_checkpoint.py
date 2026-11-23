#!/usr/bin/env python3
"""Replay a frame-indexed, multi-vehicle journal around an in-memory checkpoint.

Use only on a dedicated server built from this branch. This is a controlled
physics test, not a checkpoint of Traffic Manager or ScenarioRunner state.
"""

import argparse
import json
import math
import sys

import carla
import vehicle_physics_checkpoint as single


CONTROL_FIELDS = {"throttle", "steer", "brake", "hand_brake", "reverse", "manual_gear_shift", "gear"}


def validate_scenario(scenario, checkpoint_after):
    if not isinstance(scenario, dict) or not isinstance(scenario.get("map"), str):
        raise ValueError("scenario must have a map string")
    actors = scenario.get("actors")
    frames = scenario.get("frames")
    if not isinstance(actors, list) or not actors or not isinstance(frames, list):
        raise ValueError("actors must be non-empty and frames must be a list")
    if not 1 <= checkpoint_after < len(frames):
        raise ValueError("checkpoint-after must be at least one and leave a non-empty suffix")
    labels = []
    for actor in actors:
        if not isinstance(actor, dict) or not isinstance(actor.get("id"), str) or not actor["id"]:
            raise ValueError("each actor requires a non-empty id")
        if actor["id"] in labels:
            raise ValueError("actor ids must be unique")
        if not isinstance(actor.get("blueprint"), str) or not actor["blueprint"].startswith("vehicle."):
            raise ValueError("each actor requires a vehicle blueprint")
        if set(actor) != {"id", "blueprint", "spawn_index"}:
            raise ValueError("each actor must specify id, blueprint and spawn_index")
        if type(actor["spawn_index"]) is not int or actor["spawn_index"] < 0:
            raise ValueError("spawn_index must be a non-negative integer")
        labels.append(actor["id"])
    for frame_index, frame in enumerate(frames):
        if not isinstance(frame, dict) or set(frame) != set(labels):
            raise ValueError("frame {} must contain exactly one command per actor".format(frame_index))
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


def spawn_actors(world, actor_specs):
    actors = {}
    try:
        for spec in actor_specs:
            actors[spec["id"]] = single.spawn(world, spec["blueprint"], spec["spawn_index"])
    except Exception:
        for actor in actors.values():
            actor.destroy()
        raise
    return actors


def step(client, world, actors, commands):
    labels = list(actors)
    batch = [carla.command.ApplyVehicleControl(actors[label].id, carla.VehicleControl(**commands[label]))
             for label in labels]
    responses = client.apply_batch_sync(batch, False)
    if len(responses) != len(labels) or any(response.error for response in responses):
        raise RuntimeError("control batch failed: {}".format(responses))
    previous_frame = world.get_snapshot().frame
    frame = world.tick()
    if frame != previous_frame + 1:
        raise RuntimeError("unexpected world tick: another client may be ticking")
    return {label: single.sample(world, actors[label]) for label in labels}


def run_frames(client, world, actors, frames, trace):
    for commands in frames:
        trace.append(step(client, world, actors, commands))


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


def manifest(client, world, actors, actor_specs, build_id):
    return {label: single.collect_manifest(client, world, actors[label], spec["spawn_index"], build_id)
            for label, spec in ((spec["id"], spec) for spec in actor_specs)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario", help="JSON file containing map, actors and frame commands; omit for demo")
    parser.add_argument("--checkpoint-after", type=int, default=40, help="number of frames before saving")
    parser.add_argument("--repetitions", type=int, default=3)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=2000)
    parser.add_argument("--build-id", help="user-supplied server, engine and content build identifiers")
    parser.add_argument("--report", required=True, help="write the journal, traces and results as JSON")
    args = parser.parse_args()
    if args.repetitions < 1:
        parser.error("repetitions must be positive")
    if args.scenario:
        with open(args.scenario, encoding="utf-8") as source:
            scenario = json.load(source)
    else:
        scenario = demo_scenario()
    try:
        labels = validate_scenario(scenario, args.checkpoint_after)
    except ValueError as error:
        parser.error(str(error))

    traces = {"prefix": [], "baseline": [], "restores": [], "replay_prefix": [], "replay_suffix": []}
    report = {"scenario": scenario, "checkpoint_after": args.checkpoint_after, "repetitions": args.repetitions,
              "manifest": None, "replay_manifest": None, "traces": traces, "summary": None, "failure": None}
    client = carla.Client(args.host, args.port)
    client.set_timeout(30.0)
    original_settings = client.get_world().get_settings()
    world = None
    actors = {}
    stage = "load_world"
    try:
        world = single.load_world(client, scenario["map"])
        stage = "spawn"
        actors = spawn_actors(world, scenario["actors"])
        stage = "prefix"
        run_frames(client, world, actors, scenario["frames"][:args.checkpoint_after], traces["prefix"])
        report["manifest"] = manifest(client, world, actors, scenario["actors"], args.build_id)
        stage = "save_checkpoint"
        report["checkpoint_frame"] = checkpoint_actors(world, actors)
        suffix = scenario["frames"][args.checkpoint_after:]
        stage = "baseline"
        run_frames(client, world, actors, suffix, traces["baseline"])
        restores = []
        for index in range(args.repetitions):
            stage = "restore_{}".format(index)
            checkpoint_actors(world, actors, restore=True)
            traces["restores"].append([])
            stage = "restore_suffix_{}".format(index)
            run_frames(client, world, actors, suffix, traces["restores"][-1])
            restores.append(compare_scene(traces["baseline"], traces["restores"][-1], labels))
        stage = "replay_load_world"
        world = single.load_world(client, scenario["map"])
        actors = {}
        stage = "replay_spawn"
        actors = spawn_actors(world, scenario["actors"])
        stage = "replay_prefix"
        run_frames(client, world, actors, scenario["frames"][:args.checkpoint_after], traces["replay_prefix"])
        report["replay_manifest"] = manifest(client, world, actors, scenario["actors"], args.build_id)
        stage = "replay_suffix"
        run_frames(client, world, actors, suffix, traces["replay_suffix"])
        stage = "compare"
        summary = {"restores": restores, "prefix_replay": compare_scene(traces["prefix"], traces["replay_prefix"], labels),
                   "suffix_replay": compare_scene(traces["baseline"], traces["replay_suffix"], labels),
                   "manifest_matches_replay": report["manifest"] == report["replay_manifest"]}
        report["summary"] = summary
        print(json.dumps(summary, indent=2, sort_keys=True))
        return 0 if all(item["exact"] for item in restores) and summary["prefix_replay"]["exact"] and summary["suffix_replay"]["exact"] and summary["manifest_matches_replay"] else 1
    except Exception as error:
        report["failure"] = {"stage": stage, "type": type(error).__name__, "message": str(error)}
        raise
    finally:
        try:
            with open(args.report, "w", encoding="utf-8") as output:
                json.dump(report, output, indent=2, sort_keys=True)
                output.write("\n")
        finally:
            for actor in actors.values():
                actor.destroy()
            if world is not None:
                world.apply_settings(original_settings)


if __name__ == "__main__":
    sys.exit(main())
