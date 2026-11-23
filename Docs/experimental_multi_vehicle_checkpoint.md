# Multi-vehicle checkpoint harness

`PythonAPI/examples/multi_vehicle_physics_checkpoint.py` extends the first
vehicle-only harness to a fixed set of independently controlled vehicles. The
caller supplies every vehicle's control at every simulation frame; all controls
are submitted in one acknowledged batch, followed by exactly one tick.
At a selected frame the harness saves each vehicle's in-memory slot while the
world is stopped, records the uninterrupted suffix, repeatedly restores all
slots and replays the suffix, and finally reloads the map and replays the entire
journal as a separate reference. A report contains manifests, per-actor traces,
the command journal and actor-specific comparison results.

For a dedicated test server built from this branch, once experiments are allowed:

```sh
python PythonAPI/examples/multi_vehicle_physics_checkpoint.py \
  --checkpoint-after 40 --repetitions 3 --report multi-vehicle-report.json

python PythonAPI/examples/multi_vehicle_physics_checkpoint.py \
  --checkpoint-after 40 --repetitions 3 --traffic-lights --report with-lights.json

python PythonAPI/examples/multi_vehicle_physics_checkpoint.py \
  --checkpoint-after 40 --repetitions 3 --traffic-lights --weather --report with-weather.json
```

The default is a two-vehicle Town03 demonstration. Its spawn points and actions
are not guaranteed to avoid collisions; inspect the report and map when running
it. Supply `--scenario journal.json` for recorded fixed-actor controls. The JSON
format is:

```json
{
  "map": "Town03",
  "actors": [
    {"id": "ego", "blueprint": "vehicle.lincoln.mkz_2020", "spawn_index": 0},
    {"id": "other", "blueprint": "vehicle.lincoln.mkz_2020", "spawn_index": 1}
  ],
  "frames": [
    {"ego": {"throttle": 0.2}, "other": {"brake": 1.0}},
    {"ego": {"steer": 0.1, "throttle": 0.2}, "other": {"throttle": 0.2}}
  ]
}
```

Choose `--checkpoint-after 1` for this two-frame example. The value counts
completed journal frames, so it must be at least one and leave at least one
suffix frame. Controls use `carla.VehicleControl` field names; omitted fields
take its defaults. Each frame must explicitly contain every actor, actor ids
must be unique, and each actor must use a vehicle blueprint and spawn-point
index. Use the same `--build-id` convention as the single-vehicle harness.

The multi-vehicle save consists of sequential per-vehicle RPCs, not one atomic
world transaction. The single-tick-owner requirement is essential. The harness
checks that the client's observed world frame has not advanced during the save
or restore, but it cannot fence another client or prove that no unobserved
server-side work is in flight. A failed save or restore invalidates the run;
there is no rollback of previously saved or restored vehicles. The report
preserves completed traces and the failing stage where possible.

This step does **not** create a complete scenario checkpoint. It uses a fixed
actor set from map spawn points, does not capture actor creation and deletion,
Traffic Manager, ScenarioRunner, pedestrian controllers,
sensors or their random states, and does not restore PhysX scene contact and
solver caches. The actor snapshots compare kinematics; collision events are not
recorded independently. The checkpoint slots exist only in the same episode,
and the prefix replay after a world reload is a reference, not a direct restore
into a fresh process. See [the single-vehicle checkpoint guide](experimental_vehicle_physics_checkpoint.md)
for further implementation constraints. Neither a full build nor a CARLA
runtime test has been performed for this harness.

With `--traffic-lights`, the harness additionally saves and restores the
same-episode traffic-light groups, controller state, phase timings and individual
light states. It compares each exposed light's state, elapsed time, durations
and frozen flag after each tick, using unique non-empty OpenDRIVE IDs. It rejects
maps whose exposed traffic lights lack such IDs or have duplicates. The server
checkpoint also validates group/controller/light topology and the fixed timestep.
The group slot is in memory and is invalid after a world reload. Saving or
restoring the vehicles and traffic lights is sequential, not one world-wide
transaction. Light restoration dispatches Unreal light-change callbacks and
does not restore their external side effects, such as AI or scenario state. The
public per-light comparison cannot prove that all internal traffic-light state
or callback side effects were restored, and this path has not been run in CARLA.

With `--weather`, the harness saves the current CARLA weather parameters and
the weather actor's day/night switch and restores them in the same episode. It
records and compares all 14 exposed weather parameters after each frame. The
public trace does not expose the day/night switch, and the command journal does
not yet support frame-by-frame weather changes. Applying weather invokes
Blueprint callbacks and refreshes sensor post-processing; their internal state,
scene lights and rendering history are not captured. The weather path is also
untested in a live simulator.
