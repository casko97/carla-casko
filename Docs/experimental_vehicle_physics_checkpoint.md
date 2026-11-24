# Experimental single-vehicle physics checkpoint

This prototype investigates whether a single vehicle can be restored within an
existing synchronous episode and driven again. **It is not yet a complete or
validated simulator checkpoint.** It is an in-memory slot owned by one vehicle;
it is lost when the actor, world or server is destroyed. It is not a disk format
and cannot be transferred to another vehicle, process, build or episode.

With the matching server and Python API from this branch, call
`vehicle.save_physics_checkpoint()` after a completed tick and before queuing
the next frame's commands. Call `vehicle.restore_physics_checkpoint()` between
ticks to restore the slot. Both return `False` if their preconditions fail.
They block until the server processes the request. Only the same Python vehicle
handle that saved the slot can restore it. Its first direct control after a
restore is always sent, so the handle's sticky-control cache cannot suppress
that command. Other handles' caches are not synchronized. The most recent
successful save replaces the previous slot. A failed save does not invalidate
a previous successful save.
Restore checks that the physics body matches the vehicle drive before changing
the vehicle, but is not a transaction: if a physics write fails after the transform is applied, it can
return false after partially changing the vehicle. Treat a failed restore as
invalidating the current run and reload the world.

The world must be synchronous with a fixed timestep, and a single client must
own all ticks. Use one active PhysX wheeled vehicle with the default movement
component, no autopilot, no Ackermann or constant-velocity controller, no RVO,
no sensors and no moving actors. The caller is responsible for these conditions;
the API checks only the conditions visible on the vehicle. Do not change the
vehicle physics configuration, map, world settings, actors or controller mode
between save and restore. The API checks the PhysX drive object's identity and
vehicle setup tag, but cannot detect every possible configuration change.
Restore also checks that the fixed timestep and physics substep settings still
match the save. These checks do not track every world setting.
It compares the exposed vehicle physics configuration, including the torque
curve, while excluding wheel positions because CARLA reports those as changing
simulation state. This catches some configuration changes; hidden PhysX and
Unreal properties and curve interpolation details can still differ.
Changing physics control, wheel friction, movement component, physics-enabled
state or doors invalidates the slot. Saving and restoring reject a vehicle with
a door currently simulating physics. Other changes can still escape detection.

The slot currently captures the actor transform, linear and angular velocity,
the underlying PhysX body's pose and velocities without Unreal conversion, and
awake/asleep flag and PhysX wake counter,
CARLA input and previous control, vehicle light state, failure/rollover state and
the rollover timer's existence, remaining time, rate and paused state, PhysX drive dynamic
data (including analog inputs, engine, gearbox and gear-change timers), wheel
rotation speeds and angles, corrected wheel speeds, tire low-speed timers,
previous suspension jounce, Unreal raw and smoothed control inputs, and the
world observer's previous-velocity cache used to calculate reported acceleration.
Restoring light state invokes the vehicle's Blueprint refresh callback if the
state differs; arbitrary Blueprint or rendering side effects are not saved.
The rollover timer is rescheduled against the current Unreal timer clock; its
remaining time is exposed by Unreal as a float and exact timing is unverified.
It does not restore simulation time or the PhysX scene's contact manifolds,
solver caches, broadphase state, outstanding forces or
suspension-query history. It does
not save traffic lights, random generators, scenario state, sensors, render
history or other actors. Consequently even an immediate continuation may differ,
especially during contacts. A successful return means the supported fields were
applied; it does not imply an equivalent or deterministic physics state.
Wheel history uses a private PhysX 3.4 source header from the matching Unreal
engine. If that header is unavailable at build time, this experimental API
rejects checkpoint saves and restores instead of silently omitting that state.
Its layout is engine-version-specific and must be re-audited if PhysX changes.

`PythonAPI/examples/vehicle_physics_checkpoint.py` is a future validation
harness. On a **dedicated** server built from this worktree, it loads a map and
creates one vehicle. It runs a fixed warm-up and prefix, saves, records an
uninterrupted suffix, restores and repeats the suffix. It also reloads the world
and replays the prefix and suffix from the beginning. Its JSON reports the first
numerical difference and maximum per-field absolute error in actor snapshot data.
Equality uses Python floating-point equality, not a bitwise comparison.
It also compares the reload warm-up, so a divergence before the prefix is not
hidden. It returns nonzero if any comparison differs. Absolute server frame numbers
continue advancing after a restore and are not compared as physical state.

For example, once running tests is permitted:

```sh
python PythonAPI/examples/vehicle_physics_checkpoint.py --map Town03 --repetitions 3
python PythonAPI/examples/vehicle_physics_checkpoint.py --map Town03 --profile maneuver --report checkpoint-maneuver.json
```

The `maneuver` profile journals steering in both directions, braking, and
manual shifts between gears one and two. `--report` saves a manifest, the
explicit commands for every frame, the comparison summary, and all recorded
traces as JSON. The harness compares the initial and replayed manifests as
well as the traces. Those commands are replayed identically for each continuation.
For slow map loads, set `--timeout` to a larger RPC timeout in seconds; the
default is 30 seconds.
If a run fails after report initialization, the report includes the failing
stage and any traces completed before the error. Non-finite values are emitted
using Python's JSON extensions so they remain visible for diagnosis.
Client and server versions, vehicle attributes and physics settings are included,
but version strings do not establish binary identity. Supply an externally
computed build and asset identifier using `--build-id`.
With `--build-id ID --export-checkpoint vehicle.msgpack`, the harness also writes
the saved vehicle slot as a versioned, pointer-free MessagePack value record.
The build identifier is supplied by the caller and is not verified by CARLA.
The version 2 record includes the native PhysX body pose and velocities and the
world observer's velocity cache. Version 1 records do not contain those fields
and do not pass the version 2 structural check. The change requires matching
client and server builds. This is export only: no import or fresh-episode
restore is implemented yet, and the file omits scene contact and solver state, other
actors and sensors.
The manifest also records the active PhysX scene's raw flag bitmask and selected
decoded PhysX 3.4 flags via a synchronous server query. In this engine's headers, enhanced
determinism is bit 20, GPU dynamics bit 19, stabilization bit 14, disabled
contact cache bit 11 and persistent contact manifolds bit 9. The project's
`DefaultEngine.ini` requests enhanced determinism, but config overrides and
different builds can change the active scene. The flag is set at scene creation
and cannot recover a saved scene's solver or contact caches. Matching flags are
not evidence that restored physics will reproduce a trajectory.

This script changes the server's map and settings and destroys its vehicle. It
must not be pointed at a shared server. The worktree contains source files only;
the server and Python API must first be built against matching CARLA and Unreal
dependencies and content. `vehicle.lincoln.mkz_2020` is the default blueprint;
check that the selected map, spawn point and suffix avoid other objects.

The intended next step is to record failures and first divergences, then extend
the native state and validation one subsystem at a time. An exact prefix replay
is the reference: it is an independent check and a fallback when direct restore
cannot reconstruct the needed hidden state.
