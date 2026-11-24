# Checkpoint runtime observations: 2026-11-23

These are observations from one machine, not a determinism guarantee. The
server and Python API were built from the `deterministic-cpt` worktree at
`29f1809`; the final single-vehicle run additionally included the observer
velocity-cache fix in this worktree. The tests used Town01, the Lincoln MKZ
2020 vehicle, synchronous mode, a 0.05 s fixed timestep, substepping enabled,
0.01 s maximum substep and 10 maximum substeps. The active PhysX scene reported
enhanced determinism enabled, GPU dynamics disabled, and the contact cache
enabled. Asset and binary identity were not independently hashed. The server
log reported missing walker animation assets and building Blueprint load errors;
the tests below did not exercise walkers or building spawning.

The harness compares returned Python numeric values exactly, not raw floating
point representations. The single-vehicle run used 20 warmup ticks, 60 prefix
ticks, 80 suffix ticks and three restores. On both builds, reloading the map and
replaying the complete journal produced exactly equal sampled warmup, prefix and
suffix values. All three same-episode checkpoint restores differed on the first
suffix tick. The first restore's first difference was the vertical position:
-0.004776515997946262 m in the baseline and -0.004776630084961653 m after
restoration. Its maximum position error over the suffix was 0.000244140625 m.

Before the observer fix, the first restore's maximum acceleration difference was
6.8929378390312195 m/s². After saving and restoring the world observer's
previous-velocity cache, it was 0.031087398529052734 m/s². The position and
velocity errors remained the same in this comparison, so restoring that cache
fixes a source of reported acceleration error but does not make the physics
checkpoint exact. A likely remaining source is uncaptured PhysX scene or wheel
history; these tests do not establish which internal value causes divergence.

On 2026-11-24, the same single-vehicle setup was run at `bdbbb30` using the
`maneuver` profile, which commands steering in both directions, braking, and
manual gear changes. The full warmup, prefix and suffix replay matched exactly.
All three checkpoint restores differed on the first suffix tick; their maximum
position component errors over the 80-tick suffix were 0.007996 m, 0.003815 m
and 0.010529 m. The local report is
`/tmp/carla-checkpoint-maneuver-bdbbb30.json`.

With two manually controlled vehicles, 40 prefix frames and 60 suffix frames,
the prefix replay and suffix replay matched exactly. None of the three
checkpoint restores was exact. The largest component-wise position error was
0.053863525390625 m. Traffic-light, vehicle-light and weather comparisons
matched in this particular run. Collision-event logs also matched, but all
were empty: this was not a collision test.

With one manual and one Traffic Manager vehicle in the same scenario, full
prefix and suffix replay matched exactly. The two checkpoint restores differed
in vehicle state, and the first Traffic Manager control difference appeared at
relative suffix frame 3. The observation does not distinguish missing manager
state from its response to already-divergent vehicle state.

On 2026-11-24, a separate Town01 run used
`PythonAPI/examples/checkpoint_scenarios/town01_vehicle_collision.json`: one
vehicle accelerated into a second vehicle held with the brake. The first impact
occurred at scenario frame 60, and the checkpoint was taken immediately after
that tick. Full replay matched the sampled actor states and collision events
exactly. Both restored runs diverged in both vehicles' state on the first suffix
tick; the first subsequent collision impulse differences appeared at suffix
frame 3. Maximum position component error was 0.00785 m over 90 suffix ticks.
This is evidence that the checkpoint is not exact in this contact scenario,
but does not isolate which missing state caused the divergence. A first attempt
timed out reloading the map with the default 30-second RPC timeout; the
successful run used `--timeout 180`. Its local report is
`/tmp/carla-checkpoint-collision-impact-bdbbb30.json`.

## Native PhysX body pose and velocity experiment

On 2026-11-24, a server built from `c127f47` plus the native body-state
checkpoint change captured the PhysX rigid actor's global pose and linear and
angular velocities directly, then restored those values after updating the
Unreal component. Under the same settings, all three straight-driving restores
and all three maneuver restores matched the uninterrupted 80-tick suffix in
every sampled kinematic field. The warmup, prefix and suffix from a full
reload-and-replay also matched. In the collision scenario, both restores and
full replay matched all sampled actor fields and collision-event records;
each restored suffix contained 172 sensor-event records across the two sensors.
This is exact numeric comparison of these recorded values, not proof that all
internal state or other scenarios match. The prior results above describe the
earlier implementation and are retained for comparison.

A Traffic Manager run on the same build did not complete: the server crashed
with an assertion in Unreal's tagged skeletal-mesh scene-proxy rendering path.
The crash's cause and relationship, if any, to the checkpoint change are not
established. It is not evidence of a successful or failed Traffic Manager
restore. Local reports for the completed runs are
`/tmp/carla-checkpoint-native-body-straight.json`,
`/tmp/carla-checkpoint-native-body-maneuver.json` and
`/tmp/carla-checkpoint-native-body-collision.json`.

The complete local reports were written to `/tmp/carla-checkpoint-straight.json`,
`/tmp/carla-checkpoint-straight-observer.json`,
`/tmp/carla-checkpoint-multi.json`, and `/tmp/carla-checkpoint-tm.json`.
These temporary files are not part of the repository. Repeat runs should retain
their own reports and verified build identifiers. Broader contacts, pedestrians,
general sensor state and the VAD closed loop remain unverified.
