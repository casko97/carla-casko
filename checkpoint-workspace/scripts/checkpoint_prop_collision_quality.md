# Prop collision and checkpoint experiments

`checkpoint_prop_collision_quality.py` runs a scripted Town01 vehicle-versus-
cone experiment on a dedicated development editor server. It holds the
vehicle's experimental fixed-sleep setting constant across runs. It records
sampled vehicle and prop state and the vehicle collision sensor's events.
Those events are diagnostic and are not a complete PhysX contact log.

In the 2026-11-24 comparison, both variants spawned the cone in the vehicle's
path. The dynamic variant enabled prop physics; the fixed variant disabled it
and placed the cone at the measured settled height of the dynamic control.
The pre-impact heights differed by about 2.4e-8 m. The first recorded vehicle
collision with the cone was frame 111 in both runs. The dynamic cone moved
about 2.90 m from its first sampled position, while the fixed cone did not
move. At the end of the 160-frame run, sampled vehicle speed was about 0.782
m/s with the dynamic cone and 0.0165 m/s with the fixed cone. The largest
vehicle position component difference between runs was about 2.905 m. This
one constructed interaction demonstrates that fixing a prop can materially
change the outcome; it does not measure all prop types or benchmark quality.

The script also has `--checkpoint-after N` in dynamic mode. It saves the
vehicle physics slot and the prop's publicly observed transform, linear
velocity and angular velocity, then restores them in the same episode after
the baseline suffix. This is deliberately a limited probe: it does not save
the prop's sleep state, contact and solver history, actor observer cache, or
the complete physics scene. The collision sensor's own internal history is
not restored either. It is not an implementation of a portable prop snapshot.

With a checkpoint after frame 100, before the impact, the first overall
sampled difference was suffix frame 1 in the prop's velocity, angular velocity
and reported acceleration. The first vehicle-state difference and first
collision-event difference occurred at suffix frame 11, when the impact
occurred. With a checkpoint after frame 120, during the impact's aftermath,
both the vehicle and prop sampled state differed at the first suffix frame.
CARLA calculates reported acceleration using an observer velocity cache that
this public-state prop probe does not restore; that is a candidate explanation
for part of the acceleration difference, not proof of the other differences.

The reports are in `checkpoint-results/20261124-prop-impact/`. They use the
same development editor build and selected on-disk hashes, which do not prove
complete runtime or asset identity. No package build was used.

## Native same-episode prop slot

The experimental server now exposes `save_rigid_prop_checkpoint` and
`restore_rigid_prop_checkpoint` for a single active, unattached, non-kinematic
`static.prop.*` actor with one simulated root component. The slot captures the
Unreal transform and velocities, native PhysX pose and velocities, wake state
and counter, and CARLA's observer velocity cache. It checks the episode,
actor and body identity, timestep settings, selected rigid-body configuration,
and scene flags before restoring. A failed save invalidates the old slot.
The slot itself is same-actor and same-episode only; a separate experimental
portable record can be used to populate a slot in a fresh episode (below).
Restore can partially mutate the actor if a later operation fails. Shape count
and selected configuration checks do not establish that all geometry,
constraints, or other state are unchanged.

`scripts/checkpoint_prop_rpc.cpp` is a minimal research client for these RPCs;
it can be compiled locally without building a package:

```bash
g++ -std=c++14 -O2 -pthread -DBOOST_ERROR_CODE_HEADER_ONLY -Ideterministic-cpt/LibCarla/source -IBuild/rpclib-v2.2.1_c5-c10-libstdcxx-install/include -IBuild/boost-1.80.0-c10-install/include scripts/checkpoint_prop_rpc.cpp Build/rpclib-v2.2.1_c5-c10-libstdcxx-install/lib/librpc.a -o /tmp/checkpoint_prop_rpc
```

Pass `--native-prop-rpc /tmp/checkpoint_prop_rpc` to the collision probe on a
dedicated matching editor server. With a checkpoint after frame 100, the
native slot matched the sampled vehicle and prop states for the first ten
suffix frames; the first difference occurred on suffix frame 11 at impact.
The public transform/velocity probe first differed at suffix frame 1. With a
checkpoint after frame 120, the native slot still differed at suffix frame 1.
The collision sensor state, PhysX contact and solver caches, sleep-filter
accumulators, constraints and the wider world remain outside the checkpoint;
these observations do not identify which omitted state caused the differences.

The guard probe confirmed acceptance for a compatible prop and rejection for
an unsaved or destroyed actor, a vehicle, changed physics settings, disabled
prop physics, an invalidated slot, and a previous episode. Its results and the
native collision reports are in `checkpoint-results/20261124-prop-impact/`.
They cover the tested actor and settings only. An incremental editor build was
used; no package build was performed.

## Portable prop record

The experimental `export_rigid_prop_checkpoint` and
`import_rigid_prop_checkpoint` RPCs serialize a pointer-free record for one
compatible rigid prop. They check the blueprint, actor attributes, map, zero
world origin, timestep settings and selected physics configuration, including
mass, inertia, center of mass, flags and shape count. The caller supplies a
build identifier; matching it is not proof that builds or assets are identical.
Shape count does not verify all geometry or materials. Import populates a slot;
a separate restore applies it, and restore can partially mutate the actor if a
later operation fails. Contact and solver caches and other world state remain
outside the record.

Using `--portable-prop-checkpoint` with the native RPC helper, a checkpoint
after frame 100 (before the cone impact) reproduced all 60 sampled suffix
frames and diagnostic collision events after loading a fresh episode and
restoring both vehicle and prop. In the same constructed test, a checkpoint
after frame 120 (after impact) differed in the first suffix frame. These are
observations for this setup, not a general guarantee of safe checkpoint timing
or replay equivalence. The intended use prioritizes checkpoint boundaries
outside active or recent impacts; detecting suitable boundaries remains work.

The portable guard report also covers wrong build and incompatible mass
rejection, preservation of the previous slot after rejection, and import into
a fresh episode. The pre-impact replay and guard probe passed again after a
slot storage fix for a crash discovered during the incompatible-mass test.
Reports and checkpoint files are in
`checkpoint-results/20261124-prop-impact/`.

## Sensor-free live-boundary comparisons

The live probe supports `--prefix-frames` to choose a boundary in the scripted
cone scenario and `--brake-after` to brake during the prefix. These are explicit
experiment controls, not automatic selection or rejection of checkpoint times.
With fixed map props and the fixed vehicle sleep option, capture at prefix
frame 120 or 150 during the interaction produced differences in both vehicle
and cone state on the first replay tick. Over the next 80 ticks, maximum
Euclidean position differences were respectively 0.0928 m and 0.0843 m for
the vehicle, and 0.1091 m and 0.0954 m for the cone. Maximum speed-vector
differences were 0.1046 and 0.1246 m/s for the vehicle, and 0.1398 and
0.1217 m/s for the cone.

Another run braked the vehicle from prefix tick 160 and captured at tick 300.
At capture, the sampled vehicle speed was approximately 0.000083 m/s and cone
speed approximately 0.0043 m/s. Replay then accelerated the vehicle again.
Both sampled states differed on the first replay tick; over 80 ticks maximum
position errors were 0.1064 m for the vehicle and 0.1072 m for the cone, and
maximum velocity-vector errors were 0.1294 m/s and 0.1471 m/s. These results
do not establish why the divergence occurs or whether an unobserved contact
persists at the checkpoint. They are materially larger than the tiny differences
accepted in the earlier Traffic Manager comparison. No sensors were used in
these three comparisons. The checkpoints and traces are in
`checkpoint-results/20261124-live-vehicles/` under
`checkpoint-live-impact-boundary-120`, `checkpoint-live-impact-boundary-150`,
and `checkpoint-live-impact-settled-300`.
