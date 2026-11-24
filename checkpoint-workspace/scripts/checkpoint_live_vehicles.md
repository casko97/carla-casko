# Live manual-vehicle checkpoint experiment

`checkpoint_live_vehicles.py` can save selected vehicle physics state from an
already-running synchronous episode on a locally fingerprintable flat map. Capture reads the current frame;
it does not advance or reload the source world. It does replace the selected
vehicles' existing experimental checkpoint slots. The caller must be the sole
tick owner and prevent other clients or asynchronous commands from changing
the world during capture. Frame checks do not provide a server-wide barrier.

The source map must have local main map, built-data, OpenDRIVE, Traffic Manager
and navigation files in the standard flat-map locations. The identity hashes
those selected files, not every sublevel or asset. The source must use the harness settings: synchronous mode, 0.05 s fixed
delta, substepping enabled, maximum substep delta 0.01 s, ten maximum substeps,
and deterministic ragdolls. All visible vehicles must be included by CARLA
actor ID, use `sticky_control=false`, and be manually controlled. The tool
cannot detect whether another client or Traffic Manager is controlling them;
that remains a caller responsibility. It can optionally capture explicitly selected manual pedestrians as described below.
It rejects sensors, walker AI controllers and, by default, unselected props. Town01 contains dynamic map props;
`--allow-uncaptured-props` permits an experiment while recording their count,
but their state is omitted. The opt-in fixed-prop mode described below instead
fixes map props from scenario initialization. Traffic-light controller state can
optionally be saved as described below. Other unrecognized actors,
ScenarioRunner, pending messages, sensor state, simulation clock, and PhysX
contact/solver caches are not saved. The source origin is assumed to be zero.

The development source now records the world origin in the in-memory vehicle
slot and rejects portable export from a nonzero captured origin or when the
current origin differs; same-episode vehicle restore also rejects a differing
current origin. A change away from and back to the same origin is not detected.
The portable record still only supports zero-origin import. This guard does not
restore streaming or rebasing state, and it must be present in the running
server build to apply. Shifted-origin rejection has not yet been exercised in a
live scenario. A rebuilt server passed a Town02 zero-origin smoke test: one
vehicle's sampled states matched for 40 replay frames. The result is in
`checkpoint-results/20261124-live-vehicles/checkpoint-origin-guard-zero-town02/`.

The development source also records the origin in the in-memory dynamic-prop
slot and checks it at restore and portable export. The incremental editor
build passed. A Town01 non-contact smoke test matched sampled vehicle and prop
states for 20 frames; the minimum sampled center-to-center distance was 7.04 m.
Its artifacts are in `checkpoint-results/20261124-live-vehicles/checkpoint-origin-guard-prop-zero-town01/`.
Shifted-origin rejection for props remains untested.

The resulting file stores selected vehicle slots, sampled boundary state,
weather, source frame and selected on-disk artifact hashes. The hashes do not
cover all loaded code or assets or prove the server is running those bytes.
Replay restores the captured weather at the boundary. The caller must keep
weather fixed for the suffix; later weather changes are not journalled. The
optional action recorder checks the current weather against that boundary
before and after each step and before publishing, but cannot detect a change
that is reversed between checks.
Bootstrap transforms specify where to spawn fresh actors before applying their
saved state. Supply the original spawn transforms when known: initializing at
a different pose can alter physics history. Without them, the fallback spawns
one metre above each captured pose and may fail or diverge.

For example, with a dedicated compatible server, a JSON file `actors.json`
containing `{"ego": 123}` maps the label to the current CARLA actor ID. A
bootstrap JSON file maps each label to an object with `x`, `y`, `z`, `pitch`,
`yaw`, and `roll`. The actor ID and poses must come from the actual episode.

```bash
export PYTHON_EGG_CACHE=/tmp/carla-egg-cache
export PYTHONPATH=deterministic-cpt/PythonAPI/carla/dist/carla-0.9.15-py3.8-linux-x86_64.egg
.venv/bin/python scripts/checkpoint_live_vehicles.py capture --actors actors.json --bootstrap-transforms bootstrap.json --allow-uncaptured-props --checkpoint /tmp/live-vehicles.json --port 23000
.venv/bin/python scripts/checkpoint_live_vehicles.py replay --checkpoint /tmp/live-vehicles.json --actions actions.json --reference baseline.json --report /tmp/live-restore.json --port 23000
```

The `actions.json` file is a list of at least two frame dictionaries mapping
every saved label to a `VehicleControl` dictionary. This replay operation
loads a fresh map, replaces the target world, and executes those controls; use
a dedicated target server. Optional `baseline.json` is a list of sampled
frames in the format produced by `checkpoint_scenario_bundle.execute`. Without
a reference, the report's `exact` value is `null`; executing a replay alone
does not establish reproducibility. The alternative lifecycle journal described below supports later vehicle
creation and destruction. Commands for other components remain unsupported.

`checkpoint_live_probe.py` is a dedicated-server experiment: it drives a
vehicle, captures it in the running episode without ticking, records 40 later
actions and observed baseline frames, then reloads and compares the suffix.
On 2026-11-24, with the experimental fixed-sleep mode and 115 uncaptured map
props, two runs using the original spawn pose matched all 40 sampled suffix
frames. A run that bootstrapped from the captured pose plus one metre diverged
at frame 1 even though its recaptured portable fields matched. Its boundary
observation and baseline matched the original-spawn runs. This shows a
bootstrap-sensitive result for this case, not a general guarantee. Reports
are in `checkpoint-results/20261124-live-vehicles/`.

A further test with two manual vehicles and their original spawn transforms
also matched all 40 sampled suffix frames after fresh-episode restore. It used
the same fixed-sleep mode and omitted the 115 visible map props. These tests
do not validate moving props, interactions with traffic lights, sensor output,
or recovery of a complete running scenario.

## Fixed map props

`prepare-fixed-props` is an opt-in experiment for a newly loaded synchronous
world, before scenario vehicles, walkers, or sensors are spawned. It sends
physics-disable commands to all visible `static.prop.*` actors, advances one
frame, and records their observed IDs, attributes, poses and velocities in a
guard file. These commands change the world, and a partial command failure is
not rolled back. Do not run preparation midway through a scenario. Fixing props
prevents them from behaving as normal dynamic objects, including during
collisions; the effect on benchmark quality requires separate evaluation.

```bash
.venv/bin/python scripts/checkpoint_live_vehicles.py prepare-fixed-props --fixed-props-guard /tmp/fixed-props.json --port 23000
# Start and control the scenario while keeping the props fixed.
.venv/bin/python scripts/checkpoint_live_vehicles.py capture --actors actors.json --bootstrap-transforms bootstrap.json --fixed-props-guard /tmp/fixed-props.json --checkpoint /tmp/live-fixed-props.json --port 23000
```

Capture verifies that the source actor IDs, observed poses, velocities and
attributes still match the guard. During fresh-episode replay, the tool fixes
target props before spawning vehicles, checks the same visible prop records
independently of runtime IDs, and checks them again after the suffix. The
acknowledged commands and observations do not independently prove the current
simulate-physics flag: CARLA's Python API has no public readback for it. Other
clients must not re-enable or otherwise mutate the props. The guard is not an
authenticated server receipt, and matching visible state does not capture
dynamic prop internals, unrecognized actor types, or other world state.

In one two-vehicle fixed-sleep experiment, all 115 visible Town01 props were
fixed at initialization. The live checkpoint and fresh-episode replay matched
all 40 sampled vehicle suffix frames, and the visible fixed prop state matched
before and after replay. A modified prop pose in the checkpoint caused replay
to reject the scene. The sampled vehicle baseline also matched the earlier
unfixed-prop run for those 40 frames; those vehicles did not exercise prop
collisions, so that comparison does not establish acceptable quality when
vehicles interact with props. Results are in
`checkpoint-results/20261124-live-vehicles/checkpoint-live-fixed-props-v2/`.

A separate controlled cone impact showed materially different vehicle and
prop behavior with physics fixed. See `checkpoint_prop_collision_quality.md`;
fixed props should not be treated as behavior-preserving when such interactions
matter.

## Explicit dynamic props

The live capture experiment also accepts `--props`, a JSON map of labels to
live actor IDs, `--prop-bootstrap-transforms`, a JSON map of those labels to
their original spawn transforms, and `--prop-rpc`, the path to the standalone
native prop helper described in `checkpoint_prop_collision_quality.md`. Each
transform has `x`, `y`, `z`, `pitch`, `yaw`, and `roll`. Labels must be unique
across vehicles and props. The selected props must satisfy the server's rigid
prop constraints, including active simulated physics and zero world origin.
Use the same server port for the helper and Python client. Capture overwrites
each selected prop's experimental slot and embeds its portable record in the
JSON checkpoint; the caller must control all ticks and concurrent mutations.

Unselected props still require either the opt-in fixed-prop guard, prepared
before spawning the dynamic props, or `--allow-uncaptured-props`. The latter
does not save or constrain them. The fixed-prop guard checks the original map
prop set while excluding the explicitly selected dynamic props. It does not
prove that physics remains disabled. The helper reconstructs selected props
at their supplied initial transforms, enables physics, advances one frame,
then spawns the vehicles, warms the world and applies the saved states. This
particular initialization order is not a general actor lifecycle journal.

Replay requires `--prop-rpc` when dynamic props are present. An optional
`--prop-reference` supplies one label-to-sampled-state object per action
frame. The report records `prop_exact` and the first difference separately
from vehicle `exact`; absent a prop reference, `prop_exact` is null. Sampling
does not cover all physics internals, contact state, sensors or controllers.
There is no automatic check of whether the chosen capture time is appropriate.

On 2026-11-24, the live probe captured before a cone impact and replayed 60
actions in a fresh episode. One- and two-vehicle tests with 115 fixed map
props matched all sampled vehicle and cone frames; the cone moved about 2.9 m
during the suffix. A one-vehicle test with those map props left dynamic and
uncaptured also matched the sampled frames. These experiments used the opt-in
fixed vehicle sleep setting and one constructed interaction; they do not
establish determinism or acceptable quality for arbitrary scenarios. The
reports are in `checkpoint-results/20261124-live-vehicles/` under the
`checkpoint-live-props-*` directories. No package build was performed.

## Portable traffic-light controllers

With `--traffic-rpc /tmp/checkpoint_traffic_rpc`, live capture also exports
the traffic-light groups' current controller, frozen flags, per-controller
stage definitions and indices, elapsed time, sequence, and component and actor
signal states. Replay requires the same helper and imports the record after
warmup and before applying the vehicle and prop checkpoints. It checks the
map, fixed timestep, caller-supplied build identifier, junction IDs, controller
IDs, and ordered OpenDRIVE signal IDs. It rejects unsupported or ambiguous
topology. The selected-file identifier is not a proof of complete runtime or
asset identity. The helper can be compiled without a package build:

```bash
g++ -std=c++14 -O2 -pthread -DBOOST_ERROR_CODE_HEADER_ONLY -Ideterministic-cpt/LibCarla/source -IBuild/rpclib-v2.2.1_c5-c10-libstdcxx-install/include -IBuild/boost-1.80.0-c10-install/include scripts/checkpoint_traffic_rpc.cpp Build/rpclib-v2.2.1_c5-c10-libstdcxx-install/lib/librpc.a -o /tmp/checkpoint_traffic_rpc
```

Import prepares target-bound slots before applying the group states. An import
that reaches that preparation stage invalidates the existing same-episode
traffic-light checkpoint even if it is later rejected. It does not restore
traffic-light trigger overlap lists, vehicle/AI controller state, pending
events, the simulator clock, or arbitrary external modifications. Restoring
component states may invoke signal-change callbacks. A single tick owner and
exclusive control of the episode remain required.

An optional `--traffic-reference` file contains a list of per-frame maps keyed
by OpenDRIVE ID, with the observed state, elapsed time, stage durations and
frozen flag. The replay reports `traffic_exact` and the first difference; a
missing reference produces null. The test probe's `--traffic-diagnostics`
records sampled lights, and `--traffic-count` controls the number selected
evenly by ID. These samples do not measure all controller internals or all
signals.

In a 2026-11-24 test without a portable light record, vehicle and prop samples
matched but a sampled light's elapsed time differed on suffix frame 1. With
the record, a one-vehicle, one-cone test matched 120 vehicle and prop frames
and four sampled lights, including a green-to-yellow transition. A two-vehicle
test with the cone and 115 fixed map props matched 120 sampled frames for
vehicles, prop and 12 sampled lights, including four observed transitions.
Wrong-build and incompatible-controller imports were rejected without ticking
or changing visible sampled lights; a compatible import was accepted.
Reports are under `checkpoint-results/20261124-live-vehicles/`. These
results do not prove behavior for arbitrary maps, controllers or scenarios.

## Vehicle lifecycle in a replay suffix

`--actions` still accepts the controls-only list described above. It also
accepts an object with `actors` and `frames`. `actors` contains definitions
for vehicles that may be created after the checkpoint, using the same
blueprint, attributes and spawn-index or explicit-transform format described
in `checkpoint_journal.md`. It must not repeat a checkpoint vehicle or prop
label. `frames` contains ordered `events` and `controls` for each replay tick:

```json
{
  "actors": [{"id": "later", "blueprint": "vehicle.lincoln.mkz_2020", "spawn_index": 1}],
  "frames": [
    {"events": [{"action": "spawn", "label": "later"}],
     "controls": {"ego": {"throttle": 0.2}, "later": {"brake": 1.0}}},
    {"events": [{"action": "destroy", "label": "later"}],
     "controls": {"ego": {"throttle": 0.2}}}
  ]
}
```

Events execute in their listed order before controls and the tick. Controls
must name exactly the vehicles alive after those events; at least one must
remain alive each frame. The complete journal is validated before the target
map is loaded. A checkpoint vehicle can be destroyed and respawned using its
original checkpoint spawn specification. The caller supplies the journal;
this harness does not automatically intercept external commands or record
Traffic Manager decisions. Prop lifecycle events, other actor types, sensor
outputs and arbitrary scenario commands are not represented.

In 2026-11-24 tests, a live checkpoint with a dynamic cone, fixed map props
and portable traffic lights was followed by a 120-frame journal. A vehicle
spawned, was destroyed, respawned and was destroyed again. In a second run, a
vehicle present at capture was also destroyed and respawned. Both fresh-episode
replays matched sampled vehicle and cone states and eight sampled traffic
lights for all 120 frames. These runs used the opt-in fixed vehicle sleep
mode and do not establish general lifecycle or scenario determinism. Results
are in the `checkpoint-live-lifecycle-*` directories under
`checkpoint-results/20261124-live-vehicles/`.

## Derived vehicle lights

The optional `LiveActionRecorder` adapter applies the checkpoint's derived-light
policy when executing recorded vehicle controls, and checks the checkpoint's
selected map and the live world's map. When a policy is present, it writes
`vehicle_light_reference.json` for comparison during replay. It currently
rejects walkers and does not record walker controls or lifecycle events; a
caller can supply the walker journal separately to the replay interface.
In a 2026-11-24 Town02 experiment, two initial vehicles, six spawn/destroy
events, fixed map props, traffic-light controllers and derived vehicle lights
were recorded for 120 frames. The saved action and reference files were read
back and replayed in a fresh episode; sampled vehicle, traffic-light and
vehicle-light traces matched exactly. The recording manifest's checkpoint hash
matched the saved checkpoint. This test did not exercise contacts or walkers.
The artifacts are in `checkpoint-results/20261124-live-vehicles/checkpoint-live-recorder-town02-20261124/`.

For manually controlled vehicles, the optional
`scripts/checkpoint_vehicle_light_policy.py` policy derives brake and reverse
lights each tick from that tick's recorded controls. Brake lights are on when
`brake > 0.5`; reverse lights follow the `reverse` control flag. The other
lights use a fixed bit mask. For example, `{"mode": "fixed-plus-control-v1",
"fixed_mask": 3}` fixes position and low-beam lights on and leaves indicators,
fog lights, high beams and special lights off. A different supported fixed
mask can be supplied, excluding the derived brake and reverse bits. This is
a deliberate light-behavior change and may matter for camera data; turn
indicators are not inferred from steering or route intent.

Apply the same policy from scenario initialization, for every manually
controlled vehicle and every tick, with Traffic Manager and other light writers
disabled. `checkpoint_journal_bundle.execute(world, client, actors, frame,
specs, policy)` applies the light batch after lifecycle events and before
controls and the tick. On capture, pass the policy as `vehicle_light_policy`
to `checkpoint_live_vehicles.capture` or pass a JSON file with
`--vehicle-light-policy`. Capture checks that current light states match the
last observed controls and policy; that check does not prove the policy was
used throughout the prefix. The checkpoint records the policy configuration
and the existing portable vehicle state includes the boundary light state.
Replay applies the policy automatically; light commands need not be added to
the actions journal. Set `--vehicle-light-reference` to a JSON list of
label-to-integer light states to compare sampled light output in the report.

On 2026-11-24, a two-vehicle live test used fixed position and low-beam lights,
with brake and reverse lights derived as above. The 120-frame suffix included
braking, reversing, new vehicle spawn and respawn, a dynamic cone and sampled
traffic-light transitions. Fresh-episode replay matched all sampled vehicle
light states as well as the sampled vehicle, prop and traffic-light traces.
The report and reference are in
`checkpoint-results/20261124-live-vehicles/checkpoint-derived-lights-20261124/`.
This is one controlled scenario, not a camera-image or visual-quality test.

## Manual pedestrian experiment

The optional live walker path captures all visible `walker.pedestrian.*`
actors using a caller-provided label-to-actor-ID map, their original spawn
transforms and the experimental velocity RPC helper. Pass `--walkers`,
`--walker-bootstrap-transforms` and `--walker-rpc` to capture. Source and
target must use the same selected-file identity and zero world origin; only
live walkers in Unreal's walking movement mode with CARLA's walker controller
are accepted. Other clients and AI walker controllers must not control them.
Capture records the streamed root transform and native character movement
velocity and observer cache; it does not capture animation, character movement
internals, controller plans, jumps in progress, contacts, or a full scene.
Restore teleports the fresh walker and then writes the saved velocities; a
later failure can leave it partially changed. Actor type and attributes are
checked, but the native velocity probe does not check all movement settings.
This is an experimental restricted solution, not a full pedestrian snapshot.

Build the standalone research client, without a package build:

```bash
g++ -std=c++14 -O2 -pthread -DBOOST_ERROR_CODE_HEADER_ONLY -Ideterministic-cpt/LibCarla/source -IBuild/rpclib-v2.2.1_c5-c10-libstdcxx-install/include -IBuild/boost-1.80.0-c10-install/include scripts/checkpoint_walker_velocity_rpc.cpp Build/rpclib-v2.2.1_c5-c10-libstdcxx-install/lib/librpc.a -o /tmp/checkpoint_walker_velocity_rpc
```

Replay requires `--walker-rpc` and `--walker-actions`. The latter is a JSON
list with one label-to-control map per vehicle action frame, for example
`[{"ped": {"direction": [1, 0, 0], "speed": 1.4, "jump": false}}]`.
Controls are applied before each tick. Walker AI controllers are not
represented. Provide
`--walker-reference` to compare sampled walker positions and velocities.
The report gives maximum Euclidean errors in metres and metres per second;
its default comparison tolerances are 0.01 m and 0.05 m/s, configurable with
`--walker-position-tolerance` and `--walker-velocity-tolerance`. These are
test thresholds, not a general claim of acceptable quality. Without a reference
the tolerance result is null. Orientation, animation and sensor output are
not assessed by this comparison.

In a 2026-11-24 live test, a manually controlled pedestrian changed direction
and briefly stopped during a 120-frame suffix. The pedestrian's maximum
sampled position and velocity errors were both zero in that run; two vehicles,
a dynamic cone, and sampled traffic and vehicle light states also matched.
The other map props were fixed, the pedestrian did not interact with a vehicle,
and the opt-in fixed vehicle sleep setting was used. The combined report is in
`checkpoint-results/20261124-live-vehicles/checkpoint-live-walker-combined-20261124/`.
These observations do not establish behavior for vehicle-pedestrian contacts,
AI pedestrians, jumps, other blueprints, sensor imagery or arbitrary scenarios.

For walker spawn and destruction, `--walker-actions` may instead specify a
lifecycle journal with `actors` and `frames`. `actors` maps new labels to
definitions containing `blueprint`, `attributes`, `spawn_attributes`, and
the six-field `spawn_transform`; captured walker labels already have their
definitions in the checkpoint. Each frame has an ordered `events` list of
`{"action": "spawn" | "destroy", "label": "..."}` records and a
`controls` map covering exactly the walkers alive after those events. Spawned
walkers use their defined initial transform and attributes; they do not inherit
the destroyed walker's saved movement state. Walker events run before vehicle
events and both sets of controls are applied before the tick. References must
contain exactly the live walker labels on each corresponding frame. The whole
journal is checked before loading the replay world, although a failed runtime
spawn or destroy can leave the target episode partially changed.

A 2026-11-24 Town02 comparison exercised two destroy-and-respawn cycles of a
captured pedestrian and the spawn and destruction of a separately labelled
pedestrian, alongside two vehicles with lifecycle events, fixed map props,
traffic-light controllers and derived vehicle lights. For 120 sampled frames,
vehicle and sampled light traces matched exactly, while the maximum pedestrian
position and velocity errors were both zero. This uses manual pedestrian
controls without walker-vehicle contacts and does not validate AI, animation,
or general lifecycle behavior. Evidence is stored in
`checkpoint-results/20261124-live-vehicles/checkpoint-live-walker-lifecycle-20261124/`.

## Other flat maps

Live capture and replay fingerprint the selected local map rather than
assuming Town01. The map name must be a simple asset name and all five
selected map files must be present in the standard flat-map locations.
Sublevels, shared assets, runtime memory and all loaded code are not fully
fingerprinted. Large tiled maps and other layouts need separate support.
Town12 work is deferred at the user's request; current work targets supported
flat maps.
A mismatched selected identity is rejected before the target map is loaded.

In a 2026-11-24 Town02 test, a checkpoint with two manual vehicles, one
pedestrian, portable traffic lights, derived vehicle lights, and 65 fixed map
props reproduced 120 sampled suffix frames, including vehicle lifecycle
events. The sampled pedestrian position and velocity errors were zero. The
prior Town01 identity remained unchanged after the fingerprint update. These
are restricted experiments and do not establish equivalence for all flat
maps or all scenarios. The report is in
`checkpoint-results/20261124-live-vehicles/checkpoint-live-town02-20261124/`.

## Traffic Manager control replay experiment

`scripts/checkpoint_tm_control_probe.py` tests a limited alternative to
restoring Traffic Manager: run it synchronously in the original episode,
observe every vehicle's applied control after each tick, and replay those
controls manually after a fresh-episode vehicle restore. The complete future
control sequence must already be known. This does not resume autonomous
decision-making or restore Traffic Manager routes, random state, registrations,
controller state, or responses to a changed scenario. It also does not record
vehicle light changes, actor lifecycle events, or other controller actions.
The probe uses fixed map props and the opt-in fixed vehicle sleep setting; it
requires a dedicated server and exclusive control of its ticks.

On 2026-11-24, one Town01 vehicle using seed 17 matched the recorded vehicle
state and controls exactly over 120 suffix ticks, including acceleration and
braking. A separate seed 31 test with two vehicles over 200 ticks did not match
exactly: vehicle 0 matched, while vehicle 1 first differed at suffix frame 1.
Its maximum sampled position and velocity differences were approximately
1.15e-7 m and 2.28e-6 m/s, respectively. The vehicles were not set up to
interact, and these comparisons do not establish collision, traffic-light,
sensor, or general scenario equivalence. The stored checkpoints, controls,
reference traces and reports are in `checkpoint-results/20261124-live-vehicles/`
under `traffic-manager-observed-controls-single` and
`traffic-manager-observed-controls-two`.
The user has accepted differences of this observed size for now; this is not a
general numerical tolerance for other scenarios or state fields.

A further Town01 run used two Traffic Manager vehicles initially 12 m apart
in the same lane, disabled their automatic lane changes, and captured portable
traffic-light controller state. Their minimum sampled center-to-center distance
was 6.20 m; both vehicles moved and braked. Replaying their observed controls
matched both vehicles' sampled state and controls exactly for 200 ticks. Eight
sampled traffic lights also matched, including four observed state changes.
This establishes a nearby-vehicle replay case, not that Traffic Manager's
collision-avoidance decisions themselves were restored. The evidence is in
`checkpoint-results/20261124-live-vehicles/traffic-manager-observed-controls-following`.
