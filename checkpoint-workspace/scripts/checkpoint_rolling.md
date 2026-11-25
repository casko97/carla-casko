# Rolling reactive checkpoint recipes

`checkpoint_rolling.py` adds a bounded buffer for the existing restricted
Traffic Manager prefix-reconstruction workflow. It stores immutable copies of
validated reconstruction recipes, including every ego control since setup,
and portable native vehicle physics checkpoints exported at the same frame.
It does **not** store independent native Traffic Manager or server slots: the
native slot is still overwritten by each capture. A selected recipe can be
reconstructed by `restore_reactive_traffic` on a fresh compatible server by
replaying its prefix. The existing reconstruction routine does not yet import
or compare the additional vehicle blobs; they are preserved for the next
integration experiment. Prefix replay takes time proportional to the prefix and is not a
full snapshot of arbitrary simulator or controller state.

Create `ReactivePrefixBuffer` immediately after the supported reactive scenario
setup, before its first controlled tick. The caller must be the sole tick and
mutation owner. After **each** tick, call `observe_reactive` with the exact ego
control applied before that tick, the client, TM, actor mapping, fixed-prop
guard, actor specifications, seed, initial weather and TM configuration used at
setup. It captures on the first observed tick and then at the configured
simulation-time interval. Calls between captures still journal controls; missed
ticks, episode changes, prefix limits, capture failures and changes of frame
during capture invalidate further observations. Already captured recipe bytes
remain readable, but validity of the failed scenario is the caller's decision.

The current backend requires Town01/Town02, one manual `expert`, selected TM
vehicles, fixed weather and map props, no sensors, walkers, lifecycle changes,
recent collisions or uncontrolled actors, and the other restrictions described
in `checkpoint_tm_reconstruction_api.md`. Supplying actions is a caller
attestation: this buffer cannot detect unrecorded changes or prove hidden TM
state or policy context are equivalent. Camera and Bench2Drive support are
future work. The buffered recipe must be revalidated by the reconstruction
backend; continuation comparisons are essential.

`buffer.select_and_pin(trigger_frame, trigger_seconds)` returns the newest
entry at or before `trigger_seconds - before_seconds` whose frame is no later
than the trigger. Defaults are 1-second capture interval, 5-second lookback,
16 entries and 128 MiB of encoded recipes. Thus an on-time selection is
normally 5–6 simulated seconds old. An early or late trigger with insufficient
retained history raises `LookupError`; a not-yet-observed trigger is rejected.
Each selection increments a reference count. Call `buffer.release(entry)`
once per selection after durable publication or cancellation. Pinned entries
cannot be evicted, and exhaustion fails explicitly without discarding them.
The byte limit counts encoded recipes, not Python object overhead. Prefix
journaling is separately limited to 10,000 frames and 8 MiB by default;
reaching either limit fails instead of silently truncating the required prefix.

Each frozen entry has `frame`, `elapsed_seconds`, `episode_id`, `sha256`, and
`recipe_bytes`; `recipe()` returns a new dictionary. Recipes can be published
through the existing `checkpoint_scenario_bundle.atomic_create(path,
entry.recipe())` and consumed by `checkpoint_live_vehicles.restore_reactive_traffic`.
The selected entry is an in-memory object until explicitly persisted and does
not survive a client crash. Existing exclusive-create files may remain partial
on interruption; the integrating pipeline must validate completed artifacts.
Do not interpret a matching boundary as proof of a full simulator restore.

Run the pure buffer tests from this directory with
`python -m unittest -v test_checkpoint_rolling`.

An initial live Town01 capture-only smoke run with recipe retention (before
vehicle blobs were added) observed no capture ticks and preserved a pinned
recipe selected 5.95 seconds before the synthetic trigger through 80 later
ticks. The updated smoke test also checks portable vehicle export and payload
digests. Its 2026-11-25 run was stopped after the development server remained
in initial startup for several minutes; the updated capture has therefore not
yet been verified against a live server at that point.

## Same-controls reconstruction (2026-11-25)

A subsequent Town01 controlled test captured a periodic recipe containing
native vehicle blobs, pinned the recipe selected 5.95 s before the synthetic
trigger, continued the source for 199 suffix ticks, reconstructed a fresh
episode by replaying the recipe prefix, and applied the same suffix controls.
The expert held the brake before the trigger and then accelerated. There were
two vehicles; Traffic Manager controlled the following vehicle. The selected
boundary and all sampled suffix fields matched exactly, including vehicle
position, rotation, linear and angular velocity, acceleration, applied
controls, separation, and 36 traffic-light observations. Maximum sampled
position and velocity errors were zero at the API's precision. This was one
restricted no-sensor, fixed-prop, no-ScenarioRunner test, not a claim of
full-world identity. Reconstruction used prefix replay; the newly exported
vehicle blobs were saved but not independently imported or compared.

The new server launch for a second steering variant timed out on its initial
`get_world()` call before simulation. A separate earlier same-episode test is
available in `checkpoint-results/20261125-tm-reactive/same-episode-coordinator.json`:
its `move` and `move_repeat` sampled traces match for 120 ticks while TM is
active. It uses a native same-episode slot rather than this rolling buffer.

The first side-by-side video was rendered later from the saved pose traces in
a packaged CARLA server, using fixed cameras and recolored vehicles. The
second is a schematic rendered from the stored actor states after the CARLA
rendering server timed out. Neither is camera output captured during the
checkpoint simulations. They are stored in the Zoo checkout's ignored `outputs/feedback_closed_loop/
rolling_checkpoint_20261125/` directory, with a copy of each comparison.

