# Reactive Traffic Manager checkpoint in one episode

`checkpoint_tm_same_episode.py` provides an experimental in-memory coordinator
for branching from a checkpoint while Traffic Manager continues making new
decisions. It saves the selected vehicle physics slots, traffic-light slots,
and the existing Traffic Manager slot at one synchronous frame. Restore
reinstates those slots before the caller applies the next expert action.

This is a **same-episode** facility. It requires a dedicated compatible server,
one tick owner, a synchronous fixed-step world with the harness settings, fixed
map props prepared at scenario initialization, and a Traffic Manager in
synchronous mode. The selected vehicles must cover all visible vehicles; actor
identities and the episode must remain unchanged. Walkers, AI walker
controllers and sensors are outside this coordinator. Weather must remain at
the captured value. Other clients must not tick, alter actors or settings, or
overwrite any vehicle, light or Traffic Manager checkpoint slots. The fixed
prop guard cannot independently read back the physics-disable flag.

Capture and restore use sequential RPCs, not an atomic world transaction. If a
step fails, the coordinator becomes unusable; some slots or actors may already
have changed. The caller must discard or reconstruct the episode after a
partial failure. World time is not rewound; the experimental Traffic Manager
slot rebases its selected internal timestamps to the current frame. No claim is
made for unrecorded scenario controllers, pending commands, actor lifecycle,
contact/solver caches, or fresh-episode or cross-process restore.

For a prepared world and registered Traffic Manager vehicles:

```python
from checkpoint_tm_same_episode import SameEpisodeTrafficCheckpoint

checkpoint = SameEpisodeTrafficCheckpoint(client, world, tm, actors, fixed_props_guard)
checkpoint.capture()  # immediately after a completed tick
# Advance the world with one expert action sequence.
checkpoint.restore()  # same episode, same actors, no concurrent ticks
# Advance using a different expert action sequence; TM computes new controls.
```

## Branching experiment

On 2026-11-25 a Town01 experiment placed a manually controlled expert ahead of
a Traffic Manager vehicle, disabled automatic lane changes, and captured after
60 ticks. The expert then either held the brake or accelerated for 120 ticks
(6 seconds). Each action sequence was run twice from the same saved state.
The complete sampled vehicle state and Traffic Manager control traces matched
exactly within each repeated pair. When the expert held still, the traffic car
remained braking. When the expert moved, the traffic car first released its
brake on suffix tick 52 and reached 3.63 m/s; its position differed from the
hold branch by up to 5.48 m. The minimum sampled center-to-center separation
was 8.77 m. No sensors or impact cases were used.

The same comparison on Town02 with seed 31 also matched the complete sampled
vehicle and Traffic Manager control traces exactly within each repeated pair.
Traffic first changed its control on suffix tick 28 when the expert moved,
reached 3.69 m/s, and differed from the hold branch by up to 7.53 m. The
minimum sampled center-to-center separation was 10.19 m. Its report is
`checkpoint-results/20261125-tm-reactive/same-episode-town02-seed31.json`.

These are two closely related following setups and one pair of expert action
sequences; they do not establish reactive behavior for arbitrary routes,
traffic density, or controllers. The Town01 report is
`checkpoint-results/20261125-tm-reactive/same-episode-coordinator.json`.
Run the probe on a dedicated server with
`scripts/checkpoint_tm_reactive_probe.py`; it aborts if vehicle centers come
within 6 m. Four focused coordinator tests also cover repeat restore, changed
episode, fixed-weather preflight and partial-failure invalidation.
