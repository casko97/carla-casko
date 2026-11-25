# Reactive Traffic Manager reconstruction

`checkpoint_live_vehicles.capture_reactive_traffic` records a recipe at a live
checkpoint and returns a same-episode slot. `restore_reactive_traffic` loads that
recipe into a fresh episode, replays the recorded prefix, verifies the sampled
boundary, and returns `(world, actors, traffic_manager, slot, recipe)`. The caller
owns the returned session, supplies expert controls, and must unregister each
Traffic Manager actor and clean up the actors when finished. The slot supports
repeatable branching within that episode.

This is prefix reconstruction, not serialization of Traffic Manager internals.
The caller must initialize the source with `checkpoint_tm_prefix_reconstruction.setup`,
record every expert action from initialization, preserve the seed, actor spawn
order, and per-vehicle Traffic Manager configuration, and ensure it is the sole
tick and mutation owner. The live API trusts this declared history: a matching
sampled boundary does not prove that all hidden controller state matches. Compare
continuations against the source for the scenario in use.

Supported configuration is one manual actor named `expert` and one or more
Traffic Manager vehicles, with per-vehicle `auto_lane_change`,
`distance_to_leading_vehicle`, and optional `speed_difference`. Route/path
overrides and other Traffic Manager settings are not recorded. Map, build and
asset fingerprints must match. The supported maps are Town01 and Town02;
Town12 is excluded. Weather must remain fixed, map props are frozen, and
sensors, walkers, dynamic props, actor lifecycle changes and active or recent
collisions are outside this workflow. Both capture and reconstruction need a
dedicated synchronous server; reconstructing replaces its episode. Replaying a
long prefix costs time proportional to its length.

The earlier two-vehicle recipes without an explicit configuration remain
supported. `checkpoint_tm_live_multi_probe.py` exercises the main API with two
independently configured traffic vehicles and compares repeated and
counterfactual expert actions against the original continuation.
