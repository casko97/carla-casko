# Checkpoint development tests, 2026-11-24

These tests used the existing `deterministic-cpt` development server with
uncooked local assets and `-nullrhi`. The client and server reported
`0.9.15-dirty`; the caller-supplied build ID was a label, not a verified
binary or asset fingerprint. All comparisons below are exact equality of the
sampled values, not bitwise equality of the entire simulator state.

* `checkpoint-dev-test-report.json`: one Town01 vehicle, straight driving,
  two same-episode restores, full reload/replay and portable import into a
  new vehicle after a map reload. The 80-frame sampled suffix matched.
* `checkpoint-maneuver-portable-report.json`: the same test with steering,
  braking and manual gear changes. Two same-episode restores, full replay and
  one portable import matched the sampled 80-frame suffix.
* `checkpoint-collision-dev-report.json`: two controlled vehicles, checkpoint
  after frame 60 and 90-frame suffix. Both restores and full replay matched
  sampled vehicle states and collision diagnostics. The baseline suffix had
  172 collision-event records across the two sensors. These records are
  diagnostic and are not a complete PhysX contact log.
* `checkpoint-traffic-dev-report.json`: two vehicles, one Traffic Manager
  controlled, seed 0, checkpoint after frame 40 and two restores. Sampled
  vehicle states, exposed controls, traffic lights, vehicle lights and weather
  matched the baseline and full replay.
* `checkpoint-traffic-seed17-dev-report.json`: the same scenario with seed 17,
  checkpoint after frame 60, three restores and collision diagnostics.
  Sampled values matched; there were no collision records in the prefix or
  suffix, so this is not a Traffic Manager collision test.

The multi-vehicle checkpoints were sequential, same-episode in-memory slots;
only the single-vehicle test exercised import after reloading a map. These tests
do not establish a portable world checkpoint, atomicity across clients, sensor
or camera equality, pedestrians, ScenarioRunner or agent state, actor lifecycle
reconstruction, or a complete PhysX solver/contact snapshot. The server log
reported missing pedestrian assets and building Blueprint errors. Further work
should preserve a full-prefix replay as a reference and add one subsystem at a
time, starting with stable build/asset fingerprints and a controlled scenario
with independently recorded lifecycle and controller state.
