# Manual vehicle journal checkpoints

`checkpoint_journal_bundle.py` captures and replays a caller-provided journal
of manually controlled vehicles. It is an experimental step toward a scenario
checkpoint, not a complete simulator checkpoint. Run it only on a dedicated
server built from `deterministic-cpt`; it loads a map and destroys its actors.
Do not use a package build.

The JSON format is illustrated in `checkpoint_journal_example.json`. It has a
locally fingerprintable flat map, a list of vehicle definitions accepted by the multi-vehicle
harness, an ordered list of initial actor IDs, and frame records. Every frame
lists ordered spawn/destroy events and exactly one control for every actor
alive after those events. Vehicle definitions may use a spawn index or explicit
transform and supported blueprint attributes. The checkpoint boundary counts
completed frames. At least one actor must remain alive in every frame. The
journal is validated before the map is loaded.

From the main workspace, using the Python 3.8 environment and the matching
client egg, for example:

```bash
export PYTHON_EGG_CACHE=/tmp/carla-egg-cache
export PYTHONPATH=deterministic-cpt/PythonAPI/carla/dist/carla-0.9.15-py3.8-linux-x86_64.egg
.venv/bin/python scripts/checkpoint_journal_bundle.py capture --journal scripts/checkpoint_journal_example.json --checkpoint /tmp/example-journal-checkpoint.json --checkpoint-after 2 --port 23000
.venv/bin/python scripts/checkpoint_journal_bundle.py restore --checkpoint /tmp/example-journal-checkpoint.json --report /tmp/example-journal-restore.json --port 23000
.venv/bin/python scripts/checkpoint_journal_bundle.py replay-prefix --checkpoint /tmp/example-journal-checkpoint.json --report /tmp/example-journal-prefix.json --port 23000
```

Capture loads Town01, creates the journal's initial actors and executes the
prefix before saving. It does not attach to an already-running scenario or
record that scenario's commands. The bundle includes the full journal, selected on-disk artifact hashes,
checkpoint mode, actor attributes and portable vehicle slots, a boundary
observation, and the expected sampled suffix. The selected hashes do not cover
all assets, loaded server code or runtime memory. The restore starts a fresh
episode and spawns live vehicles in sorted ID order; a change in actor ordering,
contact history, untracked actors or other omitted state may cause divergence.
The saved origin is assumed to be zero, not independently verified. Sequential
captures and restores require one tick owner and are not atomic. Comparisons
cover sampled transforms, velocities, accelerations, and observed controls.
They do not prove equality of all physics state or sensor output.

The tool accepts maps with the selected local flat-map artifacts described in
`checkpoint_live_vehicles.md`; large tiled maps are not supported. It supports manually controlled vehicles only: pedestrians,
Traffic Manager, traffic-light controller state, weather changes, sensors,
ScenarioRunner state, arbitrary external actions and PhysX scene caches are
not captured. Prefix replay uses the supplied journal as a reference and can
reconstruct only the behavior actually represented in that journal. The
fixed-sleep mode remains opt-in and changes simulated behavior.

On 2026-11-24, a custom 80-frame journal with spawn, destruction and respawn
was tested with the experimental fixed-sleep mode. A frame 21 checkpoint
restored into a fresh episode and matched all 59 sampled suffix frames; full
prefix replay matched the boundary and suffix. A frame 61 checkpoint, after
the second vehicle was destroyed, restored and matched 19 suffix frames,
including its later respawn. The results are in
`checkpoint-results/20261124-journal/`. These are two restricted scenarios,
not evidence of arbitrary-scenario reproducibility.
