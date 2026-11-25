# Rolling checkpoint controlled validation

`checkpoint_rolling_reactive_validation.py` exercises a synthetic trigger in
the restricted reactive Traffic Manager setup. A sole client initializes
Town01 with fixed map props, one manual ego vehicle and one Traffic Manager
vehicle. It records controls and sampled state, maintains the rolling recipe
buffer, and pins the latest checkpoint at least five seconds before the
synthetic trigger. The original continuation is recorded for another five
seconds. On a fresh episode, the script reconstructs the recipe by replaying
its prefix, verifies the boundary, and applies the original controls. It then
restores the reconstructed same-episode slot and runs a changed ego action
sequence twice; Traffic Manager computes new controls in both branches.

This uses prefix reconstruction, not a portable serialization of all TM state.
The saved portable vehicle blobs are not imported or compared in this test.
It has no sensors, agent, ScenarioRunner, walkers, actor lifecycle changes or
collision cases. It uses the opt-in fixed vehicle sleep mode and changes map
props to fixed from the start, which can affect collisions. The synthetic
trigger tests selection timing, not the project's trigger detectors. The same
controls are replayed directly; this does not rerun the original policy model.

On 2026-11-25 two trials used a 1 s checkpoint cadence and 5 s requested
lookback. The trigger occurred after 140 ticks; both selected a checkpoint
5.95 s earlier. Each recorded 100 ticks (5 s) after the trigger, yielding
219 continuation ticks from the checkpoint. The `straight` original used
braking followed by throttle 0.4; `steer` added steering 0.05 to that throttle.
In both, the fresh-episode boundary and all sampled same-control states,
controls and traffic-light fields matched exactly. For the alternative branch,
throttle 0.4 was applied for the whole suffix, twice from the reconstructed
same-episode slot; all sampled fields matched exactly between the two repeats.
Traffic Manager controls differed by up to 1.0 from the original-control
branch and first differed at suffix tick 27; maximum traffic position changes
were 14.00 m and 13.87 m respectively. Minimum sampled vehicle-center
separation in both trials was 8.77 m. These are two closely related scenarios
with no impact; the result does not establish general reactive replay fidelity.

Reports and recipes are in the Zoo checkout's ignored
`outputs/feedback_closed_loop/rolling_checkpoint_20261125/step2/` directory.
Each report contains the source and three branch traces. Capture and restore
are sequential RPCs, not an atomic world checkpoint; a failed restore requires
discarding the episode. Selected artifact hashes do not cover every loaded
asset or runtime byte.

Run on a dedicated custom development server with fixed sleep enabled and the
matching Python API. The probe loads worlds and destroys actors. Choose unused
paths for each output; do not share the server with other work:

```bash
PYTHONDONTWRITEBYTECODE=1 PYTHON_EGG_CACHE=/tmp/carla-egg-cache \
PYTHONPATH=deterministic-cpt/PythonAPI/carla/dist/carla-0.9.15-py3.8-linux-x86_64.egg \
.venv/bin/python deterministic-cpt/checkpoint-workspace/scripts/checkpoint_rolling_reactive_validation.py \
  --port 43000 --tm-port 9300 --variant straight \
  --recipe /tmp/rolling-controlled-recipe.json --report /tmp/rolling-controlled-report.json
```

Use `--variant steer` and new output paths for the steering case.
