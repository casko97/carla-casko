# Live camera checkpoint experiment

On 2026-11-25 a custom CARLA 0.9.15 development server ran with RGB rendering
enabled on Town01. In a controlled scenario, one manual vehicle was followed
by one Traffic Manager vehicle. After 60 brake ticks, the experimental
same-episode coordinator captured vehicle, traffic-light and Traffic Manager
slots. An overhead RGB camera was spawned after capture, and 120 ticks of
throttle 0.4 were recorded. The camera was stopped and destroyed, one cleanup
tick was advanced to refresh CARLA's actor snapshot, and the coordinator
restored the slots. A new camera at the same transform then captured the same
120 commands. Simulation time was not rewound. Map props were fixed, vehicle
sleep used the experimental fixed setting, and no ScenarioRunner or policy
model was present.

Both MP4s contain actual camera images generated during their respective
simulations; they are not pose playback. In both, all 120 sensor frame numbers
matched their corresponding world frame. The sampled position, rotation,
velocity, angular velocity, acceleration, controls, traffic-light values and
vehicle separation matched exactly between branches. All raw camera frame
hashes differed; no cause has been established and camera-image equality is
not claimed. The per-frame hashes and sampled traces are in the result JSON.
The renderer used the process-local
`r.GenerateMeshDistanceFields=False` workaround, which can change lighting.

Both native CARLA recorder files were enabled with additional data and checked
using `show_recorder_file_info`: each reported 120 frames and 5.95 s duration.
The MP4s contain 120 frames at 20 fps, or 6.0 s. Native recorder playback
does not capture the original RGB images; the separate MP4s do.

Artifacts are in the Zoo checkout's ignored `outputs/feedback_closed_loop/
rolling_checkpoint_20261125/live_camera_retry2/` directory. Earlier output in
`live_camera_retry/` contains a valid original video and recorder, but restore
was rejected because the destroyed camera remained in the stale actor snapshot.
The successful retry added the cleanup tick. The extra tick happens before the
restore and thus does not enter the compared suffix, but may influence hidden
state not included in the coordinator. This result does not establish a
general live checkpoint, sensor-state or visual determinism guarantee.

To reproduce, start a **dedicated** renderer-enabled checkpoint server with
the matching Python API. The probe reloads maps, changes world settings, creates
and destroys actors and writes CARLA recorder files. From the parent source
workspace, use a new output directory:

```bash
PYTHON_EGG_CACHE=/tmp/carla-egg-cache \
PYTHONPATH=deterministic-cpt/PythonAPI/carla/dist/carla-0.9.15-py3.8-linux-x86_64.egg \
/home/casko/phd-code/Bench2DriveZoo/b2d_zoo.venv/bin/python \
deterministic-cpt/checkpoint-workspace/scripts/checkpoint_live_camera_probe.py \
  --port 41000 --tm-port 9100 --output /tmp/checkpoint-live-camera-new
```
