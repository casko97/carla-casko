# Server-side checkpoint boundary observation

The scripted two-vehicle bundle can recapture each vehicle's portable physics
fields immediately after importing and restoring it. It compares the MessagePack
encoding field by field, excluding the capture frame, and reinstalls the original
checkpoint slot. This uses sequential RPCs and requires a single tick owner.
It does not inspect fields omitted from the vehicle checkpoint, the PhysX scene,
sensor state, or other controllers. Encoded equality is build-specific.

On 2026-11-24, a Town01 capture and a fresh-map restore using the local
development server matched all sampled vehicle states and controls for the
40-frame suffix, including the scripted destruction and respawn. The server
recapture matched every encoded field for the stationary vehicle. For the
moving vehicle only `rotation_quaternion` differed: the largest component
difference was approximately 5.96e-8. The recorded native PhysX body pose,
velocities and the other encoded fields matched. This does not establish the
cause of the quaternion difference or equality of the full simulator state.

The client world snapshot immediately after the restore still reflected the
last streamed frame, so its mismatch is reported separately and cannot verify
the server state before a tick. A durable next step is to inspect whether the
Unreal component quaternion undergoes a conversion or normalization on restore,
and to expand the boundary observations to other simulator components.

## Variable capture boundaries

The version 3 scripted bundle accepts `--checkpoint-after` from 1 through 79,
counting completed journal frames. It saves the actors alive at that boundary
and replays subsequent scripted spawn and destruction events. Version 2 bundles
remain readable at their fixed frame 40 boundary. This is still a fixed Town01
scenario; the journal and supported actor definitions are not yet generic.

In local development-server experiments on 2026-11-24, capture after frame 20
(before the other vehicle spawned) reproduced all 60 sampled suffix frames.
Capture after frame 61 (after its destruction) reproduced all 19 sampled suffix
frames, including the later respawn. Encoded portable vehicle fields matched at
both restored boundaries, excluding the frame number.

Capture after frame 21, one frame after the other vehicle spawned, did not
reproduce the baseline. Two independent restores yielded identical sampled
suffixes and both first diverged at suffix frame 3, despite matching all encoded
vehicle checkpoint fields at the boundary. The ego vehicle's first differing
position component was approximately 0.00038 m; its first differing vertical
velocity component was approximately 0.00682 m/s. A separate full-prefix run
matched the original baseline boundary and all 59 suffix frames. This points
to restore-related state omitted from the current vehicle records in this case;
it does not identify the component responsible. The PhysX contact/solver state
and actor lifecycle state remain candidates, not established causes.

The `checkpoint_same_episode_probe.py` experiment compared the first 10 suffix
frames after frame 21 without a map reload or actor replacement. Two restores
first differed from the uninterrupted baseline on frames 3 and 7 respectively;
the encoded portable fields matched at each restored boundary. The first
same-episode replay exactly matched the first 10 frames of the earlier fresh-map
replay. Repeating the experiment after removing the second vehicle produced the
same ego baseline and the same ego traces for both restores. Thus actor
reconstruction and the other vehicle are not necessary to reproduce this
divergence. These observations do not establish which unrecorded state causes it.

Further controlled probes used the same frame 21 checkpoint and the same
10-frame suffix. An immediate restore, without advancing the world after save,
matched the suffix. After advancing 3 or 5 frames and then restoring, the first
sampled difference occurred at suffix frame 7; after advancing 10 frames, it
occurred at suffix frame 3. All recorded portable vehicle fields matched at
the restored boundary in these runs. This identifies a dependency on state that
changes as the world advances, but does not identify that state.

A separate same-episode experiment raised the lone ego vehicle's spawn point
100 m. Both restores matched their 10-frame suffix exactly; sampled vehicle
altitude during that suffix ranged from about 88.45 m to 94.32 m. This is
consistent with a road-contact-related explanation, but the airborne and road
runs have different trajectories and contact events were not instrumented, so
the experiment does not establish causation. The engine's vehicle manager
normally issues suspension queries before its vehicle update; PhysX also has
cached query data and vehicle constraints, while the checkpoint does not save
the PhysX scene's contact or solver state. Those are candidates for further
instrumentation, not confirmed causes.

## Wake counter and collision diagnostics

With a collision sensor attached to the lone ego vehicle, the ground and
airborne probes recorded no collision events in the prefix or the tested
suffixes. The ground baseline and both restored trajectories matched their
corresponding runs without the sensor. CARLA's collision sensor does not
report all PhysX contacts or suspension raycast hits, so zero reported events
does not rule out road interaction.

In a separate ground run the probe recaptured the portable physics fields
after each suffix tick, reinstalling the original slot after each observation.
The baseline and restored trajectories exactly matched the earlier
uninstrumented ground run. On both restores the first recorded physics difference
was `wake_counter` at suffix frame 2, before the first reported trajectory
difference at frame 3 or 7. The baseline counter at frame 2 was approximately
0.205377 seconds; both restored runs had approximately 0.039381 seconds.

PhysX 3.4's `DyBodyCoreIntegrator.h`, in `updateWakeCounter`, uses accumulated
linear and angular sleep velocities and interaction counts when deciding how to
reset the wake counter. The vehicle checkpoint saves the current counter but
does not capture those accumulators or the full interaction state. Their
involvement is a testable hypothesis, not yet a demonstrated cause; the public
vehicle fields and collision events alone cannot show their values. Capturing
or instrumenting those engine internals requires build-specific changes and
careful handling of the physics thread and actor lifetime.

## Fixed sleep-threshold diagnostic

The first diagnostic build used `-CheckpointNoSleepOnSave` to change the
threshold at save time. That flag has been removed; the following results
describe that earlier experiment. The current opt-in mode is described below.

Using the same on-disk editor build, one vehicle, frame 21 and a 30-frame suffix,
the default-setting restores first differed at suffix frames 3 and 7. With the
flag enabled, both restored trajectories and all recorded portable physics
fields matched the 30-frame baseline exactly. These are two comparisons in one
scenario and do not establish determinism generally or prove which omitted
sleep state caused the original mismatch.

This intervention also changed the uninterrupted baseline: the first sampled
difference from the default setting was suffix frame 7. Across the 30 frames,
the largest absolute component differences were approximately 0.01172 m in
position, 0.07731 m/s in velocity, 1.42442 degrees/s in angular velocity, and
2.11496 m/s² in acceleration. These figures are specific to this early
low-speed scenario; they do not establish whether the quality tradeoff is
acceptable for driving, collisions, pedestrians, or benchmark evaluation.
Before using a fixed property as a complementary solution, configure it
consistently from scenario initialization, include it in the manifest and
compatibility checks, and evaluate its effect on representative scenarios.

The current `-CheckpointFixedNoSleep` mode applies the zero threshold at vehicle
initialization and after the known CARLA vehicle physics recreation paths. A
checkpoint save rejects the vehicle if the threshold is not zero; the
same-episode slot also checks the saved threshold on restore. The portable
vehicle export appends a mode marker to its caller-provided build ID and the
server checks that marker during import. New scripted bundles expose the mode
as `checkpoint_mode`, and their full-prefix replay verifies the mode before
running. This marker relies on the caller supplying the same build ID; it is not
a cryptographic fingerprint or a general record of every physics setting. The
mode is opt-in and remains an experimental change to physics.

In a fixed-mode frame 21 test, a fresh-episode restore reproduced all 59
sampled suffix frames, and two same-episode restores reproduced 30 frames and
their recorded portable physics fields. A full-prefix replay also matched the
boundary and 59 suffix frames. On the same editor build, fixed-to-default and
default-to-fixed imports were each rejected. In a paired fixed/default
uninterrupted run, the first sampled suffix difference occurred at frame 7;
the largest ego position component difference over 59 frames was about
0.05112 m and the largest velocity component difference was 0.07731 m/s.
These results do not establish acceptable simulation quality or deterministic
behavior in arbitrary scenarios, and the setting is not enabled by default.
An additional fixed-mode probe saved successfully after applying vehicle physics
control, which recreates the vehicle physics state. This checks that one
recreation path reapplies the setting; it does not establish coverage of every
possible external component rebuild or vehicle type.

## Collision and Traffic Manager comparisons

On the same editor build, paired default and fixed-sleep runs used the supplied
Town01 collision journal (150 frames, checkpoint after frame 60). Both runs
matched their two same-episode restores and full-prefix replay for the sampled
vehicle fields and diagnostic collision events. Across the uninterrupted runs,
the first sampled trajectory difference was frame 28. The largest position
component differences were about 0.0120 m for the moving vehicle and 0.0107 m
for the parked vehicle; the largest velocity component differences were about
0.3396 m/s and 0.1114 m/s respectively. The collision sensor recorded 174 and
176 event records respectively, counting reports for both vehicles; these are
not counts of unique physical contacts. Their first recorded collision was at
frame 60 in both runs. The records are diagnostic and omit some contacts.

A paired 100-frame Town01 run with Traffic Manager seed 17, traffic lights,
weather and vehicle lights used a checkpoint after frame 40. Both settings
matched two same-episode restores and full-prefix replay for the sampled fields.
Across the uninterrupted runs, the first vehicle-state difference was frame
28 and the first observed Traffic Manager control difference was frame 93;
the sampled light and weather traces matched. The largest position component
difference was about 0.00995 m for the manual vehicle and 0.0000611 m for the
Traffic Manager vehicle. These scenarios do not by themselves establish
acceptable quality or coverage of pedestrians, sensors or arbitrary contacts.

The four original reports and machine-readable comparisons are in
`checkpoint-results/20261124-fixed-sleep/`. The mode labels come from the
server launch commands used for these runs; the underlying multi-vehicle
reports do not independently prove their launch flags or executable identity.
`scripts/compare_checkpoint_experiments.py` compares their sampled traces and
records SHA-256 hashes of its inputs.

`checkpoint_scenario_bundle.py replay-prefix --checkpoint BUNDLE --report REPORT`
now reconstructs the fixed script from its start and compares its boundary and
suffix with the bundle. In the frame 21 case it matched both the boundary and
all 59 sampled suffix frames. The prefix is the program's fixed schedule, not a
general recorded scenario journal; it is a reference and fallback only for that
specific script and compatible environment. The comparison still does not cover
the full simulator state.
