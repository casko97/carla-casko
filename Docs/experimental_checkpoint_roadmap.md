# Toward a scenario checkpoint

The current prototype is a set of same-episode, in-memory component slots, not
a serialized world checkpoint. The vehicle, traffic-light and weather slots
have not been validated in a live simulator. A successful harness comparison
would establish equality only for the sampled fields and chosen scenario; it
would not establish equality of all internal state or sensor output.

The intended contract is to capture an arbitrary **completed frame boundary**,
start a compatible fresh episode, reconstruct the world, and run a recorded
suffix with the same results. Define the outputs and equality tolerance for
each experiment first: physical trajectories, collision events, traffic
behavior and sensor images may have different determinism limits. This contract
has not been achieved or shown to be achievable bit-for-bit.

## Required increments

1. Introduce a world-wide quiescence protocol: one tick owner, block new
   commands, finish the physics step and component work, drain or account for
   in-flight sensor and client messages, and save a consistent frame and
   simulation time. Restore should validate every component before mutation,
   order dependent components explicitly, and fail closed if a component cannot
   be restored. Existing sequential RPCs and frame-number checks do not provide
   this guarantee.
2. Define a versioned, portable checkpoint format with build, engine, map and
   content fingerprints, fixed-step and physics settings, persistent actor IDs,
   actor types, blueprint attributes, attachments, lifecycle and component
   records. Map checkpoint IDs to newly spawned runtime IDs, including references
   held by controllers. Make vehicle/light/weather state serializable without
   raw pointers and reconstruct it in a fresh episode. Capture actor creation,
   destruction and external commands after the checkpoint in the journal.
   `PortableVehicleCheckpoint` defines a versioned, pointer-free representation
   of the current vehicle slot with explicit native units and structural checks.
   The native vehicle can now export its current in-memory slot into that value
   representation, including the capture frame and active PhysX flags. The
   Python vehicle API can export that record to MessagePack bytes, and the
   single-vehicle harness can write it to a file. Import and fresh-episode
   restoration are not implemented. An
   importer must validate all numeric ranges and build, map, blueprint and actor
   compatibility before changing the simulator; the schema alone is not proof of
   reproducibility or cross-build compatibility.
3. Complete and validate physics scene restoration. The present vehicle slot
   saves selected vehicle and rigid-body dynamics but not PhysX contact
   manifolds, constraint/solver warm-start state, broadphase and ordering, or
   other actors and joints. Inspect the actual PhysX version and engine source
   for a supported scene snapshot or implement a controlled reconstruction;
   test contact, collisions, sleep/wake, substeps and repeated restoration.
   Public transforms and velocities alone are not evidence of equal future
   dynamics. Exact reproduction may require changes inside PhysX or a narrower
   supported scenario class.
4. Add Traffic Manager under the same barrier, including registration and
   parameters, waypoint buffers, stage and PID histories, junction and collision
   tracking, simulation state and the **current** RNG state. Its stages share
   `RandomGenerator` in `LibCarla/source/carla/trafficmanager`; reapplying the
   initial seed at a later frame does not restore its sequence position. Include
   walker navigation, controllers and their random state separately.
   `RandomGenerator::SerializeState` and `RestoreState` now provide a versioned
   serialization building block for its engine and distribution. They require
   the caller to stop concurrent draws and reseeding; they are not wired into
   the Traffic Manager API or a worker barrier and do not capture any other TM
   state. Treat the stream as tied to a compatible C++ library and build until
   cross-build compatibility is explicitly established.
   `MotionPlanStage::CaptureCheckpoint` / `RestoreCheckpoint` copy PID history,
   teleportation timestamps and the stage's current timestamp. These are
   same-process building blocks, not exposed through Traffic Manager; a future
   coordinator must provide worker quiescence, stable actor identities and a
   matching or correctly rebased simulation clock. No such clock restore is
   currently implemented, and other Traffic Manager state is still missing.
   `TrafficLightStage` separately captures junction arrival queues, actor-to-
   junction associations and stop timestamps under the same restrictions;
   this is distinct from the server's traffic-light controller checkpoint.
   `LocalizationStage` can copy its lane-change and junction history, but the
   snapshot contains waypoint pointers valid only for the same local map. It
   does not contain the separately owned route buffers and traffic tracker.
   `CollisionStage` can copy collision locks and its per-cycle geometry caches;
   this is traffic-decision state, not the PhysX scene's contact or solver state.
   The actor lifecycle manager has a separate in-memory snapshot of its actor
   handles, idle timers, last destruction time and physics flags. Those handles
   cannot be transferred across episodes. Its referenced actor registration,
   waypoint buffers, tracker and other stage state must be handled separately.
   `checkpoint_time::Rebase` can shift saved motion, junction and lifecycle
   timestamps to a later same-episode frame while preserving their elapsed ages.
   It rejects invalid or future history without modifying the supplied snapshots.
   It is not wired into a coordinator, does not restore the simulator clock,
   and floating-point rebasing is not guaranteed to preserve bitwise results.
   `Parameters` can copy its configuration, pending lane changes and custom
   route/path commands, with a deep copy of collision-ignore sets. Capturing
   individual maps under locks does not make the aggregate snapshot atomic:
   all worker updates and external parameter setters must be excluded across
   the whole operation. Its actor handles remain tied to the current episode.
   `TrafficManagerLocal` now offers internal aggregate capture and restore
   methods for these snapshots, waypoint buffers, the tracker, simulation cache
   and actor lifecycle. They validate episode, local map, settings, actor IDs,
   registration state and saved random state, and rebase the component timers.
   Experimental `TrafficManager.save_checkpoint()` and `restore_checkpoint()`
   expose a single in-memory slot through local or remote TM clients. A failed
   save or TM stop invalidates the slot; it cannot survive an episode or process
   change. These methods are not a world checkpoint. An experimental TM mutex excludes its worker cycle
   including command submission, setters and registration during each operation,
   and the methods reject unless the synchronous worker is waiting for its next
   tick. A caller must still ensure no other client ticks or changes actors,
   and no TM start, stop or reset occurs during the entire operation. Frame
   checks alone do not fence the server or other clients. The worker-lock
   changes require live concurrency and deadlock testing before production use.
   Restore is not transactional if a later allocation or external action fails.
   Physical actors and pending client/server side effects
   still need separate restoration. No live CARLA validation has been performed.
5. Capture ScenarioRunner and agent state outside CARLA: behavior trees,
   criteria, timers, route progress, Python and other RNGs, policy recurrent
   state, and pending commands. Alternatively record all relevant external
   actions and lifecycle events, in which case replaying those commands tests a
   different contract from resuming the original controllers.
6. Add sensors individually, including configuration, tick phase, random
   generator state, history (for example IMU or event-camera history), pending
   asynchronous outputs and sensor-to-frame association. Camera and GPU
   rendering state, temporal history and driver/hardware differences may limit
   pixel equality even when physical state matches. Sensor RNG use is visible
   in `Unreal/CarlaUE4/Plugins/Carla/Source/Carla/Sensor`.
7. Extend the harness to restore into a new episode/process, sample immediately
   after restore and every suffix frame, compare actor lifecycle, physics,
   traffic lights, weather, collision events and per-sensor frame data, and
   report first divergence and numerical differences. Exercise randomized
   checkpoint frames, contact-heavy cases, repeated restores and process restarts
   across fixed builds. Continue comparing a full prefix replay separately.

The collision sensor now has an opt-in bounded server-side diagnostic log of
its de-duplicated hit events, exposed through synchronous world RPCs and an
optional harness trace. It does not capture every PhysX contact. Event timing and the game-thread
ordering must be validated in a live simulator before treating a drain as a
complete per-frame record.

CARLA's recorder/replayer code is useful for actor event and identity mapping,
but replaying recorded positions and events is not itself a snapshot of all
physics, controller, sensor and random state needed for a new physics rollout.
Until these components and the cross-episode barrier exist, use the current
harness only as a controlled research prototype on a dedicated server.
