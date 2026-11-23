// Copyright (c) 2020 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.
// For a copy, see <https://opensource.org/licenses/MIT>.

#include <algorithm>

#include "carla/Logging.h"

#include "carla/client/detail/Simulator.h"

#include "carla/trafficmanager/TrafficManagerLocal.h"
#include "carla/trafficmanager/CheckpointTime.h"

namespace carla {
namespace traffic_manager {

using namespace constants::FrameMemory;

namespace {
std::vector<ActorId> GetWorldActorIds(const cc::World &world) {
  std::vector<ActorId> ids;
  const auto actors = world.GetActors();
  for (const auto &actor : *actors) ids.push_back(actor->GetId());
  std::sort(ids.begin(), ids.end());
  return ids;
}
}

bool TrafficManagerLocal::CaptureCheckpointForCoordinator(Checkpoint &checkpoint) {
  std::lock_guard<std::mutex> lock(registration_mutex);
  const auto settings = world.GetSettings();
  if (!parameters.GetSynchronousMode() || !settings.synchronous_mode ||
      !settings.fixed_delta_seconds || step_begin.load() || step_end.load() || !local_map)
    return false;
  Checkpoint state;
  state.episode_id = world.GetId();
  state.map = local_map;
  state.settings = settings;
  state.timestamp = world.GetSnapshot().GetTimestamp();
  state.world_actor_ids = GetWorldActorIds(world);
  state.registered_actor_ids = registered_vehicles.GetIDList();
  state.registered_state = registered_vehicles.GetState();
  state.last_processed_registered_state = registered_vehicles_state;
  state.vehicle_ids = vehicle_id_list;
  state.buffers = buffer_map;
  state.tracker = track_traffic;
  state.simulation = simulation_state;
  state.parameters = parameters.CaptureCheckpoint();
  state.random_state = random_device.SerializeState();
  state.seed = seed;
  state.localization = localization_stage.CaptureCheckpoint();
  state.collision = collision_stage.CaptureCheckpoint();
  state.lights = traffic_light_stage.CaptureCheckpoint();
  state.motion = motion_plan_stage.CaptureCheckpoint();
  state.lifecycle = alsm.CaptureCheckpoint();
  state.marked_for_removal = marked_for_removal;
  if (world.GetSnapshot().GetTimestamp().frame != state.timestamp.frame) return false;
  checkpoint = std::move(state);
  return true;
}

bool TrafficManagerLocal::RestoreCheckpointForCoordinator(const Checkpoint &checkpoint) {
  std::lock_guard<std::mutex> lock(registration_mutex);
  const auto settings = world.GetSettings();
  if (!parameters.GetSynchronousMode() || !settings.synchronous_mode ||
      !settings.fixed_delta_seconds || settings != checkpoint.settings || !checkpoint.parameters.synchronous_mode ||
      checkpoint.episode_id != world.GetId() || checkpoint.map.lock() != local_map ||
      step_begin.load() || step_end.load() ||
      registered_vehicles.GetState() != checkpoint.registered_state ||
      registered_vehicles.GetIDList() != checkpoint.registered_actor_ids ||
      GetWorldActorIds(world) != checkpoint.world_actor_ids)
    return false;
  const auto timestamp = world.GetSnapshot().GetTimestamp();
  RandomGenerator random(0);
  if (!random.RestoreState(checkpoint.random_state)) return false;
  auto motion = checkpoint.motion;
  auto lights = checkpoint.lights;
  auto lifecycle = checkpoint.lifecycle;
  if (!checkpoint_time::Rebase(motion, lights, lifecycle, checkpoint.timestamp, timestamp) ||
      world.GetSnapshot().GetTimestamp().frame != timestamp.frame) return false;

  // All preconditions are checked before mutation, but allocation failures or
  // external activity can still leave a partial restore; there is no rollback.
  registered_vehicles_state = checkpoint.last_processed_registered_state;
  vehicle_id_list = checkpoint.vehicle_ids;
  buffer_map = checkpoint.buffers;
  track_traffic = checkpoint.tracker;
  simulation_state = checkpoint.simulation;
  parameters.RestoreCheckpoint(checkpoint.parameters);
  random_device = random;
  seed = checkpoint.seed;
  localization_stage.RestoreCheckpoint(checkpoint.localization);
  collision_stage.RestoreCheckpoint(checkpoint.collision);
  traffic_light_stage.RestoreCheckpoint(lights);
  motion_plan_stage.RestoreCheckpoint(motion);
  alsm.RestoreCheckpoint(lifecycle);
  marked_for_removal = checkpoint.marked_for_removal;
  return true;
}

TrafficManagerLocal::TrafficManagerLocal(
  std::vector<float> longitudinal_PID_parameters,
  std::vector<float> longitudinal_highway_PID_parameters,
  std::vector<float> lateral_PID_parameters,
  std::vector<float> lateral_highway_PID_parameters,
  float perc_difference_from_limit,
  cc::detail::EpisodeProxy &episode_proxy,
  uint16_t &RPCportTM)

  : longitudinal_PID_parameters(longitudinal_PID_parameters),
    longitudinal_highway_PID_parameters(longitudinal_highway_PID_parameters),
    lateral_PID_parameters(lateral_PID_parameters),
    lateral_highway_PID_parameters(lateral_highway_PID_parameters),

    episode_proxy(episode_proxy),
    world(cc::World(episode_proxy)),

    localization_stage(LocalizationStage(vehicle_id_list,
                                         buffer_map,
                                         simulation_state,
                                         track_traffic,
                                         local_map,
                                         parameters,
                                         marked_for_removal,
                                         localization_frame,
                                         random_device)),

    collision_stage(CollisionStage(vehicle_id_list,
                                   simulation_state,
                                   buffer_map,
                                   track_traffic,
                                   parameters,
                                   collision_frame,
                                   random_device)),

    traffic_light_stage(TrafficLightStage(vehicle_id_list,
                                          simulation_state,
                                          buffer_map,
                                          parameters,
                                          world,
                                          tl_frame,
                                          random_device)),

    motion_plan_stage(MotionPlanStage(vehicle_id_list,
                                      simulation_state,
                                      parameters,
                                      buffer_map,
                                      track_traffic,
                                      longitudinal_PID_parameters,
                                      longitudinal_highway_PID_parameters,
                                      lateral_PID_parameters,
                                      lateral_highway_PID_parameters,
                                      localization_frame,
                                      collision_frame,
                                      tl_frame,
                                      world,
                                      control_frame,
                                      random_device,
                                      local_map)),

    vehicle_light_stage(VehicleLightStage(vehicle_id_list,
                                          buffer_map,
                                          parameters,
                                          world,
                                          control_frame)),

    alsm(ALSM(registered_vehicles,
              buffer_map,
              track_traffic,
              marked_for_removal,
              parameters,
              world,
              local_map,
              simulation_state,
              localization_stage,
              collision_stage,
              traffic_light_stage,
              motion_plan_stage,
              vehicle_light_stage)),

    server(TrafficManagerServer(RPCportTM, static_cast<carla::traffic_manager::TrafficManagerBase *>(this))) {

  parameters.SetGlobalPercentageSpeedDifference(perc_difference_from_limit);

  registered_vehicles_state = -1;

  SetupLocalMap();

  Start();
}

TrafficManagerLocal::~TrafficManagerLocal() {
  episode_proxy.Lock()->DestroyTrafficManager(server.port());
  Release();
}

void TrafficManagerLocal::SetupLocalMap() {
  const carla::SharedPtr<const cc::Map> world_map = world.GetMap();
  local_map = std::make_shared<InMemoryMap>(world_map);

  auto files = episode_proxy.Lock()->GetRequiredFiles("TM");
  if (!files.empty()) {
    auto content = episode_proxy.Lock()->GetCacheFile(files[0], true);
    if (content.size() != 0) {
      local_map->Load(content);
    } else {
      log_warning("No InMemoryMap cache found. Setting up local map. This may take a while...");
      local_map->SetUp();
    }
  } else {
    log_warning("No InMemoryMap cache found. Setting up local map. This may take a while...");
    local_map->SetUp();
  }
}

void TrafficManagerLocal::Start() {
  run_traffic_manger.store(true);
  worker_thread = std::make_unique<std::thread>(&TrafficManagerLocal::Run, this);
}

void TrafficManagerLocal::Run() {

  localization_frame.reserve(INITIAL_SIZE);
  collision_frame.reserve(INITIAL_SIZE);
  tl_frame.reserve(INITIAL_SIZE);
  control_frame.reserve(INITIAL_SIZE);
  current_reserved_capacity = INITIAL_SIZE;

  size_t last_frame = 0;
  while (run_traffic_manger.load()) {

    bool synchronous_mode = parameters.GetSynchronousMode();
    bool hybrid_physics_mode = parameters.GetHybridPhysicsMode();
    parameters.SetMaxBoundaries(20.0f, episode_proxy.Lock()->GetEpisodeSettings().actor_active_distance);

    // Wait for external trigger to initiate cycle in synchronous mode.
    if (synchronous_mode) {
      std::unique_lock<std::mutex> lock(step_execution_mutex);
      step_begin_trigger.wait(lock, [this]() {return step_begin.load() || !run_traffic_manger.load();});
      step_begin.store(false);
    }

    // Skipping velocity update if elapsed time is less than 0.05s in asynchronous, hybrid mode.
    if (!synchronous_mode && hybrid_physics_mode) {
      TimePoint current_instance = chr::system_clock::now();
      chr::duration<float> elapsed_time = current_instance - previous_update_instance;
      chr::duration<float> time_to_wait = chr::duration<float>(HYBRID_MODE_DT) - elapsed_time;
      if (time_to_wait > chr::duration<float>(0.0f)) {
        std::this_thread::sleep_for(time_to_wait);
      }
      previous_update_instance = current_instance;
    }

    // Stop TM from processing the same frame more than once
    if (!synchronous_mode) {
      carla::client::Timestamp timestamp = world.GetSnapshot().GetTimestamp();
      if (timestamp.frame == last_frame) {
        continue;
      }
      last_frame = timestamp.frame;
    }

    std::unique_lock<std::mutex> registration_lock(registration_mutex);
    // Updating simulation state, actor life cycle and performing necessary cleanup.
    alsm.Update();

    // Re-allocating inter-stage communication frames based on changed number of registered vehicles.
    int current_registered_vehicles_state = registered_vehicles.GetState();
    unsigned long number_of_vehicles = vehicle_id_list.size();
    if (registered_vehicles_state != current_registered_vehicles_state || number_of_vehicles != registered_vehicles.Size()) {
      vehicle_id_list = registered_vehicles.GetIDList();
      number_of_vehicles = vehicle_id_list.size();

      // Reserve more space if needed.
      uint64_t growth_factor = static_cast<uint64_t>(static_cast<float>(number_of_vehicles) * INV_GROWTH_STEP_SIZE);
      uint64_t new_frame_capacity = INITIAL_SIZE + GROWTH_STEP_SIZE * growth_factor;
      if (new_frame_capacity > current_reserved_capacity) {
        localization_frame.reserve(new_frame_capacity);
        collision_frame.reserve(new_frame_capacity);
        tl_frame.reserve(new_frame_capacity);
        control_frame.reserve(new_frame_capacity);
      }

      registered_vehicles_state = registered_vehicles.GetState();
    }

    // Reset frames for current cycle.
    localization_frame.clear();
    localization_frame.resize(number_of_vehicles);
    collision_frame.clear();
    collision_frame.resize(number_of_vehicles);
    tl_frame.clear();
    tl_frame.resize(number_of_vehicles);
    control_frame.clear();
    // Reserve two frames for each vehicle: one for the ApplyVehicleControl command,
    // and one for the optional SetVehicleLightState command
    control_frame.reserve(2 * number_of_vehicles);
    // Resize to accomodate at least all ApplyVehicleControl commands,
    // that will be inserted by the motion_plan_stage stage.
    control_frame.resize(number_of_vehicles);

    // Run core operation stages.
    for (unsigned long index = 0u; index < vehicle_id_list.size(); ++index) {
      localization_stage.Update(index);
    }
    for (unsigned long index = 0u; index < vehicle_id_list.size(); ++index) {
      collision_stage.Update(index);
    }
    collision_stage.ClearCycleCache();
    vehicle_light_stage.UpdateWorldInfo();
    for (unsigned long index = 0u; index < vehicle_id_list.size(); ++index) {
      traffic_light_stage.Update(index);
      motion_plan_stage.Update(index);
      vehicle_light_stage.Update(index);
    }

    registration_lock.unlock();

    // Sending the current cycle's batch command to the simulator.
    if (synchronous_mode) {
      episode_proxy.Lock()->ApplyBatchSync(control_frame, false);
      step_end.store(true);
      step_end_trigger.notify_one();
    } else {
      if (control_frame.size() > 0){
        episode_proxy.Lock()->ApplyBatchSync(control_frame, false);
      }
    }
  }
}

bool TrafficManagerLocal::SynchronousTick() {
  if (parameters.GetSynchronousMode()) {
    step_begin.store(true);
    step_begin_trigger.notify_one();

    std::unique_lock<std::mutex> lock(step_execution_mutex);
    step_end_trigger.wait(lock, [this]() { return step_end.load(); });
    step_end.store(false);
  }
  return true;
}

void TrafficManagerLocal::Stop() {

  run_traffic_manger.store(false);
  if (parameters.GetSynchronousMode()) {
    step_begin_trigger.notify_one();
  }

  if (worker_thread) {
    if (worker_thread->joinable()) {
      worker_thread->join();
    }
    worker_thread.release();
  }

  vehicle_id_list.clear();
  registered_vehicles.Clear();
  registered_vehicles_state = -1;
  track_traffic.Clear();
  previous_update_instance = chr::system_clock::now();
  current_reserved_capacity = 0u;

  simulation_state.Reset();
  localization_stage.Reset();
  collision_stage.Reset();
  traffic_light_stage.Reset();
  motion_plan_stage.Reset();

  buffer_map.clear();
  localization_frame.clear();
  collision_frame.clear();
  tl_frame.clear();
  control_frame.clear();

  run_traffic_manger.store(true);
  step_begin.store(false);
  step_end.store(false);
}

void TrafficManagerLocal::Release() {

  Stop();

  local_map.reset();
}

void TrafficManagerLocal::Reset() {
  Release();
  episode_proxy = episode_proxy.Lock()->GetCurrentEpisode();
  world = cc::World(episode_proxy);
  SetupLocalMap();
  Start();
}

void TrafficManagerLocal::RegisterVehicles(const std::vector<ActorPtr> &vehicle_list) {
  std::lock_guard<std::mutex> registration_lock(registration_mutex);
  registered_vehicles.Insert(vehicle_list);
}

void TrafficManagerLocal::UnregisterVehicles(const std::vector<ActorPtr> &actor_list) {
  std::lock_guard<std::mutex> registration_lock(registration_mutex);
  std::vector<ActorId> actor_id_list;
  for (auto &actor : actor_list) {
    alsm.RemoveActor(actor->GetId(), true);
  }
}

void TrafficManagerLocal::SetPercentageSpeedDifference(const ActorPtr &actor, const float percentage) {
  parameters.SetPercentageSpeedDifference(actor, percentage);
}

void TrafficManagerLocal::SetGlobalPercentageSpeedDifference(const float percentage) {
  parameters.SetGlobalPercentageSpeedDifference(percentage);
}

void TrafficManagerLocal::SetLaneOffset(const ActorPtr &actor, const float offset) {
  parameters.SetLaneOffset(actor, offset);
}

void TrafficManagerLocal::SetGlobalLaneOffset(const float offset) {
  parameters.SetGlobalLaneOffset(offset);
}

void TrafficManagerLocal::SetDesiredSpeed(const ActorPtr &actor, const float value) {
  parameters.SetDesiredSpeed(actor, value);
}

/// Method to set the automatic management of the vehicle lights
void TrafficManagerLocal::SetUpdateVehicleLights(const ActorPtr &actor, const bool do_update) {
  parameters.SetUpdateVehicleLights(actor, do_update);
}

void TrafficManagerLocal::SetCollisionDetection(const ActorPtr &reference_actor, const ActorPtr &other_actor, const bool detect_collision) {
  parameters.SetCollisionDetection(reference_actor, other_actor, detect_collision);
}

void TrafficManagerLocal::SetForceLaneChange(const ActorPtr &actor, const bool direction) {
  parameters.SetForceLaneChange(actor, direction);
}

void TrafficManagerLocal::SetAutoLaneChange(const ActorPtr &actor, const bool enable) {
  parameters.SetAutoLaneChange(actor, enable);
}

void TrafficManagerLocal::SetDistanceToLeadingVehicle(const ActorPtr &actor, const float distance) {
  parameters.SetDistanceToLeadingVehicle(actor, distance);
}

void TrafficManagerLocal::SetGlobalDistanceToLeadingVehicle(const float distance) {
  parameters.SetGlobalDistanceToLeadingVehicle(distance);
}

void TrafficManagerLocal::SetPercentageIgnoreWalkers(const ActorPtr &actor, const float perc) {
  parameters.SetPercentageIgnoreWalkers(actor, perc);
}

void TrafficManagerLocal::SetPercentageIgnoreVehicles(const ActorPtr &actor, const float perc) {
  parameters.SetPercentageIgnoreVehicles(actor, perc);
}

void TrafficManagerLocal::SetPercentageRunningLight(const ActorPtr &actor, const float perc) {
  parameters.SetPercentageRunningLight(actor, perc);
}

void TrafficManagerLocal::SetPercentageRunningSign(const ActorPtr &actor, const float perc) {
  parameters.SetPercentageRunningSign(actor, perc);
}

void TrafficManagerLocal::SetKeepRightPercentage(const ActorPtr &actor, const float percentage) {
  parameters.SetKeepRightPercentage(actor, percentage);
}

void TrafficManagerLocal::SetRandomLeftLaneChangePercentage(const ActorPtr &actor, const float percentage) {
  parameters.SetRandomLeftLaneChangePercentage(actor, percentage);
}

void TrafficManagerLocal::SetRandomRightLaneChangePercentage(const ActorPtr &actor, const float percentage) {
  parameters.SetRandomRightLaneChangePercentage(actor, percentage);
}

void TrafficManagerLocal::SetHybridPhysicsMode(const bool mode_switch) {
  parameters.SetHybridPhysicsMode(mode_switch);
}

void TrafficManagerLocal::SetHybridPhysicsRadius(const float radius) {
  parameters.SetHybridPhysicsRadius(radius);
}

void TrafficManagerLocal::SetOSMMode(const bool mode_switch) {
  parameters.SetOSMMode(mode_switch);
}

void TrafficManagerLocal::SetCustomPath(const ActorPtr &actor, const Path path, const bool empty_buffer) {
  parameters.SetCustomPath(actor, path, empty_buffer);
}

void TrafficManagerLocal::RemoveUploadPath(const ActorId &actor_id, const bool remove_path) {
  parameters.RemoveUploadPath(actor_id, remove_path);
}

void TrafficManagerLocal::UpdateUploadPath(const ActorId &actor_id, const Path path) {
  parameters.UpdateUploadPath(actor_id, path);
}

void TrafficManagerLocal::SetImportedRoute(const ActorPtr &actor, const Route route, const bool empty_buffer) {
  parameters.SetImportedRoute(actor, route, empty_buffer);
}

void TrafficManagerLocal::RemoveImportedRoute(const ActorId &actor_id, const bool remove_path) {
  parameters.RemoveImportedRoute(actor_id, remove_path);
}

void TrafficManagerLocal::UpdateImportedRoute(const ActorId &actor_id, const Route route) {
  parameters.UpdateImportedRoute(actor_id, route);
}

void TrafficManagerLocal::SetRespawnDormantVehicles(const bool mode_switch) {
  parameters.SetRespawnDormantVehicles(mode_switch);
}

void TrafficManagerLocal::SetBoundariesRespawnDormantVehicles(const float lower_bound, const float upper_bound) {
  parameters.SetBoundariesRespawnDormantVehicles(lower_bound, upper_bound);
}

void TrafficManagerLocal::SetMaxBoundaries(const float lower, const float upper) {
  parameters.SetMaxBoundaries(lower, upper);
}

Action TrafficManagerLocal::GetNextAction(const ActorId &actor_id) {
  return localization_stage.ComputeNextAction(actor_id);
}

ActionBuffer TrafficManagerLocal::GetActionBuffer(const ActorId &actor_id) {
  return localization_stage.ComputeActionBuffer(actor_id);
}

bool TrafficManagerLocal::CheckAllFrozen(TLGroup tl_to_freeze) {
  for (auto &elem : tl_to_freeze) {
    if (!elem->IsFrozen() || elem->GetState() != TLS::Red) {
      return false;
    }
  }
  return true;
}

void TrafficManagerLocal::SetSynchronousMode(bool mode) {
  const bool previous_mode = parameters.GetSynchronousMode();
  parameters.SetSynchronousMode(mode);
  if (previous_mode && !mode) {
    step_begin.store(true);
    step_begin_trigger.notify_one();
  }
}

void TrafficManagerLocal::SetSynchronousModeTimeOutInMiliSecond(double time) {
  parameters.SetSynchronousModeTimeOutInMiliSecond(time);
}

carla::client::detail::EpisodeProxy &TrafficManagerLocal::GetEpisodeProxy() {
  return episode_proxy;
}

std::vector<ActorId> TrafficManagerLocal::GetRegisteredVehiclesIDs() {
  return registered_vehicles.GetIDList();
}

void TrafficManagerLocal::SetRandomDeviceSeed(const uint64_t _seed) {
  seed = _seed;
  random_device = RandomGenerator(seed);
  world.ResetAllTrafficLights();
}

} // namespace traffic_manager
} // namespace carla
