// This work is licensed under the terms of the MIT license.

#include "test.h"
#include <carla/trafficmanager/ALSM.h>

TEST(trafficmanager_lifecycle_checkpoint, copies_lifecycle_history) {
  namespace tm = carla::traffic_manager;
  std::vector<carla::ActorId> ids, removal;
  tm::AtomicActorSet registered;
  tm::BufferMap buffers;
  tm::SimulationState simulation;
  tm::TrackTraffic traffic;
  tm::LocalMapPtr map;
  tm::Parameters parameters;
  tm::LocalizationFrame localization_frame;
  tm::CollisionFrame collision_frame;
  tm::TLFrame light_frame;
  tm::ControlFrame control_frame;
  tm::RandomGenerator random(1);
  carla::client::World world{carla::client::detail::EpisodeProxy{}};
  std::vector<float> pid;
  tm::LocalizationStage localization(ids, buffers, simulation, traffic, map, parameters,
                                     removal, localization_frame, random);
  tm::CollisionStage collision(ids, simulation, buffers, traffic, parameters, collision_frame, random);
  tm::TrafficLightStage lights(ids, simulation, buffers, parameters, world, light_frame, random);
  tm::MotionPlanStage motion(ids, simulation, parameters, buffers, traffic, pid, pid, pid, pid,
                             localization_frame, collision_frame, light_frame, world, control_frame, random, map);
  tm::VehicleLightStage vehicle_lights(ids, buffers, parameters, world, control_frame);
  tm::ALSM lifecycle(registered, buffers, traffic, removal, parameters, world, map, simulation,
                     localization, collision, lights, motion, vehicle_lights);
  tm::ALSM::Checkpoint saved;
  saved.idle_time[7] = 2.5;
  saved.hero_actors[7] = nullptr;
  saved.unregistered_actors[8] = nullptr;
  saved.elapsed_last_actor_destruction = 4.0;
  saved.current_timestamp = carla::client::Timestamp(3, 1.5, 0.05, 2.0);
  saved.has_physics_enabled[7] = true;
  lifecycle.RestoreCheckpoint(saved);
  saved.idle_time.clear();
  auto captured = lifecycle.CaptureCheckpoint();
  EXPECT_DOUBLE_EQ(captured.idle_time.at(7), 2.5);
  EXPECT_EQ(captured.hero_actors.count(7), 1u);
  EXPECT_EQ(captured.unregistered_actors.count(8), 1u);
  EXPECT_DOUBLE_EQ(captured.elapsed_last_actor_destruction, 4.0);
  EXPECT_EQ(captured.current_timestamp.frame, 3u);
  EXPECT_TRUE(captured.has_physics_enabled.at(7));
}
