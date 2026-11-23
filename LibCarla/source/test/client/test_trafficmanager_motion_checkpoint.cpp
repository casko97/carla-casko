// This work is licensed under the terms of the MIT license.

#include "test.h"
#include <carla/client/detail/EpisodeProxy.h>
#include <carla/client/World.h>
#include <carla/trafficmanager/MotionPlanStage.h>

TEST(trafficmanager_motion_checkpoint, copies_history_and_removes_actors) {
  namespace tm = carla::traffic_manager;
  std::vector<carla::ActorId> ids;
  tm::SimulationState simulation;
  tm::Parameters parameters;
  tm::BufferMap buffers;
  tm::TrackTraffic traffic;
  std::vector<float> pid;
  tm::LocalizationFrame localization;
  tm::CollisionFrame collisions;
  tm::TLFrame lights;
  carla::client::World world{carla::client::detail::EpisodeProxy{}};
  tm::ControlFrame controls;
  tm::RandomGenerator random(1);
  tm::LocalMapPtr map;
  tm::MotionPlanStage stage(ids, simulation, parameters, buffers, traffic,
                            pid, pid, pid, pid, localization, collisions,
                            lights, world, controls, random, map);
  tm::MotionPlanStage::Checkpoint saved;
  carla::client::Timestamp timestamp(5, 1.5, 0.05, 2.0);
  saved.pid_state_map.emplace(7, tm::StateEntry{timestamp, 0.1f, 0.2f, 0.3f});
  saved.teleportation_instance.emplace(7, timestamp);
  saved.current_timestamp = timestamp;
  stage.RestoreCheckpoint(saved);
  saved.pid_state_map.at(7).steer = 0.9f;
  auto captured = stage.CaptureCheckpoint();
  EXPECT_FLOAT_EQ(captured.pid_state_map.at(7).steer, 0.3f);
  EXPECT_DOUBLE_EQ(captured.teleportation_instance.at(7).elapsed_seconds, 1.5);
  EXPECT_EQ(captured.current_timestamp.frame, 5u);
  stage.RemoveActor(7);
  captured = stage.CaptureCheckpoint();
  EXPECT_TRUE(captured.pid_state_map.empty());
  EXPECT_TRUE(captured.teleportation_instance.empty());
}
