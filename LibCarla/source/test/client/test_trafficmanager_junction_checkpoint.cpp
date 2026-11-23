// This work is licensed under the terms of the MIT license.

#include "test.h"
#include <carla/client/detail/EpisodeProxy.h>
#include <carla/client/World.h>
#include <carla/trafficmanager/TrafficLightStage.h>

TEST(trafficmanager_junction_checkpoint, copies_queues_and_stop_times) {
  namespace tm = carla::traffic_manager;
  std::vector<carla::ActorId> ids;
  tm::SimulationState simulation;
  tm::Parameters parameters;
  tm::BufferMap buffers;
  carla::client::World world{carla::client::detail::EpisodeProxy{}};
  tm::TLFrame output;
  tm::RandomGenerator random(1);
  tm::TrafficLightStage stage(ids, simulation, buffers, parameters, world, output, random);
  tm::TrafficLightStage::Checkpoint saved;
  saved.entering_vehicles_map[3] = {7, 8};
  saved.vehicle_last_junction[7] = 3;
  saved.vehicle_last_junction[8] = 3;
  saved.vehicle_stop_time.emplace(7, carla::client::Timestamp(5, 1.5, 0.05, 2.0));
  stage.RestoreCheckpoint(saved);
  saved.entering_vehicles_map.at(3).clear();
  auto captured = stage.CaptureCheckpoint();
  ASSERT_EQ(captured.entering_vehicles_map.at(3).size(), 2u);
  EXPECT_EQ(captured.entering_vehicles_map.at(3).front(), 7u);
  EXPECT_DOUBLE_EQ(captured.vehicle_stop_time.at(7).elapsed_seconds, 1.5);
  stage.RemoveActor(7);
  captured = stage.CaptureCheckpoint();
  EXPECT_EQ(captured.entering_vehicles_map.at(3).front(), 8u);
  EXPECT_EQ(captured.vehicle_stop_time.count(7), 0u);
}
