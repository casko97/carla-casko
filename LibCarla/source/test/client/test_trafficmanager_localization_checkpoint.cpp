// This work is licensed under the terms of the MIT license.

#include "test.h"
#include <carla/trafficmanager/LocalizationStage.h>

TEST(trafficmanager_localization_checkpoint, copies_localization_history) {
  namespace tm = carla::traffic_manager;
  std::vector<carla::ActorId> ids;
  tm::BufferMap buffers;
  tm::SimulationState simulation;
  tm::TrackTraffic traffic;
  tm::LocalMapPtr map;
  tm::Parameters parameters;
  std::vector<carla::ActorId> removal;
  tm::LocalizationFrame output;
  tm::RandomGenerator random(1);
  tm::LocalizationStage stage(ids, buffers, simulation, traffic, map, parameters, removal, output, random);
  tm::LocalizationStage::Checkpoint saved;
  saved.last_lane_change_swpt[7] = nullptr;
  saved.vehicles_at_junction.insert(7);
  saved.vehicles_at_junction_entrance[7] = {nullptr, nullptr};
  stage.RestoreCheckpoint(saved);
  saved.last_lane_change_swpt.clear();
  saved.vehicles_at_junction.clear();
  saved.vehicles_at_junction_entrance.clear();
  const auto captured = stage.CaptureCheckpoint();
  EXPECT_EQ(captured.last_lane_change_swpt.count(7), 1u);
  EXPECT_EQ(captured.vehicles_at_junction.count(7), 1u);
  EXPECT_EQ(captured.vehicles_at_junction_entrance.count(7), 1u);
  stage.RemoveActor(7);
  EXPECT_TRUE(stage.CaptureCheckpoint().vehicles_at_junction_entrance.empty());
  stage.RestoreCheckpoint(captured);
  stage.Reset();
  EXPECT_TRUE(stage.CaptureCheckpoint().vehicles_at_junction_entrance.empty());
}
