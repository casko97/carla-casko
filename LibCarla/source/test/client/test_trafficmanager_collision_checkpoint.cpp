// This work is licensed under the terms of the MIT license.

#include "test.h"
#include <carla/trafficmanager/CollisionStage.h>

TEST(trafficmanager_collision_checkpoint, copies_collision_history) {
  namespace tm = carla::traffic_manager;
  std::vector<carla::ActorId> ids;
  tm::SimulationState simulation;
  tm::BufferMap buffers;
  tm::TrackTraffic traffic;
  tm::Parameters parameters;
  tm::CollisionFrame output;
  tm::RandomGenerator random(1);
  tm::CollisionStage stage(ids, simulation, buffers, traffic, parameters, output, random);
  tm::CollisionStage::Checkpoint saved;
  saved.collision_locks[7] = {2.0, 3.0, 8};
  saved.geometry_cache[42] = {1.0, 2.0, 3.0, 4.0};
  saved.geodesic_boundary_map[7] = {carla::geom::Location(1.0f, 2.0f, 3.0f)};
  stage.RestoreCheckpoint(saved);
  saved.collision_locks.clear();
  const auto captured = stage.CaptureCheckpoint();
  EXPECT_EQ(captured.collision_locks.at(7).lead_vehicle_id, 8u);
  EXPECT_DOUBLE_EQ(captured.geometry_cache.at(42).inter_bbox_distance, 4.0);
  EXPECT_FLOAT_EQ(captured.geodesic_boundary_map.at(7).at(0).x, 1.0f);
  stage.RemoveActor(7);
  EXPECT_EQ(stage.CaptureCheckpoint().collision_locks.count(7), 0u);
  stage.ClearCycleCache();
  EXPECT_TRUE(stage.CaptureCheckpoint().geometry_cache.empty());
}
