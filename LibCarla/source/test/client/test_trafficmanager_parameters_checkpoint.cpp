// This work is licensed under the terms of the MIT license.

#include "test.h"
#include <carla/trafficmanager/Parameters.h>

TEST(trafficmanager_parameters_checkpoint, restores_consumed_commands_and_paths) {
  namespace tm = carla::traffic_manager;
  tm::Parameters parameters;
  parameters.SetMaxBoundaries(20.0f, 500.0f);
  parameters.SetGlobalLaneOffset(0.5f);
  parameters.SetSynchronousMode(true);
  auto state = parameters.CaptureCheckpoint();
  state.force_lane_change[7] = {true, true};
  state.custom_route[7] = {1, 2, 3};
  state.upload_route[7] = true;
  state.ignore_collision[7] = {};
  parameters.RestoreCheckpoint(state);
  EXPECT_TRUE(parameters.GetForceLaneChange(7).change_lane);
  EXPECT_FALSE(parameters.GetForceLaneChange(7).change_lane);
  EXPECT_EQ(parameters.GetImportedRoute(7), (tm::Route{1, 2, 3}));
  parameters.SetGlobalLaneOffset(2.0f);
  parameters.RestoreCheckpoint(state);
  EXPECT_TRUE(parameters.GetForceLaneChange(7).change_lane);
  EXPECT_FLOAT_EQ(parameters.GetLaneOffset(7), 0.5f);
  const auto restored = parameters.CaptureCheckpoint();
  EXPECT_EQ(restored.ignore_collision.count(7), 1u);
  EXPECT_TRUE(restored.ignore_collision.at(7).empty());
  EXPECT_TRUE(restored.synchronous_mode);
}
