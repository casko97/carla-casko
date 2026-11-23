// This work is licensed under the terms of the MIT license.

#include "test.h"
#include <carla/trafficmanager/CheckpointTime.h>

TEST(trafficmanager_checkpoint_time, preserves_age_and_rejects_future_history) {
  namespace tm = carla::traffic_manager;
  const carla::client::Timestamp from(100, 10.0, 0.05, 1000.0);
  const carla::client::Timestamp to(200, 20.0, 0.05, 1100.0);
  const carla::client::Timestamp old(80, 8.0, 0.05, 998.0);
  tm::MotionPlanStage::Checkpoint motion;
  tm::TrafficLightStage::Checkpoint lights;
  tm::ALSM::Checkpoint lifecycle{};
  motion.current_timestamp = lights.current_timestamp = lifecycle.current_timestamp = old;
  motion.pid_state_map.emplace(7, tm::StateEntry{old, 0.1f, 0.2f, 0.3f});
  motion.teleportation_instance[7] = old;
  lights.vehicle_stop_time[7] = old;
  lifecycle.idle_time[7] = 8.0;
  lifecycle.elapsed_last_actor_destruction = 5.0;
  ASSERT_TRUE(tm::checkpoint_time::Rebase(motion, lights, lifecycle, from, to));
  EXPECT_DOUBLE_EQ(to.elapsed_seconds - lights.vehicle_stop_time.at(7).elapsed_seconds, 2.0);
  EXPECT_DOUBLE_EQ(to.elapsed_seconds - lifecycle.idle_time.at(7), 2.0);
  EXPECT_DOUBLE_EQ(to.elapsed_seconds - lifecycle.elapsed_last_actor_destruction, 5.0);
  EXPECT_EQ(motion.pid_state_map.at(7).time_instance.frame, 180u);
  EXPECT_DOUBLE_EQ(motion.teleportation_instance.at(7).platform_timestamp, 1098.0);
  const auto before = motion.current_timestamp;
  lights.vehicle_stop_time[7].frame = 201;
  EXPECT_FALSE(tm::checkpoint_time::Rebase(motion, lights, lifecycle, to,
                                           carla::client::Timestamp(300, 30.0, 0.05, 1200.0)));
  EXPECT_EQ(motion.current_timestamp.frame, before.frame);
  EXPECT_DOUBLE_EQ(motion.current_timestamp.elapsed_seconds, before.elapsed_seconds);
}
