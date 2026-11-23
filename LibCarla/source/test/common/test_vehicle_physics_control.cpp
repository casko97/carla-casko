// This work is licensed under the terms of the MIT license.

#include "test.h"
#include <carla/rpc/VehiclePhysicsControl.h>

TEST(vehicle_physics_control, equality_includes_engine_torque_curve) {
  carla::rpc::VehiclePhysicsControl first;
  auto second = first;
  EXPECT_EQ(first, second);
  second.torque_curve[0].y += 1.0f;
  EXPECT_NE(first, second);
  second = first;
  second.torque_curve.push_back({6000.0f, 400.0f});
  EXPECT_NE(first, second);
  second = first;
  second.steering_curve[0].y += 0.1f;
  EXPECT_NE(first, second);
}
