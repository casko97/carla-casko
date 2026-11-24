// This work is licensed under the terms of the MIT license.

#include "test.h"
#include <carla/rpc/PortableVehicleCheckpoint.h>

TEST(portable_vehicle_checkpoint, round_trips_values_and_rejects_bad_shape) {
  carla::rpc::PortableVehicleCheckpoint state;
  state.position_cm = {1.0f, 2.0f, 3.0f};
  state.rotation_quaternion = {0.0f, 0.0f, 0.0f, 1.0f};
  state.scale = {1.0f, 1.0f, 1.0f};
  state.linear_velocity_cm_s = {4.0f, 5.0f, 6.0f};
  state.angular_velocity_rad_s = {0.1f, 0.2f, 0.3f};
  state.drive.analog_inputs.resize(16u);
  state.drive.engine_speed = 123.0f;
  state.wheels.resize(4u);
  state.wheels[0].jounce = 2.0f;
  state.movement_floats.resize(7u);
  state.movement_bools.resize(3u);
  state.body_position_physx = {123.5f, -456.25f, 7.0f};
  state.body_rotation_quaternion = {0.25f, 0.5f, -0.25f, 0.75f};
  state.body_linear_velocity_physx = {11.0f, 12.0f, -13.0f};
  state.body_angular_velocity_physx = {0.5f, -0.25f, 0.125f};
  state.observer_velocity_m_s = {0.1f, -1.0f, 2.0f};
  ASSERT_TRUE(state.HasValidShape());
  const auto bytes = carla::MsgPack::Pack(state);
  auto decoded = carla::MsgPack::UnPack<carla::rpc::PortableVehicleCheckpoint>(bytes);
  EXPECT_TRUE(decoded.HasValidShape());
  EXPECT_FLOAT_EQ(decoded.drive.engine_speed, 123.0f);
  EXPECT_FLOAT_EQ(decoded.wheels[0].jounce, 2.0f);
  EXPECT_EQ(decoded.body_position_physx, state.body_position_physx);
  EXPECT_EQ(decoded.body_rotation_quaternion, state.body_rotation_quaternion);
  EXPECT_EQ(decoded.body_linear_velocity_physx, state.body_linear_velocity_physx);
  EXPECT_EQ(decoded.body_angular_velocity_physx, state.body_angular_velocity_physx);
  EXPECT_EQ(decoded.observer_velocity_m_s, state.observer_velocity_m_s);
  decoded.observer_velocity_m_s.pop_back();
  EXPECT_FALSE(decoded.HasValidShape());
  decoded.observer_velocity_m_s = state.observer_velocity_m_s;
  decoded.movement_bools[1] = 2u;
  EXPECT_FALSE(decoded.HasValidShape());
  decoded.movement_bools[1] = 0u;
  decoded.version = 1u;
  EXPECT_FALSE(decoded.HasValidShape());
}
