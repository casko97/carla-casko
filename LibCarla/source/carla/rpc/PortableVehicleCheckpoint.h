// This work is licensed under the terms of the MIT license.

#pragma once

#include "carla/MsgPack.h"
#include "carla/rpc/VehicleControl.h"
#include "carla/rpc/VehiclePhysicsControl.h"

#include <cmath>
#include <cstdint>
#include <string>
#include <vector>

namespace carla {
namespace rpc {

// Version 1 is an experimental value representation of the existing vehicle
// slot. It is not a world checkpoint or proof of cross-build compatibility.
// Position, scale, rotation quaternion (x,y,z,w), linear velocity and angular
// velocity use native Unreal units: cm, dimensionless, quaternion, cm/s, rad/s.
struct PortableVehicleWheelState {
  float speed = 0.0f;
  float corrected_speed = 0.0f;
  float angle = 0.0f;
  float low_forward_speed_timer = 0.0f;
  float low_side_speed_timer = 0.0f;
  float jounce = 0.0f;
  MSGPACK_DEFINE_ARRAY(speed, corrected_speed, angle, low_forward_speed_timer, low_side_speed_timer, jounce);
};

struct PortableVehicleDriveState {
  std::vector<float> analog_inputs;
  bool use_auto_gears = false;
  bool gear_up = false;
  bool gear_down = false;
  uint32_t current_gear = 0u;
  uint32_t target_gear = 0u;
  float engine_speed = 0.0f;
  float gear_switch_time = 0.0f;
  float auto_box_switch_time = 0.0f;
  MSGPACK_DEFINE_ARRAY(analog_inputs, use_auto_gears, gear_up, gear_down, current_gear,
                       target_gear, engine_speed, gear_switch_time, auto_box_switch_time);
};

struct PortableVehicleCheckpoint {
  uint32_t version = 1u;
  uint32_t physx_major = 3u;
  uint32_t physx_minor = 4u;
  std::string blueprint;
  std::string map;
  std::string build_id;
  uint64_t capture_frame = 0u;
  uint32_t physx_scene_flags = 0u;
  double fixed_delta_seconds = 0.0;
  double max_substep_delta_time = 0.0;
  int32_t max_substeps = 0;
  bool substepping = false;
  VehiclePhysicsControl physics_configuration;
  std::vector<float> position_cm;
  std::vector<float> rotation_quaternion;
  std::vector<float> scale;
  std::vector<float> linear_velocity_cm_s;
  std::vector<float> angular_velocity_rad_s;
  bool awake = false;
  float wake_counter = 0.0f;
  VehicleControl input_control;
  VehicleControl last_control;
  uint32_t input_priority = 0u;
  uint32_t light_state = 0u;
  int32_t rollover_tracker = 0;
  uint32_t failure_state = 0u;
  bool rollover_timer_exists = false;
  bool rollover_timer_paused = false;
  float rollover_timer_remaining = 0.0f;
  float rollover_timer_rate = 0.0f;
  PortableVehicleDriveState drive;
  std::vector<PortableVehicleWheelState> wheels;
  std::vector<float> movement_floats;
  std::vector<uint8_t> movement_bools;

  // This is a structural check only. An importer must additionally validate
  // numerical ranges, component versions, build/map/blueprint and actor setup.
  bool HasValidShape() const {
    if (version != 1u || physx_major != 3u || physx_minor != 4u ||
        position_cm.size() != 3u || rotation_quaternion.size() != 4u || scale.size() != 3u ||
        linear_velocity_cm_s.size() != 3u || angular_velocity_rad_s.size() != 3u ||
        drive.analog_inputs.size() != 16u || wheels.empty() || wheels.size() > 32u ||
        movement_floats.size() != 7u || movement_bools.size() != 3u) return false;
    for (auto value : movement_bools) if (value > 1u) return false;
    return true;
  }

  MSGPACK_DEFINE_ARRAY(version, physx_major, physx_minor, blueprint, map, build_id, capture_frame,
      physx_scene_flags, fixed_delta_seconds, max_substep_delta_time, max_substeps, substepping,
      physics_configuration, position_cm, rotation_quaternion, scale, linear_velocity_cm_s,
      angular_velocity_rad_s, awake, wake_counter, input_control, last_control, input_priority,
      light_state, rollover_tracker, failure_state, rollover_timer_exists, rollover_timer_paused,
      rollover_timer_remaining, rollover_timer_rate, drive, wheels, movement_floats, movement_bools);
};

} // namespace rpc
} // namespace carla
