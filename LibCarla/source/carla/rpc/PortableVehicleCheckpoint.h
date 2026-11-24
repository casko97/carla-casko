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

// Version 2 is an experimental value representation of the existing vehicle
// slot. It is not a world checkpoint or proof of cross-build compatibility.
// Position, scale, rotation quaternion (x,y,z,w), linear velocity and angular
// velocity use native Unreal units: cm, dimensionless, quaternion, cm/s, rad/s.
// The native PhysX body values use the engine's native PhysX coordinate system
// and units, and observer_velocity_m_s is CARLA's cached world velocity in m/s.
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
  uint32_t version = 2u;
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
  std::vector<float> body_position_physx;
  std::vector<float> body_rotation_quaternion;
  std::vector<float> body_linear_velocity_physx;
  std::vector<float> body_angular_velocity_physx;
  std::vector<float> observer_velocity_m_s;

  // This is a structural check only. An importer must additionally validate
  // numerical ranges, component versions, build/map/blueprint and actor setup.
  bool HasValidShape() const {
    if (version != 2u || physx_major != 3u || physx_minor != 4u ||
        position_cm.size() != 3u || rotation_quaternion.size() != 4u || scale.size() != 3u ||
        linear_velocity_cm_s.size() != 3u || angular_velocity_rad_s.size() != 3u ||
        drive.analog_inputs.size() != 16u || wheels.empty() || wheels.size() > 32u ||
        movement_floats.size() != 7u || movement_bools.size() != 3u ||
        body_position_physx.size() != 3u || body_rotation_quaternion.size() != 4u ||
        body_linear_velocity_physx.size() != 3u || body_angular_velocity_physx.size() != 3u ||
        observer_velocity_m_s.size() != 3u) return false;
    for (auto value : movement_bools) if (value > 1u) return false;
    return true;
  }

  // Checks dynamic values that will be consumed by PhysX and Unreal. This is
  // not sufficient to import: the caller must also check compatibility with
  // the target actor, its physics configuration, the map, scene and build.
  bool HasValidDynamicValues() const {
    if (!HasValidShape() || !std::isfinite(fixed_delta_seconds) || fixed_delta_seconds <= 0.0 ||
        !std::isfinite(max_substep_delta_time) || max_substep_delta_time <= 0.0 || max_substeps <= 0 ||
        !std::isfinite(wake_counter) || wake_counter < 0.0f ||
        !std::isfinite(rollover_timer_remaining) || !std::isfinite(rollover_timer_rate) ||
        (rollover_timer_exists && (rollover_timer_remaining < 0.0f || rollover_timer_rate <= 0.0f)) ||
        (rollover_timer_paused && !rollover_timer_exists) ||
        input_priority > 6u || failure_state > 3u ||
        drive.current_gear >= 32u || drive.target_gear >= 32u ||
        !std::isfinite(drive.engine_speed) || !std::isfinite(drive.gear_switch_time) ||
        !std::isfinite(drive.auto_box_switch_time)) return false;

    const auto finite_vector = [](const std::vector<float> &values) {
      for (const auto value : values) if (!std::isfinite(value)) return false;
      return true;
    };
    const auto sane_quaternion = [](const std::vector<float> &values) {
      double squared = 0.0;
      for (const auto value : values) squared += static_cast<double>(value) * value;
      return std::isfinite(squared) && std::abs(std::sqrt(squared) - 1.0) < 0.01;
    };
    const auto finite_control = [](const VehicleControl &control) {
      return std::isfinite(control.throttle) && std::isfinite(control.steer) && std::isfinite(control.brake);
    };
    if (!finite_vector(position_cm) || !finite_vector(rotation_quaternion) || !sane_quaternion(rotation_quaternion) ||
        !finite_vector(scale) || !finite_vector(linear_velocity_cm_s) || !finite_vector(angular_velocity_rad_s) ||
        !finite_vector(body_position_physx) || !finite_vector(body_rotation_quaternion) ||
        !sane_quaternion(body_rotation_quaternion) || !finite_vector(body_linear_velocity_physx) ||
        !finite_vector(body_angular_velocity_physx) || !finite_vector(observer_velocity_m_s) ||
        !finite_vector(drive.analog_inputs) || !finite_vector(movement_floats) ||
        !finite_control(input_control) || !finite_control(last_control)) return false;
    for (const auto &wheel : wheels) {
      if (!std::isfinite(wheel.speed) || !std::isfinite(wheel.corrected_speed) || !std::isfinite(wheel.angle) ||
          !std::isfinite(wheel.low_forward_speed_timer) || !std::isfinite(wheel.low_side_speed_timer) ||
          !std::isfinite(wheel.jounce)) return false;
    }
    return true;
  }

  MSGPACK_DEFINE_ARRAY(version, physx_major, physx_minor, blueprint, map, build_id, capture_frame,
      physx_scene_flags, fixed_delta_seconds, max_substep_delta_time, max_substeps, substepping,
      physics_configuration, position_cm, rotation_quaternion, scale, linear_velocity_cm_s,
      angular_velocity_rad_s, awake, wake_counter, input_control, last_control, input_priority,
      light_state, rollover_tracker, failure_state, rollover_timer_exists, rollover_timer_paused,
      rollover_timer_remaining, rollover_timer_rate, drive, wheels, movement_floats, movement_bools,
      body_position_physx, body_rotation_quaternion, body_linear_velocity_physx,
      body_angular_velocity_physx, observer_velocity_m_s);
};

} // namespace rpc
} // namespace carla
