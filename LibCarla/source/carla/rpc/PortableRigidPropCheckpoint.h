// This work is licensed under the terms of the MIT license.
#pragma once

#include "carla/MsgPack.h"
#include <cmath>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace carla { namespace rpc {

// Pointer-free experimental value record. Unreal values use cm and rad/s;
// body values use native PhysX units. This is not a complete physics snapshot.
struct PortableRigidPropCheckpoint {
  uint32_t version = 1u, physx_major = 3u, physx_minor = 4u;
  std::string blueprint, map, build_id;
  std::map<std::string, std::string> attributes;
  double fixed_delta_seconds = 0.0, max_substep_delta_time = 0.0;
  int32_t max_substeps = 0;
  bool substepping = false;
  uint32_t scene_flags = 0u, rigid_body_flags = 0u, actor_flags = 0u, shape_count = 0u;
  float mass = 0.0f, sleep_threshold = 0.0f, wake_counter = 0.0f;
  bool awake = false;
  std::vector<float> inertia, com_position, com_rotation;
  std::vector<float> position_cm, rotation_quaternion, scale, linear_velocity_cm_s, angular_velocity_rad_s;
  std::vector<float> body_position, body_rotation, body_linear_velocity, body_angular_velocity, observer_velocity_m_s;

  bool IsValid() const {
    if (version != 1u || physx_major != 3u || physx_minor != 4u ||
        blueprint.empty() || blueprint.size() > 256u || map.empty() || map.size() > 256u ||
        build_id.empty() || build_id.size() > 256u || attributes.size() > 128u ||
        !std::isfinite(fixed_delta_seconds) || fixed_delta_seconds <= 0 ||
        !std::isfinite(max_substep_delta_time) || max_substep_delta_time <= 0 || max_substeps <= 0 ||
        !std::isfinite(mass) || mass <= 0 || !std::isfinite(sleep_threshold) || sleep_threshold < 0 ||
        !std::isfinite(wake_counter) || wake_counter < 0 || shape_count == 0 || shape_count > 1024u) return false;
    for (const auto &item : attributes)
      if (item.first.empty() || item.first.size() > 256u || item.second.size() > 4096u) return false;
    const auto vector = [](const std::vector<float> &v, size_t n) {
      if (v.size() != n) return false;
      for (const auto x : v) if (!std::isfinite(x)) return false;
      return true;
    };
    const auto quaternion = [&vector](const std::vector<float> &v) {
      if (!vector(v, 4)) return false;
      double n = 0; for (float x : v) n += double(x) * x;
      return std::abs(std::sqrt(n) - 1.0) < 0.01;
    };
    if (!vector(inertia, 3) || !vector(com_position, 3) || !quaternion(com_rotation) ||
        !vector(position_cm, 3) || !quaternion(rotation_quaternion) || !vector(scale, 3) ||
        !vector(linear_velocity_cm_s, 3) || !vector(angular_velocity_rad_s, 3) ||
        !vector(body_position, 3) || !quaternion(body_rotation) || !vector(body_linear_velocity, 3) ||
        !vector(body_angular_velocity, 3) || !vector(observer_velocity_m_s, 3)) return false;
    for (float x : inertia) if (x <= 0) return false;
    for (float x : scale) if (x <= 0) return false;
    return true;
  }

  MSGPACK_DEFINE_ARRAY(version, physx_major, physx_minor, blueprint, map, build_id, attributes,
      fixed_delta_seconds, max_substep_delta_time, max_substeps, substepping, scene_flags,
      rigid_body_flags, actor_flags, shape_count, mass, sleep_threshold, wake_counter, awake,
      inertia, com_position, com_rotation, position_cm, rotation_quaternion, scale,
      linear_velocity_cm_s, angular_velocity_rad_s, body_position, body_rotation,
      body_linear_velocity, body_angular_velocity, observer_velocity_m_s);
};

}} // namespace carla::rpc
