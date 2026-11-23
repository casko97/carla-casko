// This work is licensed under the terms of the MIT license.

#pragma once

#include "carla/MsgPack.h"
#include "carla/rpc/ActorId.h"
#include "carla/rpc/Vector3D.h"

#include <cstdint>
#include <string>
#include <vector>

namespace carla {
namespace rpc {

struct CollisionCheckpointEvent {
  uint64_t frame = 0u;
  ActorId other_actor_id = 0u;
  std::string other_actor_type;
  Vector3D normal_impulse;
  MSGPACK_DEFINE_ARRAY(frame, other_actor_id, other_actor_type, normal_impulse);
};

struct CollisionCheckpointLog {
  std::vector<CollisionCheckpointEvent> events;
  bool overflow = false;
  MSGPACK_DEFINE_ARRAY(events, overflow);
};

} // namespace rpc
} // namespace carla
