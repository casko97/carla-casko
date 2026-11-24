// Experimental pointer-free traffic light controller checkpoint.
#pragma once
#include <carla/MsgPack.h>
#include <cmath>
#include <cstdint>
#include <set>
#include <string>
#include <vector>

namespace carla { namespace rpc {
struct PortableLightStage {
  float time = 0;
  uint8_t state = 0;
  MSGPACK_DEFINE_ARRAY(time, state)
};
struct PortableLightController {
  std::string id;
  int32_t current_state = 0;
  int32_t sequence = 0;
  float elapsed = 0;
  uint8_t current_light_state = 0;
  std::vector<PortableLightStage> stages;
  std::vector<std::string> component_ids;
  std::vector<uint8_t> component_states;
  std::vector<std::string> actor_ids;
  std::vector<uint8_t> actor_states;
  MSGPACK_DEFINE_ARRAY(id, current_state, sequence, elapsed, current_light_state, stages,
                       component_ids, component_states, actor_ids, actor_states)
};
struct PortableLightGroup {
  int32_t junction_id = 0;
  int32_t current_controller = 0;
  bool frozen = false;
  std::vector<PortableLightController> controllers;
  MSGPACK_DEFINE_ARRAY(junction_id, current_controller, frozen, controllers)
};
struct PortableTrafficLightsCheckpoint {
  uint32_t version = 1;
  std::string map;
  std::string build_id;
  double fixed_delta_seconds = 0;
  bool manager_frozen = false;
  std::vector<PortableLightGroup> groups;
  bool IsValid() const {
    if (version != 1 || map.empty() || map.size() > 512 || build_id.empty() || build_id.size() > 256 ||
        !std::isfinite(fixed_delta_seconds) || fixed_delta_seconds <= 0 || groups.empty() || groups.size() > 10000) return false;
    std::set<int32_t> junctions;
    for (const auto &group : groups) {
      if (!junctions.insert(group.junction_id).second || group.controllers.empty() || group.controllers.size() > 1000 ||
          group.current_controller < 0 || static_cast<size_t>(group.current_controller) >= group.controllers.size()) return false;
      std::set<std::string> controllers;
      for (const auto &c : group.controllers) {
        if (c.id.empty() || c.id.size() > 512 || !controllers.insert(c.id).second || c.stages.empty() ||
            c.stages.size() > 1000 || c.current_state < 0 || static_cast<size_t>(c.current_state) >= c.stages.size() ||
            !std::isfinite(c.elapsed) || c.elapsed < 0 || c.current_light_state > 3 ||
            c.component_ids.size() != c.component_states.size() || c.actor_ids.size() != c.actor_states.size() ||
            c.component_ids.size() > 10000 || c.actor_ids.size() > 10000) return false;
        for (const auto &stage : c.stages) if (!std::isfinite(stage.time) || stage.time < 0 || stage.state > 3) return false;
        std::set<std::string> components, actors;
        for (const auto &id : c.component_ids) if (id.empty() || id.size() > 512 || !components.insert(id).second) return false;
        for (const auto &id : c.actor_ids) if (id.empty() || id.size() > 512 || !actors.insert(id).second) return false;
        for (const auto state : c.component_states) if (state > 3) return false;
        for (const auto state : c.actor_states) if (state > 3) return false;
      }
    }
    return true;
  }
  MSGPACK_DEFINE_ARRAY(version, map, build_id, fixed_delta_seconds, manager_frozen, groups)
};
} }
