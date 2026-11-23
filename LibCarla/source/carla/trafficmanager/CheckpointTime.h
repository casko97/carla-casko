// This work is licensed under the terms of the MIT license.

#pragma once

#include <cmath>
#include <utility>

#include "carla/trafficmanager/ALSM.h"

namespace carla {
namespace traffic_manager {

// These helpers align same-episode TM history to a later world frame. They do
// not change the simulator clock, establish a barrier, or restore other state.
// Timestamp arithmetic may lose floating-point precision; exact replay remains
// an empirical question. The caller must verify episode and actor identity.
namespace checkpoint_time {

inline bool RebaseSeconds(double &value, double from, double to) {
  if (!std::isfinite(value) || !std::isfinite(from) || !std::isfinite(to) ||
      value > from || to < from) return false;
  const double result = to - (from - value);
  if (!std::isfinite(result)) return false;
  value = result;
  return true;
}

inline bool RebaseTimestamp(cc::Timestamp &value, const cc::Timestamp &from,
                            const cc::Timestamp &to) {
  if (value.frame > from.frame || to.frame < from.frame ||
      !std::isfinite(value.delta_seconds)) return false;
  double elapsed = value.elapsed_seconds;
  double platform = value.platform_timestamp;
  if (!RebaseSeconds(elapsed, from.elapsed_seconds, to.elapsed_seconds) ||
      !RebaseSeconds(platform, from.platform_timestamp, to.platform_timestamp)) return false;
  value.frame += to.frame - from.frame;
  value.elapsed_seconds = elapsed;
  value.platform_timestamp = platform;
  return true;
}

// Work on copies and commit only if every timestamp is valid. A failure leaves
// all three input snapshots unchanged.
inline bool Rebase(MotionPlanStage::Checkpoint &motion,
                   TrafficLightStage::Checkpoint &lights,
                   ALSM::Checkpoint &lifecycle,
                   const cc::Timestamp &from, const cc::Timestamp &to) {
  auto new_motion = motion;
  auto new_lights = lights;
  auto new_lifecycle = lifecycle;
  if (!RebaseTimestamp(new_motion.current_timestamp, from, to) ||
      !RebaseTimestamp(new_lights.current_timestamp, from, to) ||
      !RebaseTimestamp(new_lifecycle.current_timestamp, from, to)) return false;
  for (auto &entry : new_motion.pid_state_map)
    if (!RebaseTimestamp(entry.second.time_instance, from, to)) return false;
  for (auto &entry : new_motion.teleportation_instance)
    if (!RebaseTimestamp(entry.second, from, to)) return false;
  for (auto &entry : new_lights.vehicle_stop_time)
    if (!RebaseTimestamp(entry.second, from, to)) return false;
  for (auto &entry : new_lifecycle.idle_time)
    if (!RebaseSeconds(entry.second, from.elapsed_seconds, to.elapsed_seconds)) return false;
  if (!RebaseSeconds(new_lifecycle.elapsed_last_actor_destruction,
                     from.elapsed_seconds, to.elapsed_seconds)) return false;
  motion = std::move(new_motion);
  lights = std::move(new_lights);
  lifecycle = std::move(new_lifecycle);
  return true;
}

} // namespace checkpoint_time
} // namespace traffic_manager
} // namespace carla
