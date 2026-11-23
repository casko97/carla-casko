// Copyright (c) 2026 Computer Vision Center (CVC) at the Universitat Autonoma
// de Barcelona (UAB).
//
// This work is licensed under the terms of the MIT license.

#include "test.h"
#include <carla/trafficmanager/RandomGenerator.h>

TEST(trafficmanager_random, resume_and_reject_invalid_state) {
  using carla::traffic_manager::RandomGenerator;
  RandomGenerator original(42);
  for (int i = 0; i != 17; ++i) original.next();
  const auto state = original.SerializeState();
  RandomGenerator resumed(1);
  ASSERT_TRUE(resumed.RestoreState(state));
  for (int i = 0; i != 100; ++i) EXPECT_EQ(original.next(), resumed.next());

  const auto before = resumed.SerializeState();
  EXPECT_FALSE(resumed.RestoreState(state + " garbage"));
  EXPECT_FALSE(resumed.RestoreState(state.substr(0, state.size() / 2)));
  EXPECT_FALSE(resumed.RestoreState("unknown"));
  EXPECT_FALSE(resumed.RestoreState(std::string(20001, 'x')));
  EXPECT_EQ(before, resumed.SerializeState());
}
