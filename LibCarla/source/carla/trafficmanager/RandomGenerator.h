
#pragma once

#include <random>
#include <locale>
#include <sstream>
#include <string>
#include <unordered_map>

#include "carla/rpc/ActorId.h"

namespace carla {
namespace traffic_manager {

class RandomGenerator {
public:
    RandomGenerator(const uint64_t seed): mt(std::mt19937(seed)), dist(0.0, 100.0) {}
    double next() { return dist(mt); }

    // The caller must stop all draws and reseeding during capture and restore.
    // This is only the generator state, not a Traffic Manager checkpoint.
    std::string SerializeState() const {
        std::ostringstream output;
        output.imbue(std::locale::classic());
        output << "carla-tm-rng-v1 " << mt << ' ' << dist;
        return output.str();
    }

    bool RestoreState(const std::string &state) {
        if (state.size() > 20000u) return false;
        std::istringstream input(state);
        input.imbue(std::locale::classic());
        std::string version;
        std::mt19937 candidate_engine;
        std::uniform_real_distribution<double> candidate_dist;
        if (!(input >> version) || version != "carla-tm-rng-v1" ||
            !(input >> candidate_engine >> candidate_dist) ||
            candidate_dist.a() != 0.0 || candidate_dist.b() != 100.0) return false;
        input >> std::ws;
        if (!input.eof()) return false;
        mt = candidate_engine;
        dist = candidate_dist;
        return true;
    }
private:
    std::mt19937 mt;
    std::uniform_real_distribution<double> dist;
};

} // namespace traffic_manager
} // namespace carla
