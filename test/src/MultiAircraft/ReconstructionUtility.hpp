#pragma once
#include "FlightReconstruction.hpp"

namespace FlightReconstruction
{
    void set_state(State &state,
                   const DerivState &vstate);
    const DerivState convert_state(const State &state);
    const Eigen::Vector3d get_euler(const State &state);

    // loaders
    std::vector<Measurement> load_encounter(State &initial_state_estimate);
    void write(const State &state);
    int reconstruct_main();

};
