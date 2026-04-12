#pragma once
#include "FlightReconstruction.hpp"
#include <array>
#include <string_view>

namespace FlightReconstruction
{
    template <size_t N>
    bool SetNamedDefault(
        std::array<std::pair<std::string_view, double>, N> &defaults,
        const std::string_view name,
        const double value)
    {
        for (auto &entry : defaults)
        {
            if (entry.first == name)
            {
                entry.second = value;
                return true;
            }
        }

        return false;
    }

    template <typename Matrix, size_t N>
    void SetDiagonalFromNamedDefaults(
        Matrix &matrix,
        const std::array<std::pair<std::string_view, double>, N> &defaults)
    {
        matrix.setZero();
        for (size_t i = 0; i < N; ++i)
        {
            (void)defaults[i].first;
            matrix(static_cast<Eigen::Index>(i), static_cast<Eigen::Index>(i)) =
                defaults[i].second;
        }
    }

    void set_state(State &state,
                   const DerivState &vstate);
    const DerivState convert_state(const State &state);
    const Eigen::Vector3d get_euler(const State &state);

    // loaders
    std::vector<Measurement> load_encounter(State &initial_state_estimate);
    void write(const State &state);
    int reconstruct_main();

};
