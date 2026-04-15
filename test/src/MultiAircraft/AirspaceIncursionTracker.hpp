// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#pragma once

#include "EventTrailBuffer.hpp"
#include "OpenAipAirspaces.hpp"
#include "Vignette.hpp"

#include <list>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace MultiAircraft
{

    class AirspaceIncursionTracker
    {
    public:
        struct ActiveIncursion
        {
            unsigned airspace_index;
            Vignette vignette;
            std::vector<std::pair<TimeStamp, double>> depth_samples;
            std::vector<std::tuple<TimeStamp, GeoPoint, double>> boundary_samples;
            std::vector<EventTrailSample> trail_samples;
            double max_depth = 0;
            bool seen = false;
            bool capped = false;
        };

        struct IncursionKey
        {
            unsigned aircraft_id;
            unsigned airspace_index;

            bool operator==(const IncursionKey &) const noexcept = default;
        };

        struct IncursionKeyHash
        {
            std::size_t operator()(const IncursionKey &key) const noexcept
            {
                return (std::size_t(key.aircraft_id) << 32) ^ key.airspace_index;
            }
        };

        void Update(const TimeStamp t,
                    std::list<AircraftModel> &group,
                    const OpenAipAirspaces &airspaces,
                    double incursion_threshold_m,
                    EventTrailBuffer &trail_buffer);

        void FlushActive();

        bool IsActive(unsigned aircraft_id) const noexcept;

        void WriteFiles(const std::list<AircraftModel> &group,
                        const OpenAipAirspaces &airspaces);
        std::unordered_map<unsigned, std::vector<ActiveIncursion>> completed_incursions;

    private:
        std::unordered_map<unsigned, double> ground_reference_by_aircraft;
        std::unordered_map<IncursionKey, ActiveIncursion, IncursionKeyHash> active_incursions;
        std::unordered_set<unsigned> previous_airspace_aircraft;
        std::unordered_set<IncursionKey, IncursionKeyHash> previous_incursion_hits;
    };

}
