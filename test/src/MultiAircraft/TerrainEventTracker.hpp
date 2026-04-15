// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#pragma once

#include "EventTrailBuffer.hpp"
#include "SRTMTerrain.hpp"
#include "TerrainDistanceSample.hpp"
#include "Vignette.hpp"

#include <list>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace MultiAircraft
{

    class TerrainEventTracker
    {
    public:
        struct ActiveTerrainEvent
        {
            Vignette vignette;
            std::vector<TerrainDistanceSample> distance_samples;
            std::vector<EventTrailSample> trail_samples;
            double min_distance = 0;
            bool seen = false;
            bool capped = false;
        };

        void Update(const TimeStamp t,
                    std::list<AircraftModel> &group,
                    const SRTMTerrain *terrain,
                    double terrain_clearance_m,
                    EventTrailBuffer &trail_buffer);

        void FlushActive();

        bool IsActive(unsigned aircraft_id) const noexcept;

        void WriteFiles(const std::list<AircraftModel> &group);

        std::unordered_map<unsigned, std::vector<ActiveTerrainEvent>> completed_terrain_events;

    private:
        std::unordered_map<unsigned, ActiveTerrainEvent> active_terrain_events;
        std::unordered_set<unsigned> previous_terrain_aircraft;
        std::unordered_set<unsigned> previous_terrain_hits;
    };

}
