// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#pragma once

#include "AircraftModel.hpp"

#include <unordered_map>
#include <vector>

namespace MultiAircraft
{

    EventTrailSample MakeEventTrailSample(const AircraftModel &aircraft);

    void AppendUniqueTrailSample(std::vector<EventTrailSample> &samples,
                                 const EventTrailSample &sample);

    class EventTrailBuffer
    {
    public:
        void Push(const AircraftModel &aircraft, const TimeStamp t);

        std::vector<EventTrailSample> Seed(unsigned aircraft_id,
                                           const TimeStamp t) const;

    private:
        std::unordered_map<unsigned, std::vector<EventTrailSample>> recent_by_aircraft;
    };

}
