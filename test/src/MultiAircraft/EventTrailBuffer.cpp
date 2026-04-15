// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "EventTrailBuffer.hpp"
#include "EncounterMapStore.hpp"

using namespace MultiAircraft;

EventTrailSample MultiAircraft::MakeEventTrailSample(const AircraftModel &aircraft)
{
    const auto &p = aircraft.GetTrail().back();
    EventTrailSample sample;
    sample.time = p.pos.time;
    sample.location = p.pos.location;
    sample.gps_altitude = p.pos.gps_altitude;
    sample.baro_altitude = p.pos.baro_altitude;
    sample.v_wind = p.v_wind;
    sample.v_ias = p.v_ias;
    sample.v_tas = p.v_tas;
    sample.bank_angle = p.bank_angle;
    sample.pitch_angle = p.pitch_angle;
    sample.yaw_angle = p.yaw_angle;
    sample.load_factor = p.load_factor;
    sample.plausible = p.plausible;
    return sample;
}

void MultiAircraft::AppendUniqueTrailSample(std::vector<EventTrailSample> &samples,
                                            const EventTrailSample &sample)
{
    if (!samples.empty() && samples.back().time == sample.time)
        return;

    samples.emplace_back(sample);
}

void EventTrailBuffer::Push(const AircraftModel &aircraft, const TimeStamp t)
{
    if (aircraft.GetTrail().empty())
        return;

    auto &history = recent_by_aircraft[aircraft.idi];
    AppendUniqueTrailSample(history, MakeEventTrailSample(aircraft));

    const FloatDuration history_window = std::max(FloatDuration{EncounterMapStore::TYP_TRAIL},
                                                  AircraftModel::reconstruction_pre_buffer);
    const TimeStamp history_min = t - history_window;

    while (!history.empty() && history.front().time < history_min)
        history.erase(history.begin());
}

std::vector<EventTrailSample>
EventTrailBuffer::Seed(const unsigned aircraft_id, const TimeStamp t) const
{
    auto it = recent_by_aircraft.find(aircraft_id);
    if (it == recent_by_aircraft.end())
        return {};

    const FloatDuration history_window = std::max(FloatDuration{EncounterMapStore::TYP_TRAIL},
                                                  AircraftModel::reconstruction_pre_buffer);
    const TimeStamp history_min = t - history_window;

    std::vector<EventTrailSample> seeded;
    seeded.reserve(it->second.size());
    for (const auto &sample : it->second)
    {
        if (sample.time >= history_min)
            seeded.emplace_back(sample);
    }

    return seeded;
}
