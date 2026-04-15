// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "TerrainEventTracker.hpp"
#include "EncounterMapStore.hpp"
#include "Geo/Geoid.hpp"
#include "Math/Vector.hpp"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>

using namespace MultiAircraft;

void TerrainEventTracker::Update(const TimeStamp t,
                                 std::list<AircraftModel> &group,
                                 const SRTMTerrain *terrain,
                                 const double terrain_clearance_m,
                                 EventTrailBuffer &trail_buffer)
{
    if (terrain == nullptr || terrain->empty())
        return;

    const FloatDuration max_event_duration = FloatDuration{EncounterMapStore::MAX_TRAIL_FACTOR * EncounterMapStore::TYP_TRAIL};
    std::unordered_set<unsigned> current_terrain_aircraft;
    std::unordered_set<unsigned> current_terrain_hits;

    for (auto &[_, active] : active_terrain_events)
        active.seen = false;

    for (const auto &a : group)
    {
        if (!a.live || !a.valid)
            continue;

        current_terrain_aircraft.emplace(a.idi);

        trail_buffer.Push(a, t);
        if (a.GetTrail().empty())
            continue;

        const EventTrailSample current_sample = MakeEventTrailSample(a);

        const auto terrain_height = terrain->GetHeight(a.interp_loc.location);
        if (!terrain_height.has_value())
            continue;

        const double geoid_offset = EGM96::LookupSeparation(a.interp_loc.location);
        const double orthometric_altitude = a.interp_loc.gps_altitude + geoid_offset;
        const double vertical_clearance = orthometric_altitude - *terrain_height;

        const auto nearest = terrain->GetNearest3D(
            a.interp_loc.location.latitude.Degrees(),
            a.interp_loc.location.longitude.Degrees(),
            orthometric_altitude,
            terrain_clearance_m);
        if (!nearest.has_value())
            continue;

        const double terrain_distance = (vertical_clearance < 0)
                                            ? -nearest->distance_3d
                                            : nearest->distance_3d;
        if (terrain_distance > terrain_clearance_m)
            continue;

        current_terrain_hits.emplace(a.idi);

        const auto wind = a.Calculated().estimated_wind;
        auto it = active_terrain_events.find(a.idi);

        if (it != active_terrain_events.end())
        {
            auto &active = it->second;
            active.seen = true;
            if (!active.capped)
            {
                if (t - active.vignette.time_start > max_event_duration)
                {
                    active.capped = true;
                }
                else
                {
                    active.vignette.time_end = t;
                    active.vignette.wind_acc += Vector(wind);
                    ++active.vignette.num_wind;
                    active.vignette.wind = SpeedVector(active.vignette.wind_acc.y / active.vignette.num_wind,
                                                       active.vignette.wind_acc.x / active.vignette.num_wind);
                    active.distance_samples.push_back({t, terrain_distance,
                                                       nearest->terrain_latitude,
                                                       nearest->terrain_longitude,
                                                       nearest->terrain_altitude});
                    AppendUniqueTrailSample(active.trail_samples, current_sample);
                    active.min_distance = std::min(active.min_distance, terrain_distance);
                }
            }
            continue;
        }

        {
            const bool seen_aircraft_prev_step = previous_terrain_aircraft.contains(a.idi);
            const bool was_terrain_prev_step = previous_terrain_hits.contains(a.idi);
            if (!seen_aircraft_prev_step || was_terrain_prev_step)
                continue;
        }

        ActiveTerrainEvent active_ev{
            Vignette(a.idi, t, a.flight_date_utc_start,
                     a.interp_loc.location,
                     a.interp_loc.baro_altitude,
                     wind),
            {},
            trail_buffer.Seed(a.idi, t),
            terrain_distance,
            true};
        active_ev.distance_samples.push_back({t, terrain_distance,
                                              nearest->terrain_latitude,
                                              nearest->terrain_longitude,
                                              nearest->terrain_altitude});
        AppendUniqueTrailSample(active_ev.trail_samples, current_sample);
        active_terrain_events.emplace(a.idi, std::move(active_ev));
    }

    for (auto it = active_terrain_events.begin(); it != active_terrain_events.end();)
    {
        if (it->second.seen)
        {
            ++it;
            continue;
        }

        completed_terrain_events[it->first].push_back(std::move(it->second));
        it = active_terrain_events.erase(it);
    }

    previous_terrain_aircraft = std::move(current_terrain_aircraft);
    previous_terrain_hits = std::move(current_terrain_hits);
}

bool TerrainEventTracker::IsActive(const unsigned aircraft_id) const noexcept
{
    return active_terrain_events.contains(aircraft_id);
}

void TerrainEventTracker::FlushActive()
{
    for (auto &[idi, active] : active_terrain_events)
        completed_terrain_events[idi].push_back(std::move(active));
    active_terrain_events.clear();
}

void TerrainEventTracker::WriteFiles(const std::list<AircraftModel> &group)
{
    constexpr FloatDuration landing_filter_window{120};

    std::size_t total_outputs = 0;
    for (const auto &[idi, events] : completed_terrain_events)
    {
        auto aircraft_it = std::find_if(group.begin(), group.end(),
                                        [idi](const AircraftModel &a)
                                        {
                                            return a.idi == (int)idi;
                                        });
        if (aircraft_it == group.end())
            continue;

        const TimeStamp landing_time = aircraft_it->get_flight_time_end();
        for (const auto &event : events)
        {
            const bool within_landing_window = landing_time.IsDefined() &&
                                               landing_time >= event.vignette.time_end &&
                                               landing_time - event.vignette.time_end <= landing_filter_window;
            if (!within_landing_window)
                ++total_outputs;
        }
    }

    if (total_outputs == 0)
        return;

    std::size_t written_outputs = 0;
    std::size_t skipped_empty = 0;
    std::size_t skipped_landing = 0;
    std::cout << "  [finalise] terrain files: 0/" << total_outputs << std::endl;

    for (auto &[idi, events] : completed_terrain_events)
    {
        auto aircraft_it = std::find_if(group.begin(), group.end(),
                                        [idi](const AircraftModel &a)
                                        {
                                            return a.idi == (int)idi;
                                        });
        if (aircraft_it == group.end())
            continue;

        for (std::size_t index = 0; index < events.size(); ++index)
        {
            auto &event = events[index];

            const TimeStamp landing_time = aircraft_it->get_flight_time_end();
            const bool within_landing_window = landing_time.IsDefined() &&
                                               landing_time >= event.vignette.time_end &&
                                               landing_time - event.vignette.time_end <= landing_filter_window;
            if (within_landing_window)
            {
                ++skipped_landing;
                continue;
            }

            if (event.trail_samples.empty())
            {
                ++skipped_empty;
                continue;
            }

            event.vignette.finalise();
            const double geoid_offset = EGM96::LookupSeparation(event.vignette.origin);

            boost::json::array aircraft_json;
            aircraft_json.emplace_back(aircraft_it->write_terrain(event.vignette,
                                                                  event.distance_samples,
                                                                  event.trail_samples));

            boost::json::object json_info = {
                {"type", "terrain"},
                {"time_start", (int)event.vignette.time_start.ToDuration().count()},
                {"time_end", (int)event.vignette.time_end.ToDuration().count()},
                {"subject", aircraft_it->id},
                {"distance_min", event.min_distance},
                {"latitude", event.vignette.origin.latitude.Degrees()},
                {"longitude", event.vignette.origin.longitude.Degrees()},
                {"geoid_offset", geoid_offset},
                {"wind_speed", event.vignette.wind.norm},
                {"wind_bearing", event.vignette.wind.bearing.Degrees()},
                {"aircraft", aircraft_json}};

            std::ostringstream filename;
            filename << "terrain-" << aircraft_it->id << "-" << index << ".json";
            std::ofstream file(filename.str());
            file << boost::json::serialize(json_info);

            ++written_outputs;
            if (written_outputs == total_outputs || written_outputs % 25 == 0)
                std::cout << "  [finalise] terrain files: " << written_outputs << "/" << total_outputs << std::endl;
        }
    }
    if (skipped_empty > 0)
        std::cout << "  [finalise] skipped " << skipped_empty << " terrain events with empty trails" << std::endl;
    if (skipped_landing > 0)
        std::cout << "  [finalise] skipped " << skipped_landing << " terrain events within 120s of landing" << std::endl;
}
