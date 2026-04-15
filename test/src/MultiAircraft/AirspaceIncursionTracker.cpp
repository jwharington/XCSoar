// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "AirspaceIncursionTracker.hpp"
#include "EncounterMapStore.hpp"
#include "Geo/Geoid.hpp"
#include "Math/Vector.hpp"

#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>

using namespace MultiAircraft;

void AirspaceIncursionTracker::Update(const TimeStamp t,
                                      std::list<AircraftModel> &group,
                                      const OpenAipAirspaces &airspaces,
                                      const double incursion_threshold_m,
                                      EventTrailBuffer &trail_buffer)
{
    if (!airspaces.IsEnabled())
        return;

    const FloatDuration max_event_duration = FloatDuration{EncounterMapStore::MAX_TRAIL_FACTOR * EncounterMapStore::TYP_TRAIL};
    std::unordered_set<unsigned> current_airspace_aircraft;
    std::unordered_set<IncursionKey, IncursionKeyHash> current_incursion_hits;

    for (auto &[_, active] : active_incursions)
        active.seen = false;

    for (const auto &a : group)
    {
        if (!a.live || !a.valid)
            continue;

        trail_buffer.Push(a, t);
        if (a.GetTrail().empty())
            continue;

        const EventTrailSample current_sample = MakeEventTrailSample(a);

        current_airspace_aircraft.emplace(a.idi);

        auto ground_it = ground_reference_by_aircraft.find(a.idi);
        if (ground_it == ground_reference_by_aircraft.end())
            ground_it = ground_reference_by_aircraft.emplace(a.idi, a.interp_loc.baro_altitude).first;
        else
            ground_it->second = std::min(ground_it->second, a.interp_loc.baro_altitude);

        const auto hits = airspaces.Query(a.interp_loc.location,
                                          a.interp_loc.baro_altitude,
                                          ground_it->second);
        const auto wind = a.Calculated().estimated_wind;

        for (const auto &hit : hits)
        {
            if (hit.depth_m < incursion_threshold_m)
                continue;

            const IncursionKey key{(unsigned)a.idi, hit.airspace_index};
            current_incursion_hits.emplace(key);

            auto it = active_incursions.find(key);

            if (it != active_incursions.end())
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
                        active.depth_samples.emplace_back(t, hit.depth_m);
                        active.boundary_samples.emplace_back(t, hit.boundary_location, hit.boundary_altitude_m);
                        AppendUniqueTrailSample(active.trail_samples, current_sample);
                        active.max_depth = std::max(active.max_depth, hit.depth_m);
                    }
                }
                continue;
            }

            {
                const bool seen_aircraft_prev_step = previous_airspace_aircraft.contains(a.idi);
                const bool was_incursion_prev_step = previous_incursion_hits.contains(key);
                if (!seen_aircraft_prev_step || was_incursion_prev_step)
                    continue;
            }

            ActiveIncursion active_inc{
                hit.airspace_index,
                Vignette(a.idi, t, a.flight_date_utc_start,
                         a.interp_loc.location,
                         a.interp_loc.baro_altitude,
                         wind),
                {},
                {},
                trail_buffer.Seed(a.idi, t),
                hit.depth_m,
                true};
            active_inc.depth_samples.emplace_back(t, hit.depth_m);
            active_inc.boundary_samples.emplace_back(t, hit.boundary_location, hit.boundary_altitude_m);
            AppendUniqueTrailSample(active_inc.trail_samples, current_sample);
            active_incursions.emplace(key, std::move(active_inc));
        }
    }

    for (auto it = active_incursions.begin(); it != active_incursions.end();)
    {
        if (it->second.seen)
        {
            ++it;
            continue;
        }

        completed_incursions[it->first.aircraft_id].push_back(std::move(it->second));
        it = active_incursions.erase(it);
    }

    previous_airspace_aircraft = std::move(current_airspace_aircraft);
    previous_incursion_hits = std::move(current_incursion_hits);
}

bool AirspaceIncursionTracker::IsActive(const unsigned aircraft_id) const noexcept
{
    for (const auto &[key, _] : active_incursions)
    {
        if (key.aircraft_id == aircraft_id)
            return true;
    }
    return false;
}

void AirspaceIncursionTracker::FlushActive()
{
    for (auto &[key, active] : active_incursions)
        completed_incursions[key.aircraft_id].push_back(std::move(active));
    active_incursions.clear();
}

void AirspaceIncursionTracker::WriteFiles(const std::list<AircraftModel> &group,
                                          const OpenAipAirspaces &airspaces)
{
    if (!airspaces.IsEnabled())
        return;

    std::size_t total_outputs = 0;
    for (const auto &[_, incursions] : completed_incursions)
        total_outputs += incursions.size();

    if (total_outputs == 0)
        return;

    std::size_t written_outputs = 0;
    std::size_t skipped_empty = 0;
    std::cout << "  [finalise] incursion files: 0/" << total_outputs << std::endl;

    for (auto &[idi, incursions] : completed_incursions)
    {
        auto aircraft_it = std::find_if(group.begin(), group.end(),
                                        [idi](const AircraftModel &a)
                                        {
                                            return a.idi == (int)idi;
                                        });
        if (aircraft_it == group.end())
            continue;

        for (std::size_t index = 0; index < incursions.size(); ++index)
        {
            auto &incursion = incursions[index];

            if (incursion.trail_samples.empty())
            {
                ++skipped_empty;
                continue;
            }

            incursion.vignette.finalise();
            const auto &metadata = airspaces.GetMetadata(incursion.airspace_index);
            const double geoid_offset = EGM96::LookupSeparation(incursion.vignette.origin);

            boost::json::array aircraft_json;
            aircraft_json.emplace_back(aircraft_it->write_incursion(incursion.vignette,
                                                                    incursion.depth_samples,
                                                                    incursion.boundary_samples,
                                                                    incursion.trail_samples));

            boost::json::object json_info = {
                {"type", "incursion"},
                {"time_start", (int)incursion.vignette.time_start.ToDuration().count()},
                {"time_end", (int)incursion.vignette.time_end.ToDuration().count()},
                {"subject", aircraft_it->id},
                {"airspace_name", metadata.name},
                {"airspace_type", metadata.type},
                {"airspace_class", metadata.icao_class_name},
                {"lower_limit", metadata.lower_label},
                {"upper_limit", metadata.upper_label},
                {"depth_max", incursion.max_depth},
                {"latitude", incursion.vignette.origin.latitude.Degrees()},
                {"longitude", incursion.vignette.origin.longitude.Degrees()},
                {"geoid_offset", geoid_offset},
                {"wind_speed", incursion.vignette.wind.norm},
                {"wind_bearing", incursion.vignette.wind.bearing.Degrees()},
                {"aircraft", aircraft_json}};

            std::ostringstream filename;
            filename << "incursion-" << aircraft_it->id << "-" << index << ".json";
            std::ofstream file(filename.str());
            file << boost::json::serialize(json_info);

            ++written_outputs;
            if (written_outputs == total_outputs || written_outputs % 25 == 0)
                std::cout << "  [finalise] incursion files: " << written_outputs << "/" << total_outputs << std::endl;
        }
    }
    if (skipped_empty > 0)
        std::cout << "  [finalise] skipped " << skipped_empty << " incursion events with empty trails" << std::endl;
}
