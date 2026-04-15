// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#pragma once

#include <boost/json.hpp>
#include <tuple>
#include <vector>

#include "AircraftModel.hpp"
#include "EncounterMapStore.hpp"

namespace MultiAircraft
{

    namespace TraceWriter
    {

        boost::json::object write_encounter(
            const AircraftModel &aircraft,
            const EncounterMapStore::EncounterInfo &info,
            unsigned id_target,
            bool detailed = false);

        boost::json::object write_vignette(
            const AircraftModel &aircraft,
            const Vignette &info);

        boost::json::object write_incursion(
            const AircraftModel &aircraft,
            const Vignette &info,
            const std::vector<std::pair<TimeStamp, double>> &depth_samples,
            const std::vector<std::tuple<TimeStamp, GeoPoint, double>> &boundary_samples,
            const std::vector<EventTrailSample> &trail_samples);

        boost::json::object write_terrain(
            const AircraftModel &aircraft,
            const Vignette &info,
            const std::vector<std::pair<TimeStamp, double>> &distance_samples,
            const std::vector<EventTrailSample> &trail_samples);

    } // namespace TraceWriter

} // namespace MultiAircraft
