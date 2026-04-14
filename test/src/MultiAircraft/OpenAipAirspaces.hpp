// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#pragma once

#include "Geo/GeoPoint.hpp"
#include "Geo/Flat/FlatPoint.hpp"
#include "Geo/Flat/FlatProjection.hpp"
#include "Geo/SearchPoint.hpp"
#include "system/Path.hpp"

#include <boost/json/fwd.hpp>

#include <string>
#include <vector>

namespace MultiAircraft
{

    class OpenAipAirspaces
    {
    public:
        struct Hit
        {
            unsigned airspace_index;
            double depth_m;
        };

        struct Metadata
        {
            std::string name;
            int type = -1;
            int icao_class = -1;
            std::string lower_label;
            std::string upper_label;
        };

        void Load(const Path &path);

        bool IsEnabled() const noexcept
        {
            return !airspaces.empty();
        }

        std::vector<Hit> Query(const GeoPoint &location,
                               double altitude_m,
                               double ground_reference_m) const;

        const Metadata &GetMetadata(unsigned airspace_index) const noexcept;

    private:
        struct AltitudeLimit
        {
            double value = 0;
            int unit = 0;
            int reference_datum = 0;

            double ToMeters(double ground_reference_m) const noexcept;
            std::string Describe() const;
        };

        struct Polygon
        {
            FlatProjection projection;
            std::vector<SearchPoint> points;
            std::vector<FlatPoint> projected_points;
        };

        struct Airspace
        {
            Metadata metadata;
            AltitudeLimit lower_limit;
            AltitudeLimit upper_limit;
            std::vector<Polygon> polygons;
        };

        std::vector<Airspace> airspaces;

        static AltitudeLimit ParseAltitudeLimit(const boost::json::object &object);
        static Polygon ParsePolygon(const boost::json::array &rings);
        static double DistanceToBoundary(const Polygon &polygon,
                                         const GeoPoint &location) noexcept;
    };

}