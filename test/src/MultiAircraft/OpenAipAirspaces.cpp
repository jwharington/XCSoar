// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "OpenAipAirspaces.hpp"

#include "Geo/ConvexHull/PolygonInterior.hpp"
#include "Geo/SearchPoint.hpp"
#include "io/FileReader.hxx"
#include "json/Parse.hxx"

#include <boost/json.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

using namespace MultiAircraft;

namespace
{

    static const boost::json::array &
    GetAirspaceArray(const boost::json::value &json)
    {
        if (json.is_array())
            return json.as_array();

        const auto &object = json.as_object();
        auto airspaces_it = object.find("airspaces");
        if (airspaces_it == object.end() || !airspaces_it->value().is_array())
            throw std::invalid_argument{"OpenAIP JSON must be an array or contain an 'airspaces' array"};

        return airspaces_it->value().as_array();
    }

    static GeoPoint
    ParsePoint(const boost::json::array &point)
    {
        if (point.size() < 2)
            throw std::invalid_argument{"OpenAIP coordinate must have two values"};

        return GeoPoint(Angle::Degrees(point[0].to_number<double>()),
                        Angle::Degrees(point[1].to_number<double>()));
    }

    static double
    DistanceToSegment(const FlatPoint p,
                      const FlatPoint a,
                      const FlatPoint b) noexcept
    {
        const FlatPoint ab = b - a;
        const double length_squared = ab.DotProduct(ab);
        if (length_squared <= 0)
            return p.Distance(a);

        const double t = std::clamp((p - a).DotProduct(ab) / length_squared, 0.0, 1.0);
        return p.Distance(a + ab * t);
    }

}

double
OpenAipAirspaces::AltitudeLimit::ToMeters(const double ground_reference_m) const noexcept
{
    double magnitude_m = value;
    switch (unit)
    {
    case 1:
        magnitude_m *= 0.3048;
        break;

    case 6:
        magnitude_m *= 100.0 * 0.3048;
        break;

    default:
        break;
    }

    switch (reference_datum)
    {
    case 0:
        return ground_reference_m + magnitude_m;

    default:
        return magnitude_m;
    }
}

std::string
OpenAipAirspaces::AltitudeLimit::Describe() const
{
    std::string unit_label = "m";
    switch (unit)
    {
    case 1:
        unit_label = "ft";
        break;

    case 6:
        unit_label = "FL";
        break;

    default:
        break;
    }

    std::string reference_label = "MSL";
    switch (reference_datum)
    {
    case 0:
        reference_label = "GND";
        break;

    case 1:
        reference_label = "MSL";
        break;

    case 2:
        reference_label = "STD";
        break;

    default:
        break;
    }

    return std::to_string(value) + " " + unit_label + " " + reference_label;
}

OpenAipAirspaces::AltitudeLimit
OpenAipAirspaces::ParseAltitudeLimit(const boost::json::object &object)
{
    AltitudeLimit limit;
    limit.value = object.at("value").to_number<double>();
    limit.unit = object.at("unit").to_number<int>();
    limit.reference_datum = object.at("referenceDatum").to_number<int>();
    return limit;
}

OpenAipAirspaces::Polygon
OpenAipAirspaces::ParsePolygon(const boost::json::array &rings)
{
    if (rings.empty())
        throw std::invalid_argument{"OpenAIP polygon must contain at least one ring"};

    const auto &outer_ring = rings.front().as_array();
    if (outer_ring.size() < 3)
        throw std::invalid_argument{"OpenAIP polygon must contain at least three points"};

    GeoPoint center = GeoPoint::Zero();
    for (const auto &value : outer_ring)
        center += ParsePoint(value.as_array());
    center = center * (1.0 / outer_ring.size());
    center.Normalize();

    Polygon polygon;
    polygon.projection.SetCenter(center);
    polygon.points.reserve(outer_ring.size() + 1);
    polygon.projected_points.reserve(outer_ring.size() + 1);

    for (const auto &value : outer_ring)
    {
        const GeoPoint point = ParsePoint(value.as_array());
        polygon.points.emplace_back(point, polygon.projection);
        polygon.projected_points.emplace_back(polygon.projection.ProjectFloat(point));
    }

    if (!polygon.points.front().Equals(polygon.points.back()))
    {
        polygon.points.push_back(polygon.points.front());
        polygon.projected_points.push_back(polygon.projected_points.front());
    }

    return polygon;
}

double
OpenAipAirspaces::DistanceToBoundary(const Polygon &polygon,
                                     const GeoPoint &location) noexcept
{
    if (polygon.projected_points.size() < 2)
        return 0;

    const FlatPoint projected = polygon.projection.ProjectFloat(location);
    double min_distance = std::numeric_limits<double>::max();
    for (std::size_t i = 1; i < polygon.projected_points.size(); ++i)
    {
        min_distance = std::min(min_distance,
                                DistanceToSegment(projected,
                                                  polygon.projected_points[i - 1],
                                                  polygon.projected_points[i]));
    }

    return min_distance;
}

void OpenAipAirspaces::Load(const Path &path)
{
    FileReader reader(path);
    const auto json = Json::Parse(reader);
    const auto &array = GetAirspaceArray(json);

    airspaces.clear();
    airspaces.reserve(array.size());

    for (const auto &entry : array)
    {
        const auto &object = entry.as_object();
        const auto &geometry = object.at("geometry").as_object();
        const auto geometry_type = std::string(geometry.at("type").as_string());
        if (geometry_type != "Polygon" && geometry_type != "MultiPolygon")
            continue;

        Airspace airspace;
        airspace.metadata.name = std::string(object.at("name").as_string());

        auto type_it = object.find("type");
        if (type_it != object.end() && type_it->value().is_number())
            airspace.metadata.type = type_it->value().to_number<int>();

        auto class_it = object.find("icaoClass");
        if (class_it != object.end() && class_it->value().is_number())
            airspace.metadata.icao_class = class_it->value().to_number<int>();

        airspace.lower_limit = ParseAltitudeLimit(object.at("lowerLimit").as_object());
        airspace.upper_limit = ParseAltitudeLimit(object.at("upperLimit").as_object());
        airspace.metadata.lower_label = airspace.lower_limit.Describe();
        airspace.metadata.upper_label = airspace.upper_limit.Describe();

        const auto &coordinates = geometry.at("coordinates").as_array();
        if (geometry_type == "Polygon")
            airspace.polygons.push_back(ParsePolygon(coordinates));
        else
            for (const auto &polygon_value : coordinates)
                airspace.polygons.push_back(ParsePolygon(polygon_value.as_array()));

        airspaces.push_back(std::move(airspace));
    }
}

std::vector<OpenAipAirspaces::Hit>
OpenAipAirspaces::Query(const GeoPoint &location,
                        const double altitude_m,
                        const double ground_reference_m) const
{
    std::vector<Hit> hits;

    for (unsigned index = 0; index < airspaces.size(); ++index)
    {
        const auto &airspace = airspaces[index];
        const double lower_limit_m = airspace.lower_limit.ToMeters(ground_reference_m);
        const double upper_limit_m = airspace.upper_limit.ToMeters(ground_reference_m);
        if (altitude_m < lower_limit_m || altitude_m > upper_limit_m)
            continue;

        double best_horizontal_depth = -1;
        for (const auto &polygon : airspace.polygons)
        {
            if (!PolygonInterior(location, polygon.points.begin(), polygon.points.end()))
                continue;

            best_horizontal_depth = std::max(best_horizontal_depth,
                                             DistanceToBoundary(polygon, location));
        }

        if (best_horizontal_depth < 0)
            continue;

        const double vertical_depth = std::min(altitude_m - lower_limit_m,
                                               upper_limit_m - altitude_m);
        hits.push_back(Hit{index, std::min(best_horizontal_depth, vertical_depth)});
    }

    return hits;
}

const OpenAipAirspaces::Metadata &
OpenAipAirspaces::GetMetadata(const unsigned airspace_index) const noexcept
{
    return airspaces[airspace_index].metadata;
}