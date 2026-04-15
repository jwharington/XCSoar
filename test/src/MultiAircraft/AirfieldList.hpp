// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#pragma once

#include "Geo/GeoPoint.hpp"

#include <string>
#include <vector>

namespace MultiAircraft
{

    struct Airfield
    {
        GeoPoint location;
        std::string name;
    };

    class AirfieldList
    {
        std::vector<Airfield> airfields;

    public:
        AirfieldList() = default;

        void Add(GeoPoint location, std::string name = {})
        {
            airfields.push_back({location, std::move(name)});
        }

        [[nodiscard]] const std::vector<Airfield> &GetAirfields() const noexcept
        {
            return airfields;
        }

        [[nodiscard]] bool empty() const noexcept
        {
            return airfields.empty();
        }

        [[nodiscard]] std::size_t size() const noexcept
        {
            return airfields.size();
        }
    };

} // namespace MultiAircraft
