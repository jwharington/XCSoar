// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#pragma once

#include "time/Stamp.hpp"

namespace MultiAircraft
{

    struct TerrainDistanceSample
    {
        TimeStamp time;
        double distance;
        double terrain_latitude;
        double terrain_longitude;
        double terrain_altitude;
    };

}
