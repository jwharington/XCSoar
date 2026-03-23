// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#pragma once

namespace MultiAircraft
{

    struct TrailPoint;

    struct DetectMiss
    {
        DetectMiss(const TrailPoint &p0, const TrailPoint &p1);
        DetectMiss() {};

        double xrel[3] = {0, 0, 0};
        double xrel_mag = 0.0;
        double vrel[3] = {0, 0, 0};
        double vrel_mag = 0.0;

        double miss_d[3] = {0, 0, 0};
        double miss_d_mag = 0.0;
        double TCA = 0.0;
        double distance_scale = 1.0;
        double d_mag = 0.0;

        static double VELOCITY_SCALE_MS;
    };

}
