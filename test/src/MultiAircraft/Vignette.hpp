// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#pragma once

#include "Geo/GeoPoint.hpp"
#include "Geo/SpeedVector.hpp"
#include "Geo/Flat/FlatProjection.hpp"
#include "Geo/Flat/FlatPoint.hpp"
#include "Math/Vector.hpp"
#include "time/BrokenDate.hpp"
#include "time/Stamp.hpp"

namespace MultiAircraft
{

  struct TrailPoint;

  struct Vignette
  {
    Vignette(const int _id,
             const TimeStamp _t,
             const BrokenDate _dt,
             const GeoPoint &_origin, const double _alt,
             const SpeedVector &_wind) : id(_id),
                                         time_start(_t),
                                         time_end(_t),
                                         origin_time(_t),
                                         date_utc(_dt),
                                         origin(_origin),
                                         alt(_alt),
                                         wind(_wind),
                                         wind_acc(Vector(wind)) {}

    void finalise();

    const FlatPoint project_loc_wind(const TrailPoint &p) const;
    unsigned id;
    TimeStamp time_start;
    TimeStamp time_end;
    TimeStamp origin_time;
    BrokenDate date_utc;
    GeoPoint origin;
    double alt;
    SpeedVector wind;
    Vector wind_acc;
    int num_wind = 1;

  private:
    FlatProjection proj;
    double scale;
    GeoPoint traildrift;

    const GeoPoint calc_traildrift() const;
  };

}
