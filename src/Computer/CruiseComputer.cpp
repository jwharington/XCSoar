// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "CruiseComputer.hpp"
#include "NMEA/MoreData.hpp"
#include "NMEA/Derived.hpp"
#include "GlideRatioCalculator.hpp"
#include "Geo/Math.hpp"

void
CruiseComputer::Reset(DerivedInfo &calculated)
{
  calculated.cruise_start_time = TimeStamp::Undefined();
  cruise_start_time = TimeStamp::Undefined();
}

void
CruiseComputer::ResetStats(const MoreData &basic, DerivedInfo &calculated)
{
  calculated.cruise_start_location = basic.location;
  calculated.cruise_start_altitude = basic.nav_altitude;
  calculated.cruise_start_altitude_te = basic.TE_altitude;
  calculated.cruise_start_time = basic.time;
}

void
CruiseComputer::Compute(const MoreData &basic,
                            DerivedInfo &calculated)
{
  if (!calculated.circling && basic.location_available && basic.NavAltitudeAvailable()) {
    if (!calculated.cruise_start_time.IsDefined()) {
      calculated.cruise_start_location = basic.location;
      calculated.cruise_start_altitude = basic.nav_altitude;
      calculated.cruise_start_altitude_te = basic.TE_altitude;
      calculated.cruise_start_time = basic.time;
    } else {

      if (calculated.cruise_start_time != cruise_start_time) {
        cruise_start_time = calculated.cruise_start_time;
        search_hull.clear();
        projection.SetCenter(calculated.cruise_start_location);
        search_hull.emplace_back(calculated.cruise_start_location, projection);
      }

      // update
      const AFlatGeoPoint p(projection.ProjectInteger(basic.location), basic.nav_altitude);
      if (!search_hull.IsInside(p)) {
        search_hull.emplace_back(p, projection);
        search_hull.PruneInterior();
      }

      double d;
      if (search_hull.size()>2) {
        const FlatGeoPoint s(projection.ProjectInteger(calculated.cruise_start_location));
        unsigned approx_distance_max = 0;
        auto i_best = search_hull.begin();

        for (auto i = search_hull.begin(); i != search_hull.end(); ++i) {
          const FlatGeoPoint c = i->GetFlatLocation();
          const unsigned d_this = c.Distance(s)+c.Distance(p);
          if (d_this >= approx_distance_max) {
            approx_distance_max = d_this;
            i_best = i;
          }
        }
        d = ::DoubleDistance(calculated.cruise_start_location, i_best->GetLocation(), basic.location)/2.0;
      } else {
        d = basic.location.DistanceS(calculated.cruise_start_location);
      }

      calculated.cruise_distance = d;

      calculated.cruise_gr =
          UpdateGR(calculated.cruise_gr, calculated.cruise_distance,
                   calculated.cruise_start_altitude_te - basic.TE_altitude,
                   0.5);
    }
  }

}

