/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2016 The XCSoar Project
  A detailed list of copyright holders can be found in the file "AUTHORS".

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
}
*/

#include "CruiseComputer.hpp"
#include "NMEA/MoreData.hpp"
#include "NMEA/Derived.hpp"
#include "GlideRatioCalculator.hpp"
#include "Geo/Math.hpp"

void
CruiseComputer::Reset(DerivedInfo &calculated)
{
  calculated.cruise_start_time = -1;
  cruise_start_time = 0;
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
  if (!calculated.circling && basic.NavAltitudeAvailable()) {
    if (calculated.cruise_start_time < 0) {
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

