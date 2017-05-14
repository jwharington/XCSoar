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

void
CruiseComputer::Reset(DerivedInfo &calculated)
{
  calculated.cruise_start_time = -1;
}

void
CruiseComputer::ResetStats(const MoreData &basic, DerivedInfo &calculated)
{
  calculated.cruise_start_location = basic.location;
  calculated.cruise_start_altitude = basic.nav_altitude;
  calculated.cruise_start_time = basic.time;
}

void
CruiseComputer::Compute(const MoreData &basic,
                            DerivedInfo &calculated)
{
  if (!calculated.circling && basic.location_available && basic.NavAltitudeAvailable()) {
    if (calculated.cruise_start_time < 0) {
      calculated.cruise_start_location = basic.location;
      calculated.cruise_start_altitude = basic.nav_altitude;
      calculated.cruise_start_time = basic.time;
    } else {
      calculated.cruise_distance =
          basic.location.DistanceS(calculated.cruise_start_location);

      calculated.cruise_gr =
          UpdateGR(calculated.cruise_gr, calculated.cruise_distance,
                   calculated.cruise_start_altitude - basic.nav_altitude,
                   0.5);
    }
  }

  /*
  if (!basic.NavAltitudeAvailable()) {
    Reset();
    calculated.gr = INVALID_GR;
    calculated.average_gr = 0;
    return;
  }

  if (!last_location_available) {
    Reset();
    calculated.gr = INVALID_GR;
    calculated.average_gr = 0;

    last_location = basic.location;
    last_location_available = basic.location_available;
    last_altitude = basic.nav_altitude;
    return;
  }

  if (!basic.location_available.Modified(last_location_available))
    return;

  auto DistanceFlown = basic.location.DistanceS(last_location);

  // Glide ratio over ground
  calculated.gr =
    UpdateGR(calculated.gr, DistanceFlown,
             last_altitude - basic.nav_altitude, 0.1);

  if (calculated.flight.flying && !calculated.circling) {
    if (!gr_calculator_initialised) {
      gr_calculator_initialised = true;
      gr_calculator.Initialize(settings);
    }

    gr_calculator.Add((int)DistanceFlown, (int)basic.nav_altitude);
    calculated.average_gr = gr_calculator.Calculate();
  } else
    gr_calculator_initialised = false;

  last_location = basic.location;
  last_location_available = basic.location_available;
  last_altitude = basic.nav_altitude;
  */
}
