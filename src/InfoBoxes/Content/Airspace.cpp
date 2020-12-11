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

#include "InfoBoxes/Content/Airspace.hpp"
#include "InfoBoxes/Data.hpp"
#include "Interface.hpp"
#include "Components.hpp"
#include "Engine/Airspace/AbstractAirspace.hpp"
#include "Computer/GlideComputer.hpp"
#include "Airspace/NearestAirspace.hpp"

void
UpdateInfoBoxNearestAirspaceHorizontal(InfoBoxData &data)
{
  NearestAirspace nearest = NearestAirspace::FindHorizontal(CommonInterface::Basic(),
                                                            glide_computer->GetAirspaceWarnings(),
                                                            airspace_database);
  if (!nearest.IsDefined()) {
    data.SetInvalid();
    return;
  }

  data.SetValueFromDistance(nearest.distance);
  data.SetComment(nearest.airspace->GetName());
}

void
UpdateInfoBoxNearestAirspaceVertical(InfoBoxData &data)
{
  NearestAirspace nearest = NearestAirspace::FindVertical(CommonInterface::Basic(),
                                                          CommonInterface::Calculated(),
                                                          glide_computer->GetAirspaceWarnings(),
                                                          airspace_database);
  if (!nearest.IsDefined()) {
    data.SetInvalid();
    return;
  }

  data.SetValueFromArrival(nearest.distance);
  data.SetComment(nearest.airspace->GetName());
}

void
UpdateInfoBoxNearestAirspaceRadio(InfoBoxData &data)
{
  AltitudeState altitude;
  altitude.altitude = CommonInterface::Basic().nav_altitude;
  altitude.altitude_agl = CommonInterface::Calculated().altitude_agl;

  for (const auto &i : airspace_database.QueryInside(CommonInterface::Basic().location)) {
    const AbstractAirspace &airspace = i.GetAirspace();

    /* check delta below */
    auto base = airspace.GetBase().GetAltitude(altitude);
    auto base_delta = base - altitude.altitude;

    /* check delta above */
    auto top = airspace.GetTop().GetAltitude(altitude);
    auto top_delta = altitude.altitude - top;

    if ((top_delta <= 0) && (base_delta <= 0) && airspace.GetRadioText().length()) {
      data.SetValue(airspace.GetRadioText().c_str());
      data.SetComment(airspace.GetName());
      return;
    }
  }

  data.SetInvalid();
}
