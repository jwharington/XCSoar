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

#include "AverageVarioComputer.hpp"
#include "NMEA/MoreData.hpp"

void
AverageVarioComputer::Reset()
{
  delta_time.Reset();
  vario_avg_filter.Reset();
  netto_avg_filter.Reset();
}

void
AverageVarioComputer::Compute(const MoreData &basic,
                              bool circling, bool last_circling,
                              DerivedInfo &calculated,
                              const CirclingSettings &settings)
{
  const auto dt = delta_time.Update(basic.time, 1, 0);
  if (dt < 0 || circling != last_circling) {
    Reset();
    calculated.average = basic.brutto_vario;
    calculated.netto_average = basic.netto_vario;
    return;
  }

  if (dt <= 0)
    return;

  const unsigned Elapsed = uround(dt);
  if (Elapsed == 0)
    return;

  for (unsigned i = 0; i < Elapsed; ++i) {
    vario_avg_filter.Update(basic.brutto_vario);
    netto_avg_filter.Update(basic.netto_vario);
  }

  const unsigned time = circling &&
      settings.average_1_turn && (calculated.circle_period > 0) ?
      std::min((unsigned)ceil(calculated.circle_period), settings.average_base_time * 2) :
      settings.average_base_time;

  calculated.average = vario_avg_filter.Average(time);
  calculated.netto_average = netto_avg_filter.Average(time);
}
