// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

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
  const auto dt = delta_time.Update(basic.time, std::chrono::seconds{1}, {});
  if (dt.count() < 0 || circling != last_circling) {
    Reset();
    calculated.average = basic.brutto_vario;
    calculated.netto_average = basic.netto_vario;
    return;
  }

  if (dt.count() <= 0)
    return;

  const unsigned Elapsed = std::chrono::round<std::chrono::seconds>(dt).count();
  if (Elapsed == 0)
    return;

  for (unsigned i = 0; i < Elapsed; ++i) {
    vario_avg_filter.Update(basic.brutto_vario);
    netto_avg_filter.Update(basic.netto_vario);
  }

  const unsigned time = circling &&
      settings.average_1_turn &&
      (calculated.circle_period > 0) &&
      (calculated.circle_period <= 60) ?
      (unsigned)calculated.circle_period :
      settings.average_base_time;

  calculated.average = vario_avg_filter.Average(time);
  calculated.netto_average = netto_avg_filter.Average(time);
}
