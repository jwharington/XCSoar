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

#include "system/Args.hpp"
#include "DebugReplay.hpp"
#include "Computer/AverageVarioComputer.hpp"
#include "Computer/CirclingComputer.hpp"
#include "Computer/Settings.hpp"
#include "NMEA/Aircraft.hpp"
#include "Formatter/TimeFormatter.hpp"

#include <stdio.h>
#include <stdlib.h>

static CirclingSettings circling_settings;

static void
Run(DebugReplay &replay)
{
  CirclingComputer circling_computer;
  AverageVarioComputer average_computer;

  circling_settings.average_1_turn = true;
  circling_settings.average_base_time = 20;

  printf("# time thermalling vario vario_average avg_time_period\n");

  while (replay.Next()) {
    const MoreData &basic = replay.Basic();
    const DerivedInfo &calculated = replay.Calculated();
    const bool last_circling = calculated.circling;
    DerivedInfo vario = calculated;

    circling_computer.TurnRate(replay.SetCalculated(),
                                   basic, calculated.flight);

    circling_computer.Turning(replay.SetCalculated(),
                                  basic,
                                  calculated.flight,
                                  circling_settings);

    // Calculate the vario 30s average
    average_computer.Compute(basic, calculated.circling, last_circling, vario, circling_settings);

    if (calculated.turning)
      printf("%.0f %d %.1f %.1f %.1f\n",
                basic.time,
                calculated.circling,
                basic.brutto_vario,
                vario.average,
                calculated.circle_period);
  }
}

int main(int argc, char **argv)
{
  Args args(argc, argv, "REPLAYFILE");
  DebugReplay *replay = CreateDebugReplay(args);
  if (replay == NULL)
    return EXIT_FAILURE;

  args.ExpectEnd();

  Run(*replay);
  delete replay;

  return EXIT_SUCCESS;
}
