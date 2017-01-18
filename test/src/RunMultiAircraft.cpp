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

/*
  Compile with:
      make -j8 TARGET=UNIX DEBUG=y CLANG=y WERROR=y output/UNIX/bin/RunMultiAircraft
*/

#include "system/Args.hpp"
#include <stdio.h>
#include "MultiAircraft/FlightCollectionEncounter.hpp"

bool debug = false;

int main(int argc, char **argv)
{
  MultiAircraft::FlightCollectionEncounter flights;

  //  test_visibility();

  /* last 2 arguments are the proximity distance to use.
        30 metres is good
        61 metres is "legal" in Australia
  */
  // get the distance
  flights.DISTANCE = std::stoi(argv[argc-2]);
  // get the penalty buffer
  flights.SCORE_BUFFER = std::stoi(argv[argc-1]);

  Args args(argc-2, argv, "IGC_FILE* DISTANCE SCORE_BUFFER");

  if (!flights.load_files(args)) {
    printf("Error! Can't load files\n");
    exit(EXIT_FAILURE);
  }
  args.ExpectEnd();

#ifdef DIAGNOSTICS
  printf("# id0 id1 idi0 idi1 time_start time_end time_close d_min time_pred v_max wind_bearing wind_mag lon lat alt eid pattern\n");
#endif

  flights.run();

  if (debug) {
    flights.write_diagnostics();
  }

  exit(0);
}

