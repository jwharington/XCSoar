// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "system/Args.hpp"
#include <stdio.h>
#include "MultiAircraft/FlightCollectionEncounter.hpp"
#include "IGC/IGCFRInfo.hpp"

bool debug = false;

int main(int argc, char **argv)
{
  MultiAircraft::FlightCollectionEncounter flights;

  const char* root = std::getenv("ROOT");
  if (root != nullptr) {
    std::string name = std::string(root)+"/igc_fr_geoid.txt";
    IGCFRInfoDB_init(name.c_str());
  }

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

  flights.run();

  std::ofstream file_summary("summary.json");
  boost::json::object summary = flights.record_summary();
  file_summary << boost::json::serialize(summary);

  exit(0);
}

