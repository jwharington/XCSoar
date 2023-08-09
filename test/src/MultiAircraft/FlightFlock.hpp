// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#pragma once

#include "Flock.hpp"
#include "Geo/Flat/FlatProjection.hpp"
#include "Geo/GeoBounds.hpp"
#include <list>
#include <vector>
#include <string>
#include <boost/json.hpp>

namespace MultiAircraft {

class AircraftModel;

class FlightFlock: public Flock::PSIAlgorithm {
 public:
  FlightFlock(const FlatProjection& _proj): Flock::PSIAlgorithm(1000, 4, 150), proj(_proj) {
  }
  ~FlightFlock() {
  }

  void process_time(const int t, std::list<Flock::IndexPoint> &P);
  bool find_flock(const int index) const;
  void finalise();
  void mark_in_flock(std::list<AircraftModel>& aircraft) const;
  std::vector<std::string> ids;
 protected:
  void finalise_disks(Flock::DiskStore &disks);
  boost::json::object report_disk(const Flock::DiskNode& d);
  int n_flock = 0;
  bool first_out;
  const FlatProjection& proj;
  GeoBounds bounds;
  const GeoPoint unproject_loc(const Flock::Point& p) const;
  boost::json::array json_records;
};

}

