// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#pragma once

#include "AircraftModel.hpp"

namespace MultiAircraft {

class FlightCollection {
 public:
  bool load_files(Args& args);
  bool run();
  virtual boost::json::object record_summary() const;

 protected:
  virtual std::string get_symbol(const AircraftModel& m) const;
  bool advance_to_start();
  void advance_to_time(const TimeStamp t);
  virtual void finalise();
  virtual bool process(const TimeStamp t);

  std::list<AircraftModel> group;
  GeoPoint loc_general;

  TimeStamp t_start = TimeStamp::Undefined();
  TimeStamp t_end = TimeStamp::Undefined();

  int num_flightsecs = 0;
  double alt_max = 0;
  double alt_min = 0;
  double alt_start_av = 0;
  bool first = true;

  FlatProjection proj;
  int display_interval = 300;

  const GeoPoint calc_av_flight_loc_start(const bool first_pass=true) const;
  const FlatPoint project_loc(const GeoPoint& p) const
  {
    return proj.ProjectFloat(p)*proj.GetApproximateScale();
  }
  const GeoPoint unproject_loc(const FlatPoint& p) const
  {
    return proj.Unproject(p/proj.GetApproximateScale());
  }
};

}
