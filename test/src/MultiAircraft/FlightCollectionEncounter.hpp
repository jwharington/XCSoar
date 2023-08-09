// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#pragma once

#include "FlightCollection.hpp"
#include "FlightFlock.hpp"

namespace MultiAircraft {

class FlightCollectionEncounter: public FlightCollection
{
 public:
  FlightCollectionEncounter(): flock_algorithm(proj) {
    display_interval = flock_algorithm.min_duration;
  }

  double DISTANCE = 30;
  double DISTANCE_VISIBILITY = 1000;
  int SCORE_BUFFER = 0;
  double P_THRESHOLD = 0.2;
  double HEIGHT_THRESHOLD_M = 100;

  virtual boost::json::object record_summary() const override;

 protected:
  virtual std::string get_symbol(const AircraftModel& m) const override;
  virtual bool process(const TimeStamp t) override;
  void encounter_update(const TimeStamp t);
  void visibility_update();
  virtual void finalise() override;
  double get_average_h_acc() const;

  EncounterMapStore encounter_store;
  FloatDuration time_close{0};
  GeoPoint delta_proximity;
  GeoPoint delta_visibility;
  GeoPoint delta_flock;

  FlightFlock flock_algorithm;
};

}
