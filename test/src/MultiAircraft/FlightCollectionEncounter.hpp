// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#pragma once

#include "FlightCollection.hpp"
#include "FlightFlock.hpp"
#include "Vignette.hpp"

#include <unordered_map>

namespace MultiAircraft
{

  class FlightCollectionEncounter : public FlightCollection
  {
  public:
    struct VignetteOptions
    {
      bool enabled = false;
      std::string subject;
      unsigned start_time = 0;
      unsigned end_time = 0;
    };

    FlightCollectionEncounter() : flock_algorithm(proj)
    {
      display_interval = flock_algorithm.min_duration;
    }

    double DISTANCE = 30;
    double DISTANCE_VISIBILITY = 1000;
    int SCORE_BUFFER = 0;
    double P_THRESHOLD = 0.2;
    double HEIGHT_THRESHOLD_M = 100;

    void SetVignetteOptions(const VignetteOptions &options)
    {
      vignette_options = options;
      vignette_options.enabled = !vignette_options.subject.empty();
    }

    virtual boost::json::object record_summary() const override;

  protected:
    double calc_effective_distance(const AuxiliaryPair &auxiliary) const;
    virtual std::string get_symbol(const AircraftModel &m) const override;
    virtual bool process(const TimeStamp t) override;
    void encounter_update(const TimeStamp t);
    void visibility_update();
    virtual void finalise() override;
    double get_average_h_acc() const;
    double get_effective_distance(const AircraftModel &a,
                                  const AircraftModel &b) const;
    void update_vignettes(const TimeStamp t);
    void write_vignette_file();

    EncounterMapStore encounter_store;
    std::unordered_map<unsigned, Vignette> vignette_map;
    VignetteOptions vignette_options;
    FloatDuration time_close{0};
    GeoPoint delta_proximity;
    GeoPoint delta_visibility;
    GeoPoint delta_flock;

    FlightFlock flock_algorithm;
  };

}
