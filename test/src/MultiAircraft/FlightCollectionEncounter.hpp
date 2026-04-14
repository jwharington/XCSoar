// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#pragma once

#include "FlightCollection.hpp"
#include "FlightFlock.hpp"
#include "OpenAipAirspaces.hpp"
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

    void LoadOpenAipAirspaces(const Path &path)
    {
      openaip_airspaces.Load(path);
    }

    static void SetSkipEncounterProcessing(bool value)
    {
      skip_encounter_processing = value;
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
    void update_airspace_incursions(const TimeStamp t);
    void write_vignette_file();
    void write_incursion_files();

    EncounterMapStore encounter_store;
    std::unordered_map<unsigned, Vignette> vignette_map;
    VignetteOptions vignette_options;
    FloatDuration time_close{0};
    GeoPoint delta_proximity;
    GeoPoint delta_visibility;
    GeoPoint delta_flock;

    FlightFlock flock_algorithm;
    OpenAipAirspaces openaip_airspaces;

    struct ActiveIncursion
    {
      unsigned airspace_index;
      Vignette vignette;
      std::vector<std::pair<TimeStamp, double>> depth_samples;
      double max_depth = 0;
      bool seen = false;
    };

    struct IncursionKey
    {
      unsigned aircraft_id;
      unsigned airspace_index;

      bool operator==(const IncursionKey &) const noexcept = default;
    };

    struct IncursionKeyHash
    {
      std::size_t operator()(const IncursionKey &key) const noexcept
      {
        return (std::size_t(key.aircraft_id) << 32) ^ key.airspace_index;
      }
    };

    std::unordered_map<unsigned, double> ground_reference_by_aircraft;
    std::unordered_map<IncursionKey, ActiveIncursion, IncursionKeyHash> active_incursions;
    std::unordered_map<unsigned, std::vector<ActiveIncursion>> completed_incursions;

    static bool skip_encounter_processing;
  };

}
