// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#pragma once

#include <string>
#include <fstream>
#include <tuple>

#include <boost/json.hpp>

#include "system/Args.hpp"
#include "TrailPointList.hpp"
#include "EncounterMapStore.hpp"
#include "Computer/CirclingComputer.hpp"
#include "Computer/Wind/Computer.hpp"
#include "Computer/Settings.hpp"
#include "../DebugReplay.hpp"
#include "Averager.hpp"
#include "Visibility.hpp"

namespace MultiAircraft
{

  struct EventTrailSample
  {
    TimeStamp time = TimeStamp::Undefined();
    GeoPoint location;
    double gps_altitude = 0;
    double baro_altitude = 0;
    SpeedVector v_wind = SpeedVector(0, 0);
    double v_ias = 0;
    double v_tas = 0;
    Angle bank_angle = Angle::Native(0);
    Angle pitch_angle = Angle::Native(0);
    Angle yaw_angle = Angle::Native(0);
    double load_factor = 0;
    bool plausible = true;
  };

  class AircraftModel
  {
  public:
    AircraftModel() : interpolator(FloatDuration(0.5))
    {
    }

    bool init(Args &args);

    ~AircraftModel()
    {
      if (replay)
        delete replay;
    }

    void advance_to_start(TimeStamp &t_start, TimeStamp &t_end);
    bool advance_to_time(const TimeStamp t, TimeStamp &t_end);
    std::string get_symbol() const;

    TurnModeList gen_turnmodelist(const EncounterMapStore::EncounterInfo &info) const;

    bool other_visible(const EncounterMapStore::EncounterInfo &info,
                       const unsigned id_target) const;
    boost::json::object write_encounter(const EncounterMapStore::EncounterInfo &info,
                                        const unsigned id_target,
                                        const bool detailed = false) const;
    boost::json::object write_vignette(const Vignette &info) const;
    boost::json::object write_incursion(const Vignette &info,
                                        const std::vector<std::pair<TimeStamp, double>> &depth_samples,
                                        const std::vector<std::tuple<TimeStamp, GeoPoint, double>> &boundary_samples,
                                        const std::vector<EventTrailSample> &trail_samples) const;
    boost::json::object write_terrain(const Vignette &info,
                                      const std::vector<std::pair<TimeStamp, double>> &distance_samples,
                                      const std::vector<EventTrailSample> &trail_samples) const;
    bool within_horizontal_distance(const AircraftModel &other,
                                    const TimeStamp t_min,
                                    const TimeStamp t_max,
                                    const double distance_m) const;
    bool get_average_wind(const TimeStamp t_min,
                          const TimeStamp t_max,
                          SpeedVector &wind) const;
    bool get_first_location(const TimeStamp t_min,
                            const TimeStamp t_max,
                            GeoPoint &location,
                            TimeStamp &time) const;

    boost::json::object record_summary(const double alt_start_av, const double geoid_sep,
                                       Averager &all_baro_error) const;
    void finalise(Averager &all_alt_start);

    bool valid;
    bool live = false;
    bool mark = false;
    bool in_flock = false;
    std::string id;
    std::string fr_info;
    std::string fr_id;
    int idi;
    int n_encounters = 0;
    double penalty = 0;
    double h_acc = 0.0; // standard deviation of horizontal accuracy
    double v_acc = 5.0; // standard deviation of vertical accuracy
    EulerAngles euler;

    CatmullRomInterpolator::Record interp_loc;
    CatmullRomInterpolator::Record interp_loc_last;

    const DerivedInfo &Calculated() const
    {
      assert(replay);
      return replay->Calculated();
    }
    void set_wind_if_not_available(const SpeedVector &wind_avg);

    void calc_auxiliary(const AircraftModel &target);
    bool aliased(const AircraftModel &other) const;
    bool flight_present(const bool first_pass) const;
    const TrailPointList &GetTrail() const
    {
      return trail;
    }

    static TimeStamp first_launch;
    BrokenDate flight_date_utc_start;

    const AuxiliaryPair &get_latest_auxiliary(const AircraftModel &target) const;

  private:
    AircraftModel(const AircraftModel &) = delete;

    CatmullRomInterpolator interpolator;
    CirclingComputer circling_computer;
    WindComputer wind_computer;

    DebugReplay *replay = nullptr;

    bool replay_ok = false;
    GeoPoint flight_loc_start;
    TimeStamp flight_time_start = TimeStamp::Undefined();
    TimeStamp flight_time_end = TimeStamp::Undefined();
    int flight_num_records = 0;

    Averager alt_start;
    Averager alt_end;

    double baro_offset = 0;
    Averager baro_error;

    TrailPointList trail;
    boost::json::object json_trace;

    static int num_aircraft;
    static WindSettings wind_settings;
    static CirclingSettings circling_settings;
    static GlidePolar glide_polar;

    static constexpr double ALPHA_BARO = 0.05;

    bool advance();
    void Interpolate(const TimeStamp t, const SpeedVector &wind);
    double update_baro_altitude(double &mix);

  public:
    static double MIX_BARO;
    static int filter_type;
    static FloatDuration reconstruction_pre_buffer;
    static bool write_trace_files;
    static bool keep_full_trail;

    static void SetReconstructionPreBuffer(const double seconds)
    {
      reconstruction_pre_buffer = FloatDuration(seconds);
    }

    static void SetWriteTraceFiles(const bool enabled)
    {
      write_trace_files = enabled;
    }

    static void SetKeepFullTrail(const bool enabled)
    {
      keep_full_trail = enabled;
    }

    void reset();
    GeoPoint get_flight_loc_start() const
    {
      return flight_loc_start;
    }
    const GeoPoint get_location() const;

  private:
    std::string get_trace_filename() const;
  };

}
