#pragma once

#include <string>
#include <fstream>
#include "system/Args.hpp"
#include "TrailPointList.hpp"
#include "EncounterMapStore.hpp"
#include "Computer/CirclingComputer.hpp"
#include "Computer/Wind/Computer.hpp"
#include "Computer/Settings.hpp"
#include "../DebugReplay.hpp"
#include "Averager.hpp"
#include "Visibility.hpp"

namespace MultiAircraft {

class AircraftModel {
 public:
  AircraftModel():
      interpolator(0.5) {
  }

  bool init(Args& args);

  ~AircraftModel() {
    if (replay)
      delete replay;
  }

  void advance_to_start(int &t_start, int &t_end);
  bool advance_to_time(const int t, int &t_end);
  std::string get_symbol() const;

  TurnModeList gen_turnmodelist(const EncounterMapStore::EncounterInfo &info) const;

  static void write_encounter_open(const EncounterMapStore::EncounterInfo& info, const double distance_threshold);
  static void write_encounter_close(const EncounterMapStore::EncounterInfo& info);

  bool other_visible(const EncounterMapStore::EncounterInfo& info,
                     const unsigned id_target) const;
  void write_encounter(const EncounterMapStore::EncounterInfo& info,
                       const unsigned id_target) const;

  void write_diagnostics(FILE* fout, const double alt_start_av, const double geoid_sep,
                         Averager& all_baro_error) const;
  void finalise(Averager& all_alt_start);

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

  const DerivedInfo &Calculated() const {
    assert(replay);
    return replay->Calculated();
  }
  void set_wind_if_not_available(const SpeedVector& wind_avg);

  void calc_aspect(const AircraftModel& target);

 private:
  AircraftModel(const AircraftModel&) = delete;

  const Aspect get_aspect(const AircraftModel& target) const;

  CatmullRomInterpolator interpolator;
  CirclingComputer circling_computer;
  WindComputer wind_computer;

  DebugReplay *replay = nullptr;

  bool replay_ok = false;
  GeoPoint flight_loc_start;
  double flight_time_start = 0;
  double flight_time_end = 0;
  int flight_num_records = 0;

  Averager alt_start;
  Averager alt_end;

  double baro_offset = 0;
  Averager baro_error;

  FILE* ftrace;
  FILE* fitrace;

  TrailPointList trail;
  std::ofstream json_trace_file;
  bool json_trace_first = true;

  static int num_aircraft;
  static WindSettings wind_settings;
  static CirclingSettings circling_settings;
  static GlidePolar glide_polar;

  static constexpr double ALPHA_BARO = 0.05;
  static constexpr double MIX_BARO = 0.5;

  static double first_launch;

  void advance();
  void Interpolate(const int t, const SpeedVector& wind);
  double update_baro_altitude(double& mix);

 public:
  void reset();
  GeoPoint get_flight_loc_start() const {
    return flight_loc_start;
  }
  const GeoPoint get_location() const;

 private:
  std::string get_trace_filename() const;

};

}

/*

  TODO: first_launch, DISTANCE etc should be written once, not every encounter

*/
