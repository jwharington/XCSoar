// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#pragma once

#include "Vignette.hpp"

#include <unordered_map>
#include <list>
#include <cstdio>
#include <string>

namespace MultiAircraft {

class AircraftModel;

class EncounterMapStore {
 public:

  struct EncounterInfo: public Vignette {
    EncounterInfo(const int _encounter_num,
                  const TimeStamp _t,
		  const BrokenDate _dt,
                  const GeoPoint &_origin, const double _alt,
                  const SpeedVector &_wind,

                  const double _d,
                  const double _v,
                  const double _p_free):
        Vignette(_encounter_num, _t, _dt, _origin, _alt, _wind),
        d_min(_d),
        v_max(-_v),
        time_pred(_v<0? -_d/_v: 0),
        p_free(_p_free)
    {
    }

    FloatDuration time_close{1};
    double d_min;
    double v_max;
    FloatDuration time_pred;
    double p_free;
    std::string get_encounter_filename() const;
  };

  void update(const int id1, const int id2,
              const TimeStamp t,
	      const BrokenDate date,
              const GeoPoint& loc,
              const double alt,
              const SpeedVector &wind,

              const double d,
              const double v,
              const double p_free);

  typedef unsigned KeyType;
  typedef std::unordered_map<KeyType, EncounterInfo > EncounterMap;

  FloatDuration erase_expired(const TimeStamp time, std::list<AircraftModel> &group,
                              const double distance_threshold,
                              const double height_threshold);

  static constexpr int TYP_TRAIL = 20;
  static constexpr int MAX_TRAIL = 3*TYP_TRAIL;
  static constexpr int HYS_TRAIL = 15;

 private:
  static unsigned encounter_num;

  EncounterMap encounters;

  static constexpr KeyType make_index(const int id1, const int id2) {
    return (id1 << 16) + id2;
  }
  static void index_split(const KeyType key, int& id1, int& id2) {
    id1 = key >> 16;
    id2 = key & 0xFFFF;
  }

};

}
