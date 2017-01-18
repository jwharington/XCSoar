#pragma once

#include "Vignette.hpp"

#include <unordered_map>
#include <list>
#include <cstdio>

namespace MultiAircraft {

class AircraftModel;

class EncounterMapStore {
 public:

  struct EncounterInfo: public Vignette {
    EncounterInfo(const int _encounter_num,
                  const double _t,
                  const GeoPoint &_origin, const double _alt,
                  const SpeedVector &_wind,

                  const double _d,
                  const double _v,
                  const double _p_free):
        Vignette(_encounter_num, _t, _origin, _alt, _wind),
        d_min(_d),
        v_max(-_v),
        time_pred(_v<0? -_d/_v: 0),
        p_free(_p_free)
    {
      json_filename = get_encounter_filename();
    }

    unsigned time_close = 1;
    double d_min;
    double v_max;
    double time_pred;
    double p_free;
    std::string json_filename;
   private:
    std::string get_encounter_filename() const;
  };

  void update(const int id1, const int id2,
              const double t,
              const GeoPoint& loc,
              const double alt,
              const SpeedVector &wind,

              const double d,
              const double v,
              const double p_free);

  typedef unsigned KeyType;
  typedef std::unordered_map<KeyType, EncounterInfo > EncounterMap;

  double erase_expired(const double time, std::list<AircraftModel> &group, const double distance_threshold,
                       const bool debug);

  static const int TYP_TRAIL = 20;
  static const int MAX_TRAIL = 3*TYP_TRAIL;
  static const int HYS_TRAIL = 15;

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

  void write_diagnostics_encounter(const AircraftModel& a,
                                   const AircraftModel& b,
                                   const EncounterInfo& info) const;

  void write_diagnostics_encounter_turnmodelist(const AircraftModel& a,
                                                const AircraftModel& b,
                                                const EncounterInfo& info) const;

};

}
