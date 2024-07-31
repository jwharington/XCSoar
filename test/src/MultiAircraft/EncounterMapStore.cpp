// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "EncounterMapStore.hpp"
#include "AircraftModel.hpp"
#include <set>
#include <sstream>
#include <iomanip>      // std::setprecision
#include <fstream>

using namespace MultiAircraft;

unsigned EncounterMapStore::encounter_num = 0;

void EncounterMapStore::update(const int id1, const int id2,
                               const TimeStamp t,
			       const BrokenDate date,
                               const GeoPoint& loc,
                               const double alt,
                               const SpeedVector &wind,
                               const double d,
                               const double v,
                               const double p_free)
{
  const KeyType index = make_index(id1, id2);
  auto it = encounters.find(index);
  if (it == encounters.end()) {
    encounters.emplace(index, EncounterInfo(encounter_num, t, date, loc, alt, wind, d, v, p_free));
    encounter_num++;
  } else {
    EncounterInfo &pt = it->second;
    pt.time_end = t;
    pt.time_close++;
    if (d < pt.d_min) {
      pt.d_min = d;
      pt.origin = loc;
    }
    if (v<0) {
      pt.time_pred = std::min(FloatDuration(-d/v), pt.time_pred);
      pt.v_max = std::max(-v, pt.v_max);
    }
    pt.wind_acc += Vector(wind);
    pt.num_wind ++;
    pt.wind = SpeedVector(pt.wind_acc.y/pt.num_wind, pt.wind_acc.x/pt.num_wind);
    pt.p_free *= p_free;
  }
}


static bool is_expired(const EncounterMapStore::EncounterInfo& info, const TimeStamp time)
{
  if (time - info.time_end > FloatDuration{EncounterMapStore::HYS_TRAIL}) {
    return true;
  }
  if (time - info.time_start > FloatDuration{EncounterMapStore::MAX_TRAIL}) {
    return true;
  }
  return false;
}


FloatDuration EncounterMapStore::erase_expired(const TimeStamp time, std::list<AircraftModel> &group,
                                               const double distance_threshold,
                                               const double height_threshold)
{
  FloatDuration time_close{0};
  for (auto i = encounters.begin(), last = encounters.end(); i != last; ) {
    auto& info = i->second;
    if (is_expired(info, time)) {

      const double penalty = distance_threshold-info.d_min;
      if ((penalty < 0) // ignore no-penalty items
          || (info.time_close.count()<2.0) // ignore one second intrusions
          || (info.alt < height_threshold) // ignore very low
        ) {
          i = encounters.erase(i);
          continue;
      }
      time_close += info.time_close;
      info.finalise();

      int id1;
      int id2;
      index_split(i->first, id1, id2);

      boost::json::array json_aircraft;

      // iterate over aircraft, write primary encounter aircraft
      for (auto&& a : group) {
        if ((a.idi == id1) || (a.idi == id2)) {
          json_aircraft.emplace_back(a.write_encounter(info, a.idi == id1? id2: id1, true));
          a.penalty += penalty;
          a.n_encounters++;
          a.mark = true;
        }
      }

      std::set<int> others;
      // iterate over aircraft, write primary encounter aircraft
      for (auto&& a : group) {
        if ((a.idi == id1)||(a.idi == id2)) {
          for (auto&& b : group) {
            if ((b.idi != id1) && (b.idi != id2)) {
              auto it = others.find(b.idi);
              if ((it == others.end()) && a.other_visible(info, b.idi)) {
                others.insert(b.idi);
              }
            }
          }
        }
      }
      for (auto&& id : others) {
        for (auto&& a : group) {
          if (a.idi == id) {
            json_aircraft.emplace_back(a.write_encounter(info, -1));
          }
        }
      }

      {
        std::ofstream json_encounter_file(info.get_encounter_filename());
        boost::json::object json_info = {
          {"d_threshold", distance_threshold},
          {"time_start", (int)info.time_start.ToDuration().count()},
          {"time_end", (int)info.time_end.ToDuration().count()},
          {"time_close", (int)info.time_close.count()},
          {"d_min", info.d_min},
          {"time_pred", info.time_pred.count()},
          {"v_max", info.v_max},
          {"latitude", info.origin.latitude.Degrees()},
          {"longitude", info.origin.longitude.Degrees()},
          {"v_max", info.v_max},
          {"p_close", 1-info.p_free},
          {"aircraft", json_aircraft}
        };
        json_encounter_file << boost::json::serialize(json_info);
      }

      i = encounters.erase(i);
    } else {
      ++i;
    }
  }
  return time_close;
}

std::string EncounterMapStore::EncounterInfo::get_encounter_filename() const
{
  std::ostringstream oss;
  oss << "encounter_"
      << std::setw(5) << std::setfill('0') << id
      << ".json";
  return oss.str();
}
