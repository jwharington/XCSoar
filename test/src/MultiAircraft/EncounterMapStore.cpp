#include "EncounterMapStore.hpp"
#include "AircraftModel.hpp"
#include <set>
#include <sstream>
#include <iomanip>      // std::setprecision
#include <fstream>

using namespace MultiAircraft;

unsigned EncounterMapStore::encounter_num = 0;

void EncounterMapStore::update(const int id1, const int id2,
                               const double t,
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
    encounters.emplace(index, EncounterInfo(encounter_num, t, loc, alt, wind, d, v, p_free));
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
      pt.time_pred = std::min(-d/v, pt.time_pred);
      pt.v_max = std::max(-v, pt.v_max);
    }
    pt.wind_acc += Vector(wind);
    pt.num_wind ++;
    pt.wind = SpeedVector(pt.wind_acc.y/pt.num_wind, pt.wind_acc.x/pt.num_wind);
    pt.p_free *= p_free;
  }
}


static bool is_expired(const EncounterMapStore::EncounterInfo& info, const double time)
{
  if (time - info.time_end > EncounterMapStore::HYS_TRAIL) {
    return true;
  }
  if (time - info.time_start > EncounterMapStore::MAX_TRAIL) {
    return true;
  }
  return false;
}


double EncounterMapStore::erase_expired(const double time, std::list<AircraftModel> &group, const double distance_threshold,
                                        const bool debug)
{
  double time_close = 0;
  for (auto i = encounters.begin(), last = encounters.end(); i != last; ) {
    auto& info = i->second;
    if (is_expired(info, time)) {
      info.finalise();

      AircraftModel::write_encounter_open(info, distance_threshold);

      int id1;
      int id2;
      index_split(i->first, id1, id2);

      time_close += info.time_close;
      const double penalty = distance_threshold-info.d_min;

      // iterate over aircraft, write primary encounter aircraft
      for (auto&& a : group) {
        if ((a.idi == id1) || (a.idi == id2)) {
          a.write_encounter(info, a.idi == id1? id2: id1);
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
            a.write_encounter(info, -1);
          }
        }
      }

      if (debug) {
        // write_diagnostics_encounter(a,b,info);
        // write_diagnostics_encounter_turnmodelist(a,b,info);
      }

      AircraftModel::write_encounter_close(info);

      i = encounters.erase(i);
    } else {
      ++i;
    }
  }
  return time_close;
}

/////////////////////////////////////////////////////////////////////


void EncounterMapStore::write_diagnostics_encounter(const AircraftModel& a,
                                                    const AircraftModel& b,
                                                    const EncounterInfo& info) const
{
  std::ostringstream oss;
  oss << "encounter_"
      << std::setw(5) << std::setfill('0') << info.id
      << ".gp";
  std::ofstream fenc;
  fenc.open(oss.str());
  fenc << "id0='" << a.id << "'\n"
       << "id1='" << b.id << "'\n"
       << "tstart=" << info.time_start << "\n"
       << "tend=" << info.time_end << "\n";
  fenc.close();
}

void EncounterMapStore::write_diagnostics_encounter_turnmodelist(const AircraftModel& a,
                                                                 const AircraftModel& b,
                                                                 const EncounterInfo& info) const
{
  const TurnModeList tml_a = a.gen_turnmodelist(info);
  const TurnModeList tml_b = b.gen_turnmodelist(info);

  printf("\"%s\" \"%s\" %d %d ", a.id.c_str(), b.id.c_str(), a.idi, b.idi);
  printf("%g %g %d %g %g %g ", info.time_start, info.time_end, info.time_close,
         info.d_min, info.time_pred, info.v_max);
  printf("%g %g ", info.wind.bearing.Degrees(), info.wind.norm);
  printf("%.8g %.8g %g %05d ", info.origin.longitude.Degrees(), info.origin.latitude.Degrees(), info.alt, info.id);
  printf("\"");
  printf("%s",tml_a.string().c_str());
  printf("-");
  printf("%s",tml_b.string().c_str());
  printf("\"\n");
}

std::string EncounterMapStore::EncounterInfo::get_encounter_filename() const
{
  std::ostringstream oss;
  oss << "encounter_"
      << std::setw(5) << std::setfill('0') << id
      << ".json";
  return oss.str();
}
