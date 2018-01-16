/*
Copyright_License {

  XCSoar Glide Computer - http://www.xcsoar.org/
  Copyright (C) 2000-2016 The XCSoar Project
  A detailed list of copyright holders can be found in the file "AUTHORS".

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
}
*/

#include "Computer/Wind/Computer.hpp"
#include "Computer/CirclingComputer.hpp"
#include "Computer/Settings.hpp"
#include "Formatter/TimeFormatter.hpp"
#include "OS/Args.hpp"
#include "DebugReplay.hpp"
#include "Replay/CatmullRomInterpolator.hpp"

#include "Geo/Flat/FlatProjection.hpp"
#include "Geo/Flat/FlatPoint.hpp"
#include "Geo/Math.hpp"

#include <list>
#include <stdio.h>
#include <algorithm>

#include <unordered_map>

static GlidePolar glide_polar(0);
static CirclingSettings circling_settings;
static WindSettings wind_settings;

static const int TYP_TRAIL = 15;
static const int MAX_TRAIL = 3*TYP_TRAIL;
static const int HYS_TRAIL = 10;
double DISTANCE = 30;
int SCORE_BUFFER = 0;

struct TrailPoint {
  CatmullRomInterpolator::Record pos;
  GeoVector trk;
  CirclingMode turn_mode;
  bool within_time(const double t0, const double t1) const {
    return (pos.time >= t0) && (pos.time <= t1);
  }
};

class TurnModeList: public std::list<CirclingMode>
{
 public:
  void print(FILE* f) const {
    const char symbols[] = "GECX";
    /*
      CRUISE =          0
      POSSIBLE_CLIMB =  1
      CLIMB =           2
      POSSIBLE_CRUISE = 3
    */

    for (auto m = begin(); m != end(); ++m) {
      fprintf(f, "%c", symbols[(int)*m]);
    }
  }
};

class EncounterMapStore {
 public:

  struct EncounterInfo {
    double time_start;
    double time_end;
    unsigned time_close;
    double d_min;
    unsigned encounter_num;
    double time_pred;
    GeoPoint loc_min;
    SpeedVector wind;
    double alt;
    double v_max;
  };

  void update(const int id1, const int id2, const double t, const double d, const double v, const GeoPoint& loc, const SpeedVector &wind,
              const double alt)
  {
    const KeyType index = make_index(id1, id2);
    if (encounters.find(index) == encounters.end()) {
      EncounterInfo pt;
      pt.time_start = t;
      pt.time_close = 1;
      pt.time_end = t;
      pt.d_min = d;
      pt.encounter_num = encounter_num;
      pt.time_pred = (v<0)? -d/v: 0;
      pt.loc_min = loc;
      pt.wind = wind;
      pt.alt = alt;
      pt.v_max = -v;
      encounters[index] = pt;
      encounter_num++;
    } else {
      encounters[index].time_end = t;
      encounters[index].time_close++;
      if (d < encounters[index].d_min) {
        encounters[index].d_min = d;
        encounters[index].loc_min = loc;
      }
      if (v<0) {
        encounters[index].time_pred = std::min(-d/v, encounters[index].time_pred);
        encounters[index].v_max = std::max(-v, encounters[index].v_max);
      }
    }
  }

  bool purge_expired(const int id1, const int id2, const double time, EncounterInfo& pt)
  {
    const KeyType index = make_index(id1, id2);
    auto it = encounters.find(index);
    if (it != encounters.end()) {
      const bool expire_end = time - (it->second).time_end > HYS_TRAIL;
      const bool expire_start = time - (it->second).time_start > MAX_TRAIL;
      if (expire_start || expire_end) {
        pt = it->second;
        encounters.erase(it);
        return true;
      }
    }
    return false;
  }

 private:
  static unsigned encounter_num;

  typedef unsigned KeyType;
  typedef std::unordered_map<KeyType, EncounterInfo > EncounterMap;

  EncounterMap encounters;

  static KeyType make_index(const int id1, const int id2) {
    return (id1 << 8) + id2;
  }
};

unsigned EncounterMapStore::encounter_num = 0;


class MAircraft {
 public:
  MAircraft():
      interpolator(0.5) {
  }

  int init(Args& args) {

    live = false;

    const char* ptr = args.PeekNext();
    const char* ptr_end = ptr+strlen(ptr)-1;

    while (*ptr_end != '/') {
      ptr_end--;
    }
    ptr = ptr_end;

    while (*ptr != '_') {
      ptr++;
    }
    ptr++;
    sscanf(ptr,"%[^.]", id);
    idi = num_aircraft++;

    replay = CreateDebugReplay(args);
    if (replay == NULL)
      return EXIT_FAILURE;

    wind_settings.SetDefaults();
    wind_settings.zig_zag_wind = false;
    circling_computer.Reset();
    wind_computer.Reset();
    last.Clear();
    ok = true;
    penalty = 0;
    flight_time_start = 0;
    flight_time_end = 0;
    flight_alt_diff = 0;
    flight_alt_start = 0;

    char buf[80];
    sprintf(buf,"trace-%s.txt", id);
    ftrace = fopen(buf,"w");
    return 0;
  }

  ~MAircraft() {
    if (replay)
      delete replay;
  }

  bool advance() {

    if (replay->Next()) {
      const MoreData &basic = replay->Basic();
      const DerivedInfo &calculated = replay->Calculated();

      if (calculated.flight.flying) {
        if (flight_time_start==0) {
          flight_time_start = basic.time;
          flight_alt_start = basic.gps_altitude;
          flight_alt_diff = basic.gps_altitude-basic.baro_altitude;
          // printf("\"%s\" %d %g %g %g jon\n", id, idi, basic.time, basic.baro_altitude, basic.gps_altitude);
        }
        flight_time_end = basic.time;
      } else if (flight_time_start == 0) {
      }

      interpolator.Update(basic.time, basic.location, basic.gps_altitude, basic.baro_altitude + flight_alt_diff);

      fprintf(ftrace,"%d %g %g %g %g %g\n", (int)basic.time,
              basic.location.longitude.Degrees(),
              basic.location.latitude.Degrees(),
              basic.gps_altitude,
              basic.baro_altitude,
              basic.baro_altitude + flight_alt_diff);

      circling_computer.TurnRate(replay->SetCalculated(),
                                 basic, calculated.flight);
      circling_computer.Turning(replay->SetCalculated(),
                                basic,
                                calculated.flight,
                                circling_settings);

      wind_computer.Compute(wind_settings, glide_polar, basic,
                            replay->SetCalculated());

      if (calculated.estimated_wind_available.Modified(last)) {
        TCHAR time_buffer[32];
        FormatTime(time_buffer, replay->Basic().time);
      }

      last = calculated.estimated_wind_available;
    } else {
      if (ok) {
        fclose(ftrace);
      }
      ok = false;
    }
    return ok;
  }

  void Interpolate(const double t) {
    interp_loc_last = interp_loc;
    interp_loc = interpolator.Interpolate(t);

    TrailPoint p;
    p.pos = interp_loc;
    p.trk = interpolator.GetVector(t);
    p.turn_mode = replay->Calculated().turn_mode;

    trail.push_back(p);
    while (trail.size() > MAX_TRAIL+2) {
      trail.pop_front();
    }

    valid = true;
  }

  TurnModeList gen_turnmodelist(const EncounterMapStore::EncounterInfo &info) const
  {
    const double t0 = info.time_start-TYP_TRAIL;
    const double t1 = info.time_start+1;

    bool started = false;
    TurnModeList tml;
    for (auto it = trail.begin(); it != trail.end(); ++it) {
      bool do_append = false;
      if (it->within_time(t0, t1)) {
        do_append = !started;
        if (started && (tml.back() != it->turn_mode)) {
          do_append = true;
        }
        started = true;
        if (do_append)
          tml.push_back(it->turn_mode);
      }
    }
    return tml;
  }

  void WriteTrail(bool first,
                  const SpeedVector &wind,
                  const EncounterMapStore::EncounterInfo& info) const {

    const double t0 = info.time_start - TYP_TRAIL;
    const double t1 = info.time_end + HYS_TRAIL;
    const double tzero = info.time_start;

    char buf[80];
    sprintf(buf, "encounter_%05d-%d.txt", info.encounter_num, !first);
    FILE* fenc = fopen(buf,"w");

    fprintf(fenc, "# id time_start time_end time_close d_min time_pred v_max\n");
    fprintf(fenc, "# \"%s\" %g %g %d %g %g %g\n", id, info.time_start, info.time_end, info.time_close, info.d_min, info.time_pred, info.v_max);
    fprintf(fenc, "#\n");
    fprintf(fenc, "# t x y alt_baro alt_gps v hdg turnrate bank turn_mode\n");

    const GeoPoint tp1 = FindLatitudeLongitude(info.loc_min,
                                         wind.bearing,
                                         wind.norm);
    const GeoPoint traildrift = tp1 - info.loc_min;

    FlatProjection proj(info.loc_min);
    const double scale = proj.GetApproximateScale();
    double a_last = 500;

    for (auto it = trail.begin(); it != trail.end(); ++it) {

      const double vx = it->trk.distance * it->trk.bearing.sin() + wind.norm * wind.bearing.sin();
      const double vy = it->trk.distance * it->trk.bearing.cos() + wind.norm * wind.bearing.cos();
      const double a = atan2(vx, vy);

      if ((it != trail.begin()) && it->within_time(t0, t1)) {

        const GeoPoint loc_drift = it->pos.location.Parametric(traildrift,
                                                               it->pos.time-tzero);
        const double v = sqrt(vx*vx+vy*vy);

        double r = (a-a_last);
        while (r>M_PI) {
          r-= 2*M_PI;
        }
        while (r<-M_PI) {
          r+= 2*M_PI;
        }
        double bank_angle = 180/M_PI*atan(r * v / 9.81);

        const FlatPoint fp = proj.ProjectFloat(loc_drift)*scale;

        fprintf(fenc, "%g %g %g %g %g %g %g %g %g %d\n", it->pos.time-tzero, fp.x, fp.y,
                it->pos.baro_altitude, it->pos.gps_altitude, v, 180/M_PI*a, 180/M_PI*r, bank_angle, (int)it->turn_mode);
      }
      a_last = a;
    }
    fclose(fenc);
  }

  CatmullRomInterpolator interpolator;
  CirclingComputer circling_computer;
  WindComputer wind_computer;
  Validity last;
  DebugReplay *replay;
  bool ok;
  bool valid;
  bool live;
  char id[8];
  int idi;
  double penalty;
  double flight_time_start;
  double flight_time_end;
  double flight_alt_start;
  double flight_alt_diff;

  FILE* ftrace;
  CatmullRomInterpolator::Record interp_loc;
  CatmullRomInterpolator::Record interp_loc_last;
  GeoVector interp_v;

  std::list<TrailPoint> trail;

  static int num_aircraft;

};

int MAircraft::num_aircraft = 0;

static double distance_horiz(const CatmullRomInterpolator::Record& a,
                      const CatmullRomInterpolator::Record& b)
{
  return std::max(a.location.Distance(b.location), 0.);
}

static double distance_vert(const CatmullRomInterpolator::Record& a,
                     const CatmullRomInterpolator::Record& b)
{
  return std::max(fabs(a.baro_altitude - b.baro_altitude), 0.);
}

static double distance(const CatmullRomInterpolator::Record& a,
                const CatmullRomInterpolator::Record& b)
{
  double d_horiz = distance_horiz(a, b);
  double d_vert = distance_vert(a, b);
  return sqrt(d_horiz*d_horiz+d_vert*d_vert);
}

int main(int argc, char **argv)
{
  /* last 2 arguments are the proximity distance to use.
        30 metres is good
        61 metres is "legal" in Australia
  */
  // get the distance
  DISTANCE = std::stoi(argv[argc-2]);
  // get the penalty buffer
  SCORE_BUFFER = std::stoi(argv[argc-1]);

  Args args(argc-2, argv, "DRIVER FILE");
  int retval;
  std::list<MAircraft*> group;
  EncounterMapStore encounter_store;

  do {
    MAircraft* ac = new MAircraft;
    group.push_back(ac);
    retval = ac->init(args);
  } while (!args.IsEmpty());

  args.ExpectEnd();

  // find times
  double all_t_start = 0;
  double all_t_end = 0;
  double t_start = 0;
  double t_end = 0;

  int num_flightsecs = 0;
  double acc_spacing_global = 0;
  int num_spacing_global = 0;

  printf("# id0 id1 idi0 idi1 time_start time_end time_close d_min time_pred v_max wind_bearing wind_mag lon lat alt eid pattern\n");

  for (auto i = group.begin(); i!= group.end(); ++i) {
    MAircraft& a = *(*i);
    while (!a.interpolator.Ready() && !a.replay->Calculated().flight.flying && a.ok)
      a.advance();
    if (!a.ok)
      continue;
    double t_this = a.replay->Basic().time;
    if ((t_start==0) || (t_this < t_start)) {
      t_start = t_this;
    }
    if (!a.interpolator.Ready()) {
      a.valid = false;
    } else {
      t_end = std::max(t_end, a.interpolator.GetMaxTime());
      a.valid = false;
    }
  }

  if (t_start == 0) {
    exit(EXIT_FAILURE);
  }

  int time_close = 0;
  double alt_max = 0;
  double alt_min = 0;
  bool first = true;
  GeoPoint loc_general;

  for (double t= t_start; t<= t_end; t+= 1.0) {
    all_t_end = std::max(t_end, all_t_end);

    // advance time
    for (auto i = group.begin(); i!= group.end(); ++i) {
      MAircraft& a = *(*i);
      a.valid = false;
      if (!a.ok || !a.interpolator.Ready())
        continue;
      if (t < a.interpolator.GetMinTime())
        continue;
      if (a.interpolator.NeedData(t)) {
        a.advance();
        t_end = std::max(t_end, a.interpolator.GetMaxTime());
      }
      if (!a.replay->Calculated().flight.flying)
        continue;

      a.Interpolate(t);

      if (a.interp_loc.baro_altitude < 400) {
        a.valid = false;
      } else {
        a.live = true;
        if (first) {
          alt_max = a.interp_loc.baro_altitude;
          alt_min = alt_max;
          first = false;
        } else {
          alt_max = std::max(alt_max, a.interp_loc.baro_altitude);
          alt_min = std::min(alt_min, a.interp_loc.baro_altitude);
        }
      }
    }

    // perform checks
    SpeedVector wind;

    for (auto i = group.begin(); i!= group.end(); ++i) {
      const MAircraft& a = *(*i);
      if (!a.live || !a.valid)
        continue;

      if (all_t_start == 0) {
        all_t_start = t_start;
        loc_general = a.interp_loc.location;
      }

      num_flightsecs++;

      if (a.replay->Calculated().estimated_wind_available) {
        wind = a.replay->Calculated().estimated_wind;
      }

      auto j = i; j++;
      for (; j!= group.end(); ++j) {
        const MAircraft& b = *(*j);
        if (!b.live || !b.valid)
          continue;

        if (!a.replay->Calculated().estimated_wind_available &&
            b.replay->Calculated().estimated_wind_available) {
          wind = b.replay->Calculated().estimated_wind;
        }

        const double d_vert = distance_vert(a.interp_loc, b.interp_loc);
        /*
        if (d_vert > DISTANCE/2)
          continue;
        */

        const double d_horiz = distance_horiz(a.interp_loc, b.interp_loc);
        /*
        if (d_horiz > DISTANCE)
          continue;
        */

        const double d_abs = sqrt(d_horiz*d_horiz+d_vert*d_vert);

        acc_spacing_global+= d_abs;
        num_spacing_global++;

        if (d_vert > DISTANCE/2)
          continue;

        if (d_abs > DISTANCE)
          continue;

        const double d_last = distance(a.interp_loc_last, b.interp_loc_last);
        const double v = (d_abs-d_last);

        const GeoPoint center = a.interp_loc.location.Interpolate(b.interp_loc.location, 0.5);

        encounter_store.update(a.idi, b.idi, t, d_abs, v, center, wind, a.interp_loc.baro_altitude);

      }
    }

    for (auto i = group.begin(); i!= group.end(); ++i) {
      MAircraft& a = *(*i);
      auto j = i; j++;
      for (; j!= group.end(); ++j) {
        MAircraft& b = *(*j);
        EncounterMapStore::EncounterInfo info;
        if (encounter_store.purge_expired(a.idi, b.idi, t, info)) {
          a.WriteTrail(true, wind, info);
          b.WriteTrail(false, wind, info);

          const double penalty = DISTANCE-info.d_min;
          a.penalty += penalty;
          b.penalty += penalty;

          time_close+= info.time_close;

          {
            char buf[80];
            sprintf(buf, "encounter_%05d.gp", info.encounter_num);
            FILE* fenc = fopen(buf,"w");

            fprintf(fenc, "id0='%s'\n", a.id);
            fprintf(fenc, "id1='%s'\n", b.id);
            fprintf(fenc, "tstart=%d\n", (int)info.time_start);
            fprintf(fenc, "tend=%d\n", (int)info.time_end);
            fclose(fenc);
          }

          {
            const TurnModeList tml_a = a.gen_turnmodelist(info);
            const TurnModeList tml_b = b.gen_turnmodelist(info);

            printf("\"%s\" \"%s\" %d %d ", a.id, b.id, a.idi, b.idi);
            printf("%g %g %d %g %g %g ", info.time_start, info.time_end, info.time_close,
                   info.d_min, info.time_pred, info.v_max);
            printf("%g %g ", info.wind.bearing.Degrees(), info.wind.norm);
            printf("%g %g %g %05d ", info.loc_min.longitude.Degrees(), info.loc_min.latitude.Degrees(), info.alt, info.encounter_num);
            printf("\"");
            tml_a.print(stdout);
            printf("-");
            tml_b.print(stdout);
            printf("\"\n");
          }

        }
      }
    }

  }

  //////////////
  {
    // Create a file of "penalties" for the day - this is the sum of the maximum distance of each incursion
    FILE* fout = fopen("penalty.txt","w");
    fprintf(fout, "# id idi penalty\n");
    int num_aircraft = 0;
    for (auto i = group.begin(); i!= group.end(); ++i) {
      const MAircraft& a = *(*i);
      if (a.live) {
        num_aircraft++;
        fprintf(fout, "\"%s\" %d %f\n", a.id, a.idi, a.penalty);
      }
    }
    fclose(fout);

    // Create the penalty file for the scorer to apply
    fout = fopen("scoring_penalty.txt","w");
    fprintf(fout, "# comp_id penalty\n");
    int score;
    for (auto i = group.begin(); i!= group.end(); ++i) {
      const MAircraft& a = *(*i);
      if (a.live) {
        // penalty is the sum of the "penalties" minus the buffer
        score = (int)a.penalty - SCORE_BUFFER;
        // Only if we have a postitive penalty output it.
        if(score > 0) fprintf(fout, "\"%s\" %d\n", a.id, score);
      }
    }
    fclose(fout);

    fout = fopen("flighttime.txt","w");
    fprintf(fout, "# id idi flight_time_start flight_time_end flight_alt_start flight_alt_diff\n");
    for (auto i = group.begin(); i!= group.end(); ++i) {
      const MAircraft& a = *(*i);
      if (a.live) {
        fprintf(fout, "\"%s\" %d %f %f %f %f\n", a.id, a.idi, a.flight_time_start, a.flight_time_end, a.flight_alt_start, a.flight_alt_diff);
      }
    }
    fclose(fout);

    double av_spacing = 1000000;
    if (num_spacing_global)
      av_spacing= acc_spacing_global/num_spacing_global;

    fout = fopen("summary.txt","w");
    fprintf(fout, "# num_flightsecs num_aircraft time_close alt_max av_spacing\n");
    fprintf(fout, "%d %d %d %d %g\n", num_flightsecs, num_aircraft, time_close, (int)alt_max, av_spacing);
    fclose(fout);

    fout = fopen("proximity.gp","w");
    fprintf(fout, "lat0=%g\n", loc_general.latitude.Degrees());
    fprintf(fout, "lon0=%g\n", loc_general.longitude.Degrees());
    fprintf(fout, "alt_max=%g\n", alt_max);
    fprintf(fout, "t_start=%g\n", all_t_start);
    fprintf(fout, "t_end=%g\n", all_t_end);
    fclose(fout);
  }

  exit(retval);
}
