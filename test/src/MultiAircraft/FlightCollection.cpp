#include "FlightCollection.hpp"
#include "Geo/Geoid.hpp"
#include <iostream>
#include <algorithm>

using namespace MultiAircraft;

bool FlightCollection::load_files(Args& args)
{
  do {
    group.emplace_back();
    if (!group.back().init(args)) {
      return false;
    }
  } while (!args.IsEmpty());
  return true;
}


bool FlightCollection::advance_to_start()
{
  num_flightsecs = 0;

  for (auto&& a : group) {
    a.reset();
    a.advance_to_start(t_start, t_end);
  }
  return (t_start != 0);
}


void FlightCollection::advance_to_time(const int t)
{
  // advance time
  Vector wind_acc (0,0);
  int num_wind = 0;

  for (auto&& a : group) {
    if (!a.advance_to_time(t, t_end))
      continue;

    num_flightsecs++;
    if (a.Calculated().estimated_wind_available) {
      const Vector wind_xy = Vector(a.Calculated().estimated_wind);
      wind_acc += wind_xy;
      num_wind++;
    }
    if (first) {
      alt_max = a.interp_loc.gps_altitude;
      alt_min = alt_max;
      first = false;
    } else {
      alt_max = std::max(alt_max, a.interp_loc.gps_altitude);
      alt_min = std::min(alt_min, a.interp_loc.gps_altitude);
    }
  }

  // set wind to average if absent
  if (num_wind) {
    wind_acc.x /= num_wind;
    wind_acc.y /= num_wind;
    const SpeedVector wind_avg = SpeedVector(wind_acc.y, wind_acc.x);
    for (auto&& a : group) {
      a.set_wind_if_not_available(wind_avg);
    }
  }
}


void FlightCollection::finalise()
{
  Averager alt_start;
  for (auto&& a : group) {
    a.finalise(alt_start);
  }
  alt_start.calculate();
}

static void write_header(const std::list<AircraftModel>& group)
{
  std::size_t max_len = 0;
  for (auto&& a : group) {
    max_len = std::max(max_len, a.id.length());
  }
  for (std::size_t i=0; i< max_len; ++i) {
    for (auto&& a : group) {
      if (i< a.id.length()) {
        std::cout << a.id.substr(i,1);
      } else {
        std::cout << " ";
      }
    }
    std::cout << "\n";
  }
}


bool FlightCollection::run()
{
  int tcount = 0;

  write_header(group);

  if (!advance_to_start()) {
    printf("Error! advance to start\n");
    return false;
  }

  loc_general = calc_av_flight_loc_start();
  proj = FlatProjection(loc_general);

  /////////////////
  first = true;
  for (int t= t_start; t<= t_end; t+= 1) {
    if (!process(t)) {
      printf("Error! processing at t=%d\n", t);
      return false;
    }
    if (tcount++ % display_interval) {
      continue;
    }

    for (auto&& a : group) {
      std::cout << get_symbol(a);
      a.mark = false;
    }
    std::cout << "\n";
  }
  finalise();
  /////////////////

  return true;
}


bool FlightCollection::process(const int t)
{
  advance_to_time(t);
  return true;
}


std::string FlightCollection::get_symbol(const AircraftModel& m) const
{
  return m.get_symbol();
}
///////////////////////////////////////////////////////////////////////////


void FlightCollection::write_diagnostics() const
{
  const double geoid_sep = EGM96::LookupSeparation(loc_general);
  Averager baro_error;

  FILE* fout = fopen("flighttime.txt","w");
  fprintf(fout, "# id idi flight_time_start flight_time_end flight_alt_start flight_alt_end baro_error fr_info fr_id alt_anomaly avg_timestep\n");
  for (auto&& a : group) {
    a.write_diagnostics(fout, alt_start_av, geoid_sep, baro_error);
  }
  fclose(fout);

  fout = fopen("proximity.gp","w");
  fprintf(fout, "lat0=%.8g\n", loc_general.latitude.Degrees());
  fprintf(fout, "lon0=%.8g\n", loc_general.longitude.Degrees());
  fprintf(fout, "alt_max=%g\n", alt_max);
  fprintf(fout, "t_start=%d\n", t_start);
  fprintf(fout, "t_end=%d\n", t_end);
  fprintf(fout, "geoid_sep=%g\n", geoid_sep);

  if (!baro_error.empty()) {
    baro_error.calculate();
    fprintf(fout, "baro_error=%f\n", sqrt(baro_error.get_avg()));
  }
  fclose(fout);
}

const GeoPoint FlightCollection::calc_av_flight_loc_start() const
{
  GeoPoint center(Angle::Native(0),Angle::Native(0));
  const double n = 1.0/group.size();
  for (auto&& a : group) {
    const GeoPoint &p = a.get_flight_loc_start();
    center += p*n;
  }
  return center;
}
