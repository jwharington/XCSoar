// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "FlightCollection.hpp"
#include "Geo/Geoid.hpp"
#include <iostream>
#include <algorithm>
#include <iomanip>

using namespace MultiAircraft;

bool FlightCollection::load_files(Args &args)
{
  do
  {
    group.emplace_back();
    if (!group.back().init(args))
    {
      return false;
    }
  } while (!args.IsEmpty());
  return group.size() > 0;
}

bool FlightCollection::advance_to_start()
{
  num_flightsecs = 0;
  overall_wind_x = 0;
  overall_wind_y = 0;
  overall_wind_samples = 0;

  size_t i = 0;
  for (auto &a : group)
  {
    a.reset();
    a.advance_to_start(t_start, t_end);
    std::cout << "  aircraft " << (++i) << "/" << group.size()
              << " [" << a.id << "] ready" << std::endl;
  }
  return (t_start.IsDefined());
}

void FlightCollection::advance_to_time(const TimeStamp t)
{
  // advance time
  Vector wind_acc(0, 0);
  int num_wind = 0;

  for (auto &&a : group)
  {
    if (!a.advance_to_time(t, t_end))
      continue;

    num_flightsecs++;
    if (a.Calculated().estimated_wind_available)
    {
      const Vector wind_xy = Vector(a.Calculated().estimated_wind);
      wind_acc += wind_xy;
      num_wind++;
    }
    if (first)
    {
      alt_max = a.interp_loc.gps_altitude;
      alt_min = alt_max;
      first = false;
    }
    else
    {
      alt_max = std::max(alt_max, a.interp_loc.gps_altitude);
      alt_min = std::min(alt_min, a.interp_loc.gps_altitude);
    }
  }

  // set wind to average if absent
  if (num_wind)
  {
    wind_acc.x /= num_wind;
    wind_acc.y /= num_wind;
    overall_wind_x += wind_acc.x;
    overall_wind_y += wind_acc.y;
    ++overall_wind_samples;
    const SpeedVector wind_avg = SpeedVector(wind_acc.y, wind_acc.x);
    for (auto &&a : group)
    {
      a.set_wind_if_not_available(wind_avg);
    }
    // std::cout << wind_acc.y << " " << wind_acc.x << "\n";
  }
}

void FlightCollection::finalise()
{
  Averager alt_start;
  const std::size_t total_aircraft = group.size();
  std::size_t processed_aircraft = 0;

  if (total_aircraft > 0)
    std::cout << "  [finalise] aircraft traces: 0/" << total_aircraft << "\n";

  for (auto &&a : group)
  {
    a.finalise(alt_start);
    ++processed_aircraft;
    if (processed_aircraft == total_aircraft || processed_aircraft % 10 == 0)
      std::cout << "  [finalise] aircraft traces: " << processed_aircraft << "/" << total_aircraft << "\n";
  }
  alt_start.calculate();
  alt_start_av = alt_start.get_avg();
  loc_general = calc_av_flight_loc_start(false);
}

static void write_header(const std::list<AircraftModel> &group)
{
  std::size_t max_len = 0;
  for (auto &&a : group)
  {
    max_len = std::max(max_len, a.id.length());
  }
  for (std::size_t i = 0; i < max_len; ++i)
  {
    for (auto &&a : group)
    {
      if (i < a.id.length())
      {
        std::cout << a.id.substr(i, 1);
      }
      else
      {
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

  std::cout << "Initialising " << group.size() << " aircraft..." << std::endl;
  if (!advance_to_start())
  {
    printf("Error! advance to start\n");
    return false;
  }
  std::cout << "Start t=" << (int)t_start.ToDuration().count()
            << " end t=" << (int)t_end.ToDuration().count()
            << " (" << (int)std::ceil((t_end - t_start).count()) << "s)" << std::endl;

  loc_general = calc_av_flight_loc_start();
  proj = FlatProjection(loc_general);

  /////////////////
  first = true;
  const auto max_elapsed_seconds = std::max(0, (int)std::ceil((t_end - t_start).count()));
  const auto elapsed_width = std::to_string(max_elapsed_seconds).size();
  for (TimeStamp t = t_start; t <= t_end; t += FloatDuration(1))
  {
    if (!process(t))
    {
      printf("Error! processing at t=%d\n", (int)t.ToDuration().count());
      return false;
    }
    if (tcount++ % display_interval)
    {
      continue;
    }

    const auto elapsed_seconds = std::max(0, (int)std::ceil((t - t_start).count()));
    std::cout << "t=" << std::setw((int)elapsed_width) << elapsed_seconds << " ";
    for (auto &&a : group)
    {
      std::cout << get_symbol(a);
      a.mark = false;
    }
    std::cout << std::endl;
  }
  std::cout << "Finalising outputs..." << std::endl;
  finalise();
  std::cout << "Done." << std::endl;
  /////////////////

  return true;
}

bool FlightCollection::process(const TimeStamp t)
{
  advance_to_time(t);
  return true;
}

std::string FlightCollection::get_symbol(const AircraftModel &m) const
{
  return m.get_symbol();
}
///////////////////////////////////////////////////////////////////////////

boost::json::object FlightCollection::record_summary() const
{
  const double geoid_sep = EGM96::LookupSeparation(loc_general);
  Averager baro_error;
  const bool wind_available = overall_wind_samples > 0;
  const SpeedVector overall_wind = wind_available
                                       ? SpeedVector(overall_wind_y / overall_wind_samples,
                                                     overall_wind_x / overall_wind_samples)
                                       : SpeedVector();

  boost::json::array flight_summary;
  for (auto &&a : group)
  {
    flight_summary.emplace_back(a.record_summary(alt_start_av, geoid_sep, baro_error));
  }

  boost::json::object location_summary = {
      {"latitude", loc_general.latitude.Degrees()},
      {"longitude", loc_general.longitude.Degrees()},
      {"alt_max", alt_max},
      {"alt_start_av", alt_start_av},
      {"t_start", (int)t_start.ToDuration().count()},
      {"t_end", (int)t_end.ToDuration().count()},
      {"wind_speed", wind_available ? boost::json::value(overall_wind.norm) : boost::json::value(nullptr)},
      {"wind_bearing", wind_available ? boost::json::value(overall_wind.bearing.Degrees()) : boost::json::value(nullptr)},
      {"geoid_sep", geoid_sep},
  };
  if (!baro_error.empty())
  {
    baro_error.calculate();
    location_summary.emplace("baro_error", sqrt(baro_error.get_avg()));
  }

  return {
      {"location", location_summary},
      {"records", flight_summary},
  };
}

const GeoPoint FlightCollection::calc_av_flight_loc_start(const bool first_pass) const
{
  GeoPoint center(Angle::Native(0), Angle::Native(0));
  int n = 0;
  for (auto &&a : group)
  {
    if (a.flight_present(first_pass))
    {
      n++;
    }
  }
  if (n)
  {
    const double inv_n = 1.0 / n;
    for (auto &&a : group)
    {
      const GeoPoint &p = a.get_flight_loc_start();
      if (a.flight_present(first_pass))
      {
        center += p * inv_n;
      }
    }
  }
  return center;
}
