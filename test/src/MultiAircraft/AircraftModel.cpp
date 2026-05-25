// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "AircraftModel.hpp"
#include "DetectMiss.hpp"
#include "Formatter/TimeFormatter.hpp"
#include "Math/Vector.hpp"
#include "TraceWriter.hpp"
#include <cctype>
#include <fstream>
#include <iomanip> // std::setprecision
#include <iostream>
#include <sstream>
#include <string_view>

using namespace MultiAircraft;

int AircraftModel::num_aircraft = 0;
TimeStamp AircraftModel::first_launch = TimeStamp::Undefined();
double AircraftModel::MIX_BARO = 0.5;
int AircraftModel::filter_type = 1;
FloatDuration AircraftModel::reconstruction_pre_buffer{5};
bool AircraftModel::write_trace_files = true;
bool AircraftModel::keep_full_trail = false;
GlidePolar AircraftModel::glide_polar(0);
WindSettings AircraftModel::wind_settings;
CirclingSettings AircraftModel::circling_settings;

static std::string
MakeSelectorSlug(std::string_view selector)
{
  std::string slug;
  slug.reserve(selector.size());

  bool prev_underscore = false;
  for (unsigned char ch : selector)
  {
    if (std::isalnum(ch))
    {
      slug.push_back((char)std::tolower(ch));
      prev_underscore = false;
    }
    else if (!prev_underscore)
    {
      slug.push_back('_');
      prev_underscore = true;
    }
  }

  while (!slug.empty() && slug.back() == '_')
    slug.pop_back();

  return slug;
}

bool AircraftModel::init(Args &args)
{
  if (num_aircraft == 0)
  {
    wind_settings.SetDefaults();
    wind_settings.zig_zag_wind = false;
  }

  std::string_view path_spec = args.PeekNext();
  const std::size_t hash = path_spec.find('#');
  const std::string_view path = hash == std::string_view::npos
                                    ? path_spec
                                    : path_spec.substr(0, hash);
  const std::string_view selector = hash == std::string_view::npos
                                        ? std::string_view{}
                                        : path_spec.substr(hash + 1);

  const std::size_t slash = path.find_last_of('/');
  const std::string_view basename = slash == std::string_view::npos
                                        ? path
                                        : path.substr(slash + 1);
  const std::size_t underscore = basename.find('_');
  const std::size_t start = underscore == std::string_view::npos ? 0 : underscore + 1;
  const std::size_t dot = basename.find('.', start);
  const std::size_t length = dot == std::string_view::npos
                                 ? basename.size() - start
                                 : dot - start;
  id = std::string(basename.substr(start, length));

  if (!selector.empty())
  {
    const std::string selector_slug = MakeSelectorSlug(selector);
    if (!selector_slug.empty())
    {
      id += "_";
      id += selector_slug;
    }
  }

  idi = num_aircraft++;
  fr_info.clear();
  fr_id.clear();

  replay = CreateDebugReplay(args);
  if (replay == NULL)
  {
    replay_ok = false;
    return false;
  }

  reset();
  return true;
}

std::string AircraftModel::get_symbol() const
{
  if (!replay->Calculated().flight.flying)
  {
    return std::string(" ");
  }
  else if (!valid)
  {
    return std::string("~");
  }
  else if (!live)
  {
    return std::string("?");
  }
  else
  {
    return std::string(".");
  }
}

bool AircraftModel::aliased(const AircraftModel &other) const
{
  const MoreData &basic = replay->Basic();
  const MoreData &other_basic = other.replay->Basic();
  return (basic.time == other_basic.time) &&
         (basic.location == other_basic.location) &&
         (basic.gps_altitude == other_basic.gps_altitude);
}

bool AircraftModel::advance()
{
  if (replay->Next())
  {
    const MoreData &basic = replay->Basic();
    const DerivedInfo &calculated = replay->Calculated();
    if (!basic.location_available || !basic.gps_altitude_available)
    {
      return true;
    }
    h_acc = replay->GetHAccuracy();
    if (calculated.flight.flying)
    {
      if (!first_launch.IsDefined())
        first_launch = basic.time;

      fr_info = replay->GetTypeInfo();
      fr_id = replay->GetIdentifier();

      json_trace.emplace("id", id);
      json_trace.emplace("fr_info", fr_info);
      json_trace.emplace("fr_id", fr_id);
      json_trace.emplace("trace", boost::json::array());

      if (!flight_time_start.IsDefined())
      {

        flight_loc_start = basic.location;
        flight_time_start = basic.time;
        flight_date_utc_start = basic.date_time_utc;

        if (alt_start.empty())
        {
          alt_start.add(basic.gps_altitude);
        }
      }
      else if ((basic.ground_speed < 20) && (basic.location.Distance(flight_loc_start) < 2500.0))
      {
        alt_end.add(basic.gps_altitude);
        flight_time_end = basic.time;
      }

      flight_num_records++;
    }
    else if (flight_time_start.IsDefined())
    {
      alt_end.add(basic.gps_altitude);
    }
    else
    {
      baro_offset = basic.gps_altitude - basic.baro_altitude;
      alt_start.add(basic.gps_altitude);
    }

    double mix;
    const double baro_altitude = update_baro_altitude(mix);

    interpolator.Update(basic.time, basic.location, basic.gps_altitude, baro_altitude);

    circling_computer.TurnRate(replay->SetCalculated(),
                               basic, calculated.flight);
    circling_computer.Turning(replay->SetCalculated(),
                              basic,
                              calculated.flight,
                              circling_settings);

    wind_computer.Compute(wind_settings, glide_polar, basic,
                          replay->SetCalculated());
    return true;
  }
  return false;
}

void AircraftModel::Interpolate(const TimeStamp t, const SpeedVector &wind)
{
  interp_loc_last = interp_loc;
  interp_loc = interpolator.Interpolate(t);

  if (trail.empty())
  {
    trail.emplace_back(interp_loc, interpolator.GetVector(t), replay->Calculated().turn_mode, replay->GetHAccuracy(), interpolator.IsActual(t));
    return;
  }
  const TrailPoint &prev = trail.back();
  trail.emplace_back(interp_loc, interpolator.GetVector(t), replay->Calculated().turn_mode, replay->GetHAccuracy(), interpolator.IsActual(t));
  TrailPoint &now = trail.back();
  now.update_reconstruction(prev, wind);
  euler = EulerAngles(now.bank_angle, now.pitch_angle, now.yaw_angle);

  const size_t retained_trail =
      (EncounterMapStore::MAX_TRAIL_FACTOR + 1) * EncounterMapStore::TYP_TRAIL +
      EncounterMapStore::HYS_TRAIL + 2;
  while (!keep_full_trail && trail.size() > retained_trail)
  {
    trail.pop_front();
  }

  valid = true;

  boost::json::object tp({{"t", (int)t.ToDuration().count()},
                          {"longitude", interp_loc.location.longitude.Degrees()},
                          {"latitude", interp_loc.location.latitude.Degrees()},
                          {"gps_altitude", interp_loc.gps_altitude}});
  if (json_trace.find("trace") != json_trace.end())
  {
    json_trace.at("trace").as_array().emplace_back(tp);
  }
}

bool AircraftModel::flight_present(const bool first_pass) const
{
  return flight_time_start.IsDefined() && (first_pass || (flight_time_end > flight_time_start));
}

void AircraftModel::advance_to_start(TimeStamp &t_start, TimeStamp &t_end)
{
  replay->SetCalculated().estimated_wind = SpeedVector();

  while (replay_ok && (!interpolator.Ready() || !replay->Calculated().flight.flying || !flight_time_start.IsDefined()))
    replay_ok = advance();

  if (!replay_ok)
    return;

  const TimeStamp t_this = replay->Basic().time;

  if (!t_start.IsDefined() || (t_this < t_start))
  {
    t_start = t_this;
  }
  if (!interpolator.Ready())
  {
    valid = false;
  }
  else
  {
    t_end = std::max(t_end, interpolator.GetMaxTime());
  }
}

bool AircraftModel::advance_to_time(const TimeStamp t, TimeStamp &t_end)
{
  valid = false;
  if (!interpolator.Ready())
    return false;
  if (t < interpolator.GetMinTime())
    return false;
  while (interpolator.NeedData(t) && replay_ok)
  {
    replay_ok = advance();
    t_end = std::max(t_end, interpolator.GetMaxTime());
  }
  Interpolate(t, replay->Calculated().estimated_wind);
  if (!replay->Calculated().flight.flying)
  {
    valid = false;
    return false;
  }

  live = true;

  return valid && live;
}

boost::json::object AircraftModel::record_summary(const double alt_start_av, const double geoid_sep,
                                                  Averager &all_baro_error) const
{
  if (live)
  {
    const double alt_anomaly = geoid_sep != 0 ? (alt_start.get_avg() - alt_start_av) / geoid_sep : 0;
    const double avg_timestep = flight_num_records > 1 ? std::max(0.0, (flight_time_end - flight_time_start - FloatDuration{1.0}).count() / (flight_num_records - 1)) : 0;
    const bool wind_available = Calculated().estimated_wind_available.IsValid();
    const auto &wind = Calculated().estimated_wind;
    all_baro_error.add(baro_error);

    return {
        {"id", id},
        {"idi", idi},
        {"flight_time_start", (int)flight_time_start.ToDuration().count()},
        {"flight_time_end", (int)flight_time_end.ToDuration().count()},
        {"flight_alt_start", alt_start.get_avg()},
        {"flight_alt_end", alt_end.get_avg()},
        {"baro_error", sqrt(baro_error.get_avg())},
        {"fr_info", fr_info},
        {"fr_id", fr_id},
        {"alt_anomaly", alt_anomaly},
        {"wind_speed", wind_available ? boost::json::value(wind.norm) : boost::json::value(nullptr)},
        {"wind_bearing", wind_available ? boost::json::value(wind.bearing.Degrees()) : boost::json::value(nullptr)},
        {"avg_timestep", avg_timestep},
    };
  }
  return {};
}

void AircraftModel::finalise(Averager &all_alt_start)
{
  if (!live)
    return;
  if (write_trace_files)
  {
    std::ofstream json_file(get_trace_filename());

    char date_buffer[32];
    FormatISO8601(date_buffer, flight_date_utc_start);
    json_trace.emplace("date", date_buffer);

    json_file << boost::json::serialize(json_trace);
  }

  alt_start.calculate();
  alt_end.calculate();
  if (flight_present(false))
  {
    all_alt_start.add(alt_start.get_avg());
  }
  if (!baro_error.empty())
  {
    baro_error.calculate();
  }
}

void AircraftModel::set_wind_if_not_available(const SpeedVector &wind_avg)
{
  assert(replay);
  if (!Calculated().estimated_wind_available)
  {
    replay->SetCalculated().estimated_wind = wind_avg;
  }
}

TurnModeList AircraftModel::gen_turnmodelist(const EncounterMapStore::EncounterInfo &info) const
{
  const TimeStamp t0 = info.time_start - FloatDuration{EncounterMapStore::TYP_TRAIL};
  const TimeStamp t1 = info.time_start + FloatDuration{1};
  return trail.gen_turnmodelist(t0, t1);
}

//////////////////////////////////////////////////////////////////////////////////////////////////

bool AircraftModel::other_visible(const EncounterMapStore::EncounterInfo &info,
                                  const unsigned id_target) const
{
  for (auto &&p : trail)
  {

    if (!p.within_time(info.time_start - FloatDuration{EncounterMapStore::TYP_TRAIL},
                       info.time_end + FloatDuration{EncounterMapStore::HYS_TRAIL}))
    {
      continue;
    }
    if (p.present(id_target))
      return true;
  }
  return false;
}

boost::json::object AircraftModel::write_encounter(const EncounterMapStore::EncounterInfo &info,
                                                   const unsigned id_target,
                                                   const bool detailed) const
{
  return TraceWriter::write_encounter(*this, info, id_target, detailed);
}

boost::json::object AircraftModel::write_vignette(const Vignette &info) const
{
  return TraceWriter::write_vignette(*this, info);
}

boost::json::object AircraftModel::write_incursion(
    const Vignette &info,
    const std::vector<std::pair<TimeStamp, double>> &depth_samples,
    const std::vector<std::tuple<TimeStamp, GeoPoint, double>> &boundary_samples,
    const std::vector<EventTrailSample> &trail_samples) const
{
  return TraceWriter::write_incursion(*this, info, depth_samples, boundary_samples, trail_samples);
}

boost::json::object AircraftModel::write_terrain(
    const Vignette &info,
    const std::vector<TerrainDistanceSample> &distance_samples,
    const std::vector<EventTrailSample> &trail_samples) const
{
  return TraceWriter::write_terrain(*this, info, distance_samples, trail_samples);
}

bool AircraftModel::within_horizontal_distance(const AircraftModel &other,
                                               const TimeStamp t_min,
                                               const TimeStamp t_max,
                                               const double distance_m) const
{
  auto i = trail.begin();
  auto j = other.trail.begin();

  while (i != trail.end() && i->pos.time < t_min)
    ++i;
  while (j != other.trail.end() && j->pos.time < t_min)
    ++j;

  while (i != trail.end() && j != other.trail.end())
  {
    if (i->pos.time > t_max || j->pos.time > t_max)
      break;

    if (i->pos.time == j->pos.time)
    {
      if (i->pos.location.Distance(j->pos.location) <= distance_m)
      {
        return true;
      }
      ++i;
      ++j;
      continue;
    }

    if (i->pos.time < j->pos.time)
      ++i;
    else
      ++j;
  }

  return false;
}

bool AircraftModel::get_average_wind(const TimeStamp t_min,
                                     const TimeStamp t_max,
                                     SpeedVector &wind) const
{
  Vector wind_acc(0, 0);
  unsigned num_samples = 0;

  for (auto &&p : trail)
  {
    if (!p.within_time(t_min, t_max) || !p.v_wind.norm)
    {
      continue;
    }

    const Vector airspeed = Vector(p.v_wind);
    const Vector groundspeed = Vector(SpeedVector(p.trk.bearing, p.trk.distance));
    wind_acc += airspeed - groundspeed;
    ++num_samples;
  }

  if (num_samples == 0)
  {
    return false;
  }

  wind = SpeedVector(wind_acc.y / num_samples, wind_acc.x / num_samples);
  return true;
}

bool AircraftModel::get_first_location(const TimeStamp t_min,
                                       const TimeStamp t_max,
                                       GeoPoint &location,
                                       TimeStamp &time) const
{
  for (auto &&p : trail)
  {
    if (!p.within_time(t_min, t_max))
      continue;

    location = p.pos.location;
    time = p.pos.time;
    return true;
  }

  return false;
}

double AircraftModel::update_baro_altitude(double &mix)
{
  const MoreData &basic = replay->Basic();
  const double baro_altitude = basic.baro_altitude + baro_offset;
  const double err = basic.gps_altitude - baro_altitude;
  mix = (1 - MIX_BARO) * basic.gps_altitude + MIX_BARO * baro_altitude;
  baro_error.add((basic.gps_altitude - mix) * (basic.gps_altitude - mix) + (baro_altitude - mix) * (baro_altitude - mix));
  baro_offset += ALPHA_BARO * err;
  return baro_altitude;
}

void AircraftModel::calc_auxiliary(const AircraftModel &target)
{
  // TODO: miss distance and LOS rate (bearing rate)
  const TrailPoint &p0 = trail.back();
  const TrailPoint &p1 = target.trail.back();
  const DetectMiss miss(p0, p1);
  const AuxiliaryPair auxiliary(euler.get_aspect(miss.xrel), miss);
  trail.back().set_auxiliary(target.idi, auxiliary);
}

void AircraftModel::reset()
{
  replay->Rewind();

  replay_ok = true;
  circling_computer.Reset();
  wind_computer.Reset();

  flight_time_start = TimeStamp::Undefined();
}

const GeoPoint AircraftModel::get_location() const
{
  return replay->Basic().location;
}

std::string AircraftModel::get_trace_filename() const
{
  std::ostringstream oss;
  oss << "trace_" << id
      << ".json";
  return oss.str();
}

const AuxiliaryPair &AircraftModel::get_latest_auxiliary(const AircraftModel &target) const
{
  return trail.back().get_auxiliary(target.idi);
}
