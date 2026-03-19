// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "AircraftModel.hpp"
#include "DetectMiss.hpp"
#include "Formatter/TimeFormatter.hpp"
#include <fstream>
#include <iomanip> // std::setprecision
#include <iostream>
#include "FlightReconstruction.hpp"
#include "ReconstructionUtility.hpp"

using namespace MultiAircraft;

int AircraftModel::num_aircraft = 0;
TimeStamp AircraftModel::first_launch = TimeStamp::Undefined();
double AircraftModel::MIX_BARO = 0.5;
int AircraftModel::filter_type = 1;
GlidePolar AircraftModel::glide_polar(0);
WindSettings AircraftModel::wind_settings;
CirclingSettings AircraftModel::circling_settings;

bool AircraftModel::init(Args &args)
{
  if (num_aircraft == 0)
  {
    wind_settings.SetDefaults();
    wind_settings.zig_zag_wind = false;
  }

  const char *ptr = args.PeekNext();
  const char *ptr_end = ptr + strlen(ptr) - 1;
  char buffer[80];

  while ((*ptr_end != '/') && (ptr_end > ptr))
  {
    ptr_end--;
  }
  ptr = ptr_end;

  while (*ptr != '_')
  {
    ptr++;
  }
  ptr++;
  sscanf(ptr, "%[^.]", buffer);
  id = std::string(buffer);

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

  while (trail.size() > EncounterMapStore::MAX_TRAIL_FACTOR * EncounterMapStore::TYP_TRAIL + 2)
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
        {"avg_timestep", avg_timestep},
    };
  }
  return {};
}

void AircraftModel::finalise(Averager &all_alt_start)
{
  if (!live)
    return;
  std::ofstream json_file(get_trace_filename());

  char date_buffer[32];
  FormatISO8601(date_buffer, flight_date_utc_start);
  json_trace.emplace("date", date_buffer);

  json_file << boost::json::serialize(json_trace);

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

static void update_visibility_avg(Averager &visibility_avg, const Visibility &visibility)
{
  visibility_avg.add(visibility.focus_factor * (1 - visibility.occlusion));
}

boost::json::object AircraftModel::write_encounter(const EncounterMapStore::EncounterInfo &info,
                                                   const unsigned id_target,
                                                   const bool detailed) const
{
  boost::json::array trace;

  bool plausible = true;
  Averager visibility_avg;
  FlightReconstruction::Filter filter;
  std::vector<DetectMiss> misses;
  bool kf_valid = filter_type > 0;

  for (auto &&p : trail)
  {

    if (!p.within_time(info.time_start - FloatDuration{EncounterMapStore::TYP_TRAIL},
                       info.time_end + FloatDuration{EncounterMapStore::HYS_TRAIL}))
    {
      continue;
    }

    const FlatPoint fp = info.project_loc_wind(p);
    const AuxiliaryPair &auxiliary = p.get_auxiliary(id_target);
    const DetectMiss &miss = auxiliary.second;

    // TODO: update aspect, visibility if using smoothing filter

    if (detailed)
    {
      plausible &= p.plausible;
      misses.push_back(miss);
    }
    bool proc_visible = (p.pos.time <= info.time_start);

    boost::json::object step = {
        {"t", (p.pos.time - info.time_start).count()},
        {"alt_baro", p.pos.baro_altitude},
        {"v", p.v_wind.norm},
        {"hdg", p.v_wind.bearing.Degrees()},
    };

    if (filter_type > 0)
    {
      if (trace.empty())
      {
        auto state = FlightReconstruction::get_initial_state_estimate(fp.y, fp.x,
                                                                      -p.pos.gps_altitude,
                                                                      p.v_tas,
                                                                      p.bank_angle.Radians(),
                                                                      p.pitch_angle.Radians(),
                                                                      p.yaw_angle.AsBearing().Radians());
        filter.initialise(state, 1.0);
        // FlightReconstruction::write(filter.get_state());
      }
      {
        FlightReconstruction::Measurement measurement;
        auto &[y, x, z, U] = measurement.data;
        x.value = fp.x;
        y.value = fp.y;
        z.value = -p.pos.gps_altitude;
        U.value = p.v_tas;
        try
        {
          filter.update(measurement, 1.0);
        }
        catch (const std::exception &e)
        {
          std::cerr << "Filter update failed: " << e.what() << "\n";
          kf_valid = false;
        }
      }
    }

    if (!kf_valid)
    {
      step.emplace("v_tas", p.v_tas);
      step.emplace("v_ias", p.v_ias);
      step.emplace("bank", p.bank_angle.Degrees());
      step.emplace("pitch", p.pitch_angle.Degrees());
      step.emplace("yaw", p.yaw_angle.AsBearing().Degrees());
      step.emplace("load_factor", p.load_factor);
    }
    if ((!kf_valid) || (filter_type < 2))
    {
      step.emplace("x", fp.x);
      step.emplace("y", fp.y);
      step.emplace("alt_gps", p.pos.gps_altitude);
    }

    if (detailed)
    {
      const Aspect &aspect = auxiliary.first;
      const Visibility visibility(aspect);
      step.emplace("turnrate", p.turn_rate_wind.Degrees());
      step.emplace("turn_mode", TurnModeList::to_string(p.turn_mode));
      step.emplace("actual", p.actual);
      step.emplace("plausible", p.plausible);
      step.emplace("fix_acc", p.fix_acc);

      if (!kf_valid)
      {
        step.emplace("range", aspect.range);
        step.emplace("elevation_angle", aspect.elevation_angle.Degrees());
        step.emplace("azimuth_angle", aspect.azimuth_angle.Degrees());
        step.emplace("inclination_angle", aspect.inclination_angle.Degrees());
        step.emplace("ang_size", visibility.angular_size.Degrees());
        step.emplace("occlusion", visibility.occlusion);
        step.emplace("focus_factor", visibility.focus_factor);
        if (proc_visible)
        {
          update_visibility_avg(visibility_avg, visibility);
        }
      }

      step.emplace("roc", p.roc);
      step.emplace("miss_TCA", miss.TCA);
      step.emplace("miss_d", miss.miss_d_mag);
      step.emplace("miss_vrel", miss.vrel_mag);
      step.emplace("distance_scale", miss.distance_scale);
    }

    trace.emplace_back(step);
  }

  ///////////////////////////////

  if (kf_valid)
  {
    int count = 0;
    const auto &smoothed_states = filter.get_smoothed_states();
    for (auto &step : trace)
    {
      auto &_step = step.as_object();
      auto &state = smoothed_states[count];
      auto &seuler = FlightReconstruction::get_euler(state);
      auto &aero = filter.get_aero(state);
      auto &dstate = FlightReconstruction::convert_state(state);
      _step.emplace("bank", seuler[0]);
      _step.emplace("pitch", seuler[1]);
      _step.emplace("yaw", seuler[2]);
      _step.emplace("load_factor", aero.load_factor);
      _step.emplace("v_ias", aero.V_ias);
      _step.emplace("v_tas", aero.V_tas);
      if (filter_type == 2)
      {
        _step.emplace("y", dstate[FlightReconstruction::POS_X]);
        _step.emplace("x", dstate[FlightReconstruction::POS_Y]);
        _step.emplace("alt_gps", -dstate[FlightReconstruction::POS_Z]);
      }
      if (detailed)
      {
        const EulerAngles _euler(Angle::Degrees(seuler[0]),
                                 Angle::Degrees(seuler[1]),
                                 Angle::Degrees(seuler[2]));
        const auto &miss = misses[count];
        const Aspect aspect = _euler.get_aspect(miss.xrel);
        const Visibility visibility(aspect);

        _step.emplace("range", aspect.range);
        _step.emplace("elevation_angle", aspect.elevation_angle.Degrees());
        _step.emplace("azimuth_angle", aspect.azimuth_angle.Degrees());
        _step.emplace("inclination_angle", aspect.inclination_angle.Degrees());
        _step.emplace("ang_size", visibility.angular_size.Degrees());
        _step.emplace("occlusion", visibility.occlusion);
        _step.emplace("focus_factor", visibility.focus_factor);
        if (_step.at("t").to_number<double>() <= 0)
        {
          update_visibility_avg(visibility_avg, visibility);
        }
      }
      count++;
    }
  }
  // TODO: update other outputs

  visibility_avg.calculate();

  char date_buffer[32];
  FormatISO8601(date_buffer, flight_date_utc_start);

  boost::json::object data = {
      {"id", id},
      {"fr_info", fr_info},
      {"fr_id", fr_id},
      {"turn_mode_list", gen_turnmodelist(info).string()},
      {"in_flock", in_flock},
      {"plausible", plausible},
      {"date_start", date_buffer},
      {"visibility_avg", visibility_avg.get_avg()},
      {"trace", trace}};

  return data;
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
