// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "AircraftModel.hpp"
#include "DetectMiss.hpp"
#include "Formatter/TimeFormatter.hpp"
#include "Math/Vector.hpp"
#include <fstream>
#include <iomanip> // std::setprecision
#include <iostream>
#include <string_view>
#include "FlightReconstruction.hpp"
#include "ReconstructionUtility.hpp"

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

bool AircraftModel::init(Args &args)
{
  if (num_aircraft == 0)
  {
    wind_settings.SetDefaults();
    wind_settings.zig_zag_wind = false;
  }

  std::string_view path = args.PeekNext();
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

namespace
{
  constexpr bool uses_updraft_gust_filter(const int filter_type)
  {
    return filter_type == 3 || filter_type == 4;
  }

  constexpr bool uses_smoothed_position(const int filter_type)
  {
    return filter_type == 2 || filter_type == 4;
  }

  void update_visibility_avg(Averager &visibility_avg, const Visibility &visibility)
  {
    visibility_avg.add(visibility.focus_factor * (1 - visibility.occlusion));
  }

  void append_visibility_fields(boost::json::object &step,
                                const Aspect &aspect,
                                const Visibility &visibility)
  {
    step.emplace("range", aspect.range);
    step.emplace("elevation_angle", aspect.elevation_angle.Degrees());
    step.emplace("azimuth_angle", aspect.azimuth_angle.Degrees());
    step.emplace("inclination_angle", aspect.inclination_angle.Degrees());
    step.emplace("ang_size", visibility.angular_size.Degrees());
    step.emplace("occlusion", visibility.occlusion);
    step.emplace("focus_factor", visibility.focus_factor);
  }

  void append_raw_attitude_fields(boost::json::object &step, const TrailPoint &p)
  {
    step.emplace("bank", p.bank_angle.Degrees());
    step.emplace("pitch", p.pitch_angle.Degrees());
    step.emplace("yaw", p.yaw_angle.AsBearing().Degrees());
  }

  void append_raw_attitude_fields(boost::json::object &step, const EventTrailSample &p)
  {
    step.emplace("bank", p.bank_angle.Degrees());
    step.emplace("pitch", p.pitch_angle.Degrees());
    step.emplace("yaw", p.yaw_angle.AsBearing().Degrees());
  }

  void append_raw_flight_fields(boost::json::object &step, const TrailPoint &p)
  {
    step.emplace("v_tas", p.v_tas);
    step.emplace("v_ias", p.v_ias);
    append_raw_attitude_fields(step, p);
    step.emplace("load_factor", p.load_factor);
  }

  void append_raw_flight_fields(boost::json::object &step, const EventTrailSample &p)
  {
    step.emplace("v_tas", p.v_tas);
    step.emplace("v_ias", p.v_ias);
    append_raw_attitude_fields(step, p);
    step.emplace("load_factor", p.load_factor);
  }

  void append_position_fields(boost::json::object &step, const FlatPoint &fp,
                              const TrailPoint &p)
  {
    step.emplace("x", fp.x);
    step.emplace("y", fp.y);
    step.emplace("alt_gps", p.pos.gps_altitude);
  }

  void append_position_fields(boost::json::object &step, const FlatPoint &fp,
                              const EventTrailSample &p)
  {
    step.emplace("x", fp.x);
    step.emplace("y", fp.y);
    step.emplace("alt_gps", p.gps_altitude);
  }

  void append_common_detailed_fields(boost::json::object &step,
                                     const TrailPoint &p,
                                     const DetectMiss &miss)
  {
    step.emplace("turnrate", p.turn_rate_wind.Degrees());
    step.emplace("turn_mode", TurnModeList::to_string(p.turn_mode));
    step.emplace("actual", p.actual);
    step.emplace("plausible", p.plausible);
    step.emplace("fix_acc", p.fix_acc);
    step.emplace("roc", p.roc);
    step.emplace("miss_TCA", miss.TCA);
    step.emplace("d_mag", miss.d_mag);
    step.emplace("miss_d", miss.miss_d_mag);
    step.emplace("miss_vrel", miss.vrel_mag);
    step.emplace("distance_scale", miss.distance_scale);
  }

  void append_common_zem_fields(boost::json::object &step, const DetectMiss &miss,
                                const FlatPoint &fp, const TrailPoint &p)
  {
    step.emplace("miss_x", fp.x + p.vel[0] * miss.TCA);
    step.emplace("miss_y", fp.y + p.vel[1] * miss.TCA);
    step.emplace("miss_z", p.pos.gps_altitude + p.vel[2] * miss.TCA);
  }

  inline void initialise_filter_state(FlightReconstruction::Filter &filter,
                                      const FlatPoint &fp,
                                      const TrailPoint &p)
  {
    auto state = FlightReconstruction::get_initial_state_estimate(fp.y, fp.x,
                                                                  -p.pos.gps_altitude,
                                                                  p.v_tas,
                                                                  p.bank_angle.Radians(),
                                                                  p.pitch_angle.Radians(),
                                                                  p.yaw_angle.AsBearing().Radians());
    filter.initialise(state, 1.0);
  }

  inline void initialise_filter_state(FlightReconstruction::FilterWithUpdraftGust &filter,
                                      const FlatPoint &fp,
                                      const TrailPoint &p)
  {
    auto state = FlightReconstruction::get_initial_state_estimate_with_updraft_gust(
        fp.y, fp.x,
        -p.pos.gps_altitude,
        p.v_tas,
        p.bank_angle.Radians(),
        p.pitch_angle.Radians(),
        p.yaw_angle.AsBearing().Radians());
    filter.initialise(state, 1.0);
  }

  template <typename FilterType>
  bool update_encounter_filter(FilterType &filter,
                               const bool initialise,
                               const FlatPoint &fp,
                               const TrailPoint &p)
  {
    if (initialise)
    {
      initialise_filter_state(filter, fp, p);
    }

    FlightReconstruction::Measurement measurement;
    auto &[y, x, z, U] = measurement.data;
    x.value = fp.x;
    y.value = fp.y;
    z.value = -p.pos.gps_altitude;
    U.value = p.v_tas;

    try
    {
      filter.update(measurement, 1.0);
      return true;
    }
    catch (const std::exception &e)
    {
      // std::cerr << "Filter update failed: " << e.what() << "\n";
      return false;
    }
  }

  inline void initialise_filter_state(FlightReconstruction::Filter &filter,
                                      const FlatPoint &fp,
                                      const EventTrailSample &p)
  {
    auto state = FlightReconstruction::get_initial_state_estimate(fp.y, fp.x,
                                                                  -p.gps_altitude,
                                                                  p.v_tas,
                                                                  p.bank_angle.Radians(),
                                                                  p.pitch_angle.Radians(),
                                                                  p.yaw_angle.AsBearing().Radians());
    filter.initialise(state, 1.0);
  }

  inline void initialise_filter_state(FlightReconstruction::FilterWithUpdraftGust &filter,
                                      const FlatPoint &fp,
                                      const EventTrailSample &p)
  {
    auto state = FlightReconstruction::get_initial_state_estimate_with_updraft_gust(
        fp.y, fp.x,
        -p.gps_altitude,
        p.v_tas,
        p.bank_angle.Radians(),
        p.pitch_angle.Radians(),
        p.yaw_angle.AsBearing().Radians());
    filter.initialise(state, 1.0);
  }

  template <typename FilterType>
  bool update_encounter_filter(FilterType &filter,
                               const bool initialise,
                               const FlatPoint &fp,
                               const EventTrailSample &p)
  {
    if (initialise)
    {
      initialise_filter_state(filter, fp, p);
    }

    FlightReconstruction::Measurement measurement;
    auto &[y, x, z, U] = measurement.data;
    x.value = fp.x;
    y.value = fp.y;
    z.value = -p.gps_altitude;
    U.value = p.v_tas;

    try
    {
      filter.update(measurement, 1.0);
      return true;
    }
    catch (const std::exception &e)
    {
      // std::cerr << "Filter update failed: " << e.what() << "\n";
      return false;
    }
  }

  template <typename StateType>
  FlightReconstruction::Euler get_state_euler(const StateType &state)
  {
    auto &[states, quaternion] = state.data;
    (void)states;
    const Eigen::Matrix3d R = quaternion.get_q().toRotationMatrix();
    double theta = asin(-R(2, 0));
    double psi = acos(R(0, 0) / cos(theta)) * FlightReconstruction::sign(R(1, 0));
    double phi = acos(R(2, 2) / cos(theta)) * FlightReconstruction::sign(R(2, 1));
    if (psi < 0)
      psi += 2 * M_PI;

    return FlightReconstruction::Euler(phi, theta, psi) / FlightReconstruction::DEGTORAD;
  }

  template <typename StateType>
  FlightReconstruction::DerivState get_state_vector(const StateType &state)
  {
    auto &[states, quaternion] = state.data;
    FlightReconstruction::DerivState dstate;
    dstate.reserve(states.size() + 4);
    for (Eigen::Index i = 0; i < states.size(); ++i)
      dstate.push_back(states[i]);

    const auto &q = quaternion.get_q();
    dstate.push_back(q.w());
    dstate.push_back(q.x());
    dstate.push_back(q.y());
    dstate.push_back(q.z());
    return dstate;
  }

  template <typename FilterType>
  void append_smoothed_fields(boost::json::array &trace,
                              const FilterType &filter,
                              const size_t trace_offset,
                              const int filter_type,
                              const bool detailed,
                              const std::vector<DetectMiss> &misses,
                              Averager &visibility_avg)
  {
    const auto &smoothed_states = filter.get_smoothed_states();
    if (trace_offset >= smoothed_states.size())
      return;

    const size_t n = std::min(trace.size(), smoothed_states.size() - trace_offset);

    for (size_t i = 0; i < n; ++i)
    {
      auto &_step = trace[i].as_object();
      const auto &state = smoothed_states[i + trace_offset];
      const auto seuler = get_state_euler(state);
      const auto &aero = filter.get_aero(state);
      const auto dstate = get_state_vector(state);
      _step["bank"] = seuler[0];
      _step["pitch"] = seuler[1];
      _step["yaw"] = seuler[2];
      _step["load_factor"] = aero.load_factor;
      _step["alpha"] = aero.alpha / FlightReconstruction::DEGTORAD;
      _step["v_ias"] = aero.V_ias;
      _step["v_tas"] = aero.V_tas;

      if (uses_updraft_gust_filter(filter_type) &&
          dstate.size() > FlightReconstruction::QUATERNION + 4)
      {
        _step["w_g"] = dstate[FlightReconstruction::QUATERNION];
      }

      if (uses_smoothed_position(filter_type))
      {
        _step["y"] = dstate[FlightReconstruction::POS_X];
        _step["x"] = dstate[FlightReconstruction::POS_Y];
        _step["alt_gps"] = -dstate[FlightReconstruction::POS_Z];
      }

      if (!detailed)
        continue;

      const EulerAngles euler(Angle::Degrees(seuler[0]),
                              Angle::Degrees(seuler[1]),
                              Angle::Degrees(seuler[2]));
      if (i >= misses.size())
        continue;

      const auto &miss = misses[i];
      const Aspect aspect = euler.get_aspect(miss.xrel);
      const Visibility visibility(aspect);

      append_visibility_fields(_step, aspect, visibility);
      if (_step.find("t") != _step.end() && _step.at("t").to_number<double>() <= 0)
        update_visibility_avg(visibility_avg, visibility);
    }
  }

  template <typename FilterType>
  void populate_encounter_trace(const AircraftModel &aircraft,
                                const EncounterMapStore::EncounterInfo &info,
                                const unsigned id_target,
                                const bool detailed,
                                FilterType &filter,
                                boost::json::array &trace,
                                bool &plausible,
                                Averager &visibility_avg,
                                std::vector<DetectMiss> &misses,
                                bool &kf_valid,
                                size_t &reconstruction_warmup_samples)
  {
    const TimeStamp t_min = info.time_start - FloatDuration{EncounterMapStore::TYP_TRAIL};
    const TimeStamp t_max = info.time_end + FloatDuration{EncounterMapStore::HYS_TRAIL};
    const TimeStamp t_reconstruction_min = t_min - AircraftModel::reconstruction_pre_buffer;
    const bool filter_enabled = AircraftModel::filter_type > 0;

    for (auto &&p : aircraft.GetTrail())
    {
      if (!p.within_time(t_reconstruction_min, t_max))
        continue;

      const FlatPoint fp = info.project_loc_wind(p);

      if (filter_enabled)
        kf_valid &= update_encounter_filter(filter, trace.empty(), fp, p);

      if (p.pos.time < t_min)
      {
        if (filter_enabled)
          ++reconstruction_warmup_samples;
        continue;
      }

      const AuxiliaryPair &auxiliary = p.get_auxiliary(id_target);
      const DetectMiss &miss = auxiliary.second;

      if (detailed)
      {
        plausible &= p.plausible;
        misses.push_back(miss);
      }
      const bool proc_visible = p.pos.time <= info.time_start;

      boost::json::object step = {
          {"t", (p.pos.time - info.time_start).count()},
          {"alt_baro", p.pos.baro_altitude},
          {"v", p.v_wind.norm},
          {"hdg", p.v_wind.bearing.Degrees()},
      };
      append_raw_attitude_fields(step, p);

      const bool need_raw_flight = !kf_valid;
      const bool need_raw_position = need_raw_flight || !uses_smoothed_position(AircraftModel::filter_type);

      if (need_raw_flight)
        append_raw_flight_fields(step, p);
      if (need_raw_position)
        append_position_fields(step, fp, p);

      if (detailed)
      {
        const Aspect &aspect = auxiliary.first;
        const Visibility visibility(aspect);
        append_common_detailed_fields(step, p, miss);
        append_common_zem_fields(step, miss, fp, p);

        if (need_raw_flight)
        {
          append_visibility_fields(step, aspect, visibility);
          if (proc_visible)
            update_visibility_avg(visibility_avg, visibility);
        }
      }

      trace.emplace_back(step);
    }
  }

  template <typename FilterType>
  void populate_vignette_trace(const AircraftModel &aircraft,
                               const Vignette &info,
                               FilterType &filter,
                               boost::json::array &trace,
                               bool &plausible,
                               bool &kf_valid,
                               size_t &reconstruction_warmup_samples)
  {
    const TimeStamp t_reconstruction_min = info.time_start - AircraftModel::reconstruction_pre_buffer;
    const bool filter_enabled = AircraftModel::filter_type > 0;

    for (auto &&p : aircraft.GetTrail())
    {
      if (!p.within_time(t_reconstruction_min, info.time_end))
        continue;

      const FlatPoint fp = info.project_loc_wind(p);

      if (filter_enabled)
        kf_valid &= update_encounter_filter(filter, trace.empty(), fp, p);

      if (p.pos.time < info.time_start)
      {
        if (filter_enabled)
          ++reconstruction_warmup_samples;
        continue;
      }

      boost::json::object step = {
          {"t", (p.pos.time - info.time_start).count()},
          {"alt_baro", p.pos.baro_altitude},
          {"v", p.v_wind.norm},
          {"hdg", p.v_wind.bearing.Degrees()},
      };
      append_raw_attitude_fields(step, p);

      const bool need_raw_flight = !kf_valid;
      const bool need_raw_position = need_raw_flight || !uses_smoothed_position(AircraftModel::filter_type);

      if (need_raw_flight)
        append_raw_flight_fields(step, p);
      if (need_raw_position)
        append_position_fields(step, fp, p);

      plausible &= p.plausible;
      trace.emplace_back(step);
    }
  }

  template <typename FilterType>
  void populate_incursion_trace(const Vignette &info,
                                const std::vector<std::pair<TimeStamp, double>> &depth_samples,
                                const std::vector<std::tuple<TimeStamp, GeoPoint, double>> &boundary_samples,
                                const std::vector<EventTrailSample> &trail_samples,
                                FilterType &filter,
                                boost::json::array &trace,
                                bool &plausible,
                                bool &kf_valid,
                                size_t &reconstruction_warmup_samples)
  {
    const TimeStamp t_min = info.time_start - FloatDuration{EncounterMapStore::TYP_TRAIL};
    const TimeStamp t_max = info.time_end + FloatDuration{EncounterMapStore::HYS_TRAIL};
    const TimeStamp t_reconstruction_min = info.time_start - AircraftModel::reconstruction_pre_buffer;
    const bool filter_enabled = AircraftModel::filter_type > 0;
    auto depth_it = depth_samples.begin();
    auto boundary_it = boundary_samples.begin();
    double current_depth = 0;
    GeoPoint current_boundary_location = info.origin;
    double current_boundary_altitude = info.alt;

    for (const auto &p : trail_samples)
    {
      if (p.time < t_reconstruction_min || p.time > t_max)
        continue;

      const FlatPoint fp = info.project_loc_wind(p.location, p.time);

      if (filter_enabled)
        kf_valid &= update_encounter_filter(filter, trace.empty(), fp, p);

      if (p.time < t_min)
      {
        if (filter_enabled)
          ++reconstruction_warmup_samples;
        continue;
      }

      while (depth_it != depth_samples.end() && depth_it->first <= p.time)
      {
        current_depth = depth_it->second;
        ++depth_it;
      }

      while (boundary_it != boundary_samples.end() && std::get<0>(*boundary_it) <= p.time)
      {
        current_boundary_location = std::get<1>(*boundary_it);
        current_boundary_altitude = std::get<2>(*boundary_it);
        ++boundary_it;
      }

      const bool within_event = p.time >= info.time_start && p.time <= info.time_end;
      const bool has_positive_depth = within_event && current_depth > 0;

      boost::json::object incursion = {
          {"depth", within_event ? current_depth : 0.0},
      };
      if (has_positive_depth)
      {
        const FlatPoint boundary_fp = info.project_loc_wind(current_boundary_location, p.time);
        incursion.emplace("x", boundary_fp.x);
        incursion.emplace("y", boundary_fp.y);
        incursion.emplace("alt", current_boundary_altitude);
      }

      boost::json::object step = {
          {"t", (p.time - info.time_start).count()},
          {"alt_baro", p.baro_altitude},
          {"v", p.v_wind.norm},
          {"hdg", p.v_wind.bearing.Degrees()},
          {"incursion", std::move(incursion)},
      };
      append_raw_attitude_fields(step, p);

      const bool need_raw_flight = !kf_valid;
      const bool need_raw_position = need_raw_flight || !uses_smoothed_position(AircraftModel::filter_type);

      if (need_raw_flight)
        append_raw_flight_fields(step, p);
      if (need_raw_position)
        append_position_fields(step, fp, p);

      plausible &= p.plausible;
      trace.emplace_back(step);
    }
  }

  template <typename FilterType>
  void populate_terrain_trace(const Vignette &info,
                              const std::vector<std::pair<TimeStamp, double>> &distance_samples,
                              const std::vector<EventTrailSample> &trail_samples,
                              FilterType &filter,
                              boost::json::array &trace,
                              bool &plausible,
                              bool &kf_valid,
                              size_t &reconstruction_warmup_samples)
  {
    const TimeStamp t_min = info.time_start - FloatDuration{EncounterMapStore::TYP_TRAIL};
    const TimeStamp t_max = info.time_end + FloatDuration{EncounterMapStore::HYS_TRAIL};
    const TimeStamp t_reconstruction_min = info.time_start - AircraftModel::reconstruction_pre_buffer;
    const bool filter_enabled = AircraftModel::filter_type > 0;
    auto distance_it = distance_samples.begin();
    double current_distance = 0;

    for (const auto &p : trail_samples)
    {
      if (p.time < t_reconstruction_min || p.time > t_max)
        continue;

      const FlatPoint fp = info.project_loc_wind(p.location, p.time);

      if (filter_enabled)
        kf_valid &= update_encounter_filter(filter, trace.empty(), fp, p);

      if (p.time < t_min)
      {
        if (filter_enabled)
          ++reconstruction_warmup_samples;
        continue;
      }

      while (distance_it != distance_samples.end() && distance_it->first <= p.time)
      {
        current_distance = distance_it->second;
        ++distance_it;
      }

      boost::json::object terrain = {
          {"distance", current_distance},
      };

      boost::json::object step = {
          {"t", (p.time - info.time_start).count()},
          {"alt_baro", p.baro_altitude},
          {"v", p.v_wind.norm},
          {"hdg", p.v_wind.bearing.Degrees()},
          {"terrain", std::move(terrain)},
      };
      append_raw_attitude_fields(step, p);

      const bool need_raw_flight = !kf_valid;
      const bool need_raw_position = need_raw_flight || !uses_smoothed_position(AircraftModel::filter_type);

      if (need_raw_flight)
        append_raw_flight_fields(step, p);
      if (need_raw_position)
        append_position_fields(step, fp, p);

      plausible &= p.plausible;
      trace.emplace_back(step);
    }
  }

} // namespace

boost::json::object AircraftModel::write_encounter(const EncounterMapStore::EncounterInfo &info,
                                                   const unsigned id_target,
                                                   const bool detailed) const
{
  boost::json::array trace;

  bool plausible = true;
  Averager visibility_avg;
  std::vector<DetectMiss> misses;
  const bool filter_enabled = filter_type > 0;
  const bool use_updraft_filter = uses_updraft_gust_filter(filter_type);
  bool kf_valid = filter_enabled;
  size_t reconstruction_warmup_samples = 0;

  if (use_updraft_filter)
  {
    FlightReconstruction::FilterWithUpdraftGust filter;
    populate_encounter_trace(*this, info, id_target, detailed, filter, trace,
                             plausible, visibility_avg, misses,
                             kf_valid, reconstruction_warmup_samples);
    if (kf_valid)
      append_smoothed_fields(trace, filter, reconstruction_warmup_samples,
                             filter_type, detailed, misses, visibility_avg);
  }
  else
  {
    FlightReconstruction::Filter filter;
    populate_encounter_trace(*this, info, id_target, detailed, filter, trace,
                             plausible, visibility_avg, misses,
                             kf_valid, reconstruction_warmup_samples);
    if (kf_valid)
      append_smoothed_fields(trace, filter, reconstruction_warmup_samples,
                             filter_type, detailed, misses, visibility_avg);
  }

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

boost::json::object AircraftModel::write_vignette(const Vignette &info) const
{
  boost::json::array trace;

  bool plausible = true;
  const bool filter_enabled = filter_type > 0;
  const bool use_updraft_filter = uses_updraft_gust_filter(filter_type);
  bool kf_valid = filter_enabled;
  Averager visibility_avg;
  std::vector<DetectMiss> misses;
  size_t reconstruction_warmup_samples = 0;

  if (use_updraft_filter)
  {
    FlightReconstruction::FilterWithUpdraftGust filter;
    populate_vignette_trace(*this, info, filter, trace, plausible,
                            kf_valid, reconstruction_warmup_samples);

    if (kf_valid)
      append_smoothed_fields(trace, filter, reconstruction_warmup_samples,
                             filter_type, false, misses, visibility_avg);
  }
  else
  {
    FlightReconstruction::Filter filter;
    populate_vignette_trace(*this, info, filter, trace, plausible,
                            kf_valid, reconstruction_warmup_samples);

    if (kf_valid)
      append_smoothed_fields(trace, filter, reconstruction_warmup_samples,
                             filter_type, false, misses, visibility_avg);
  }

  char date_buffer[32];
  FormatISO8601(date_buffer, flight_date_utc_start);

  boost::json::object data = {
      {"id", id},
      {"fr_info", fr_info},
      {"fr_id", fr_id},
      {"in_flock", in_flock},
      {"plausible", plausible},
      {"date_start", date_buffer},
      {"trace", trace}};

  return data;
}

boost::json::object AircraftModel::write_incursion(
    const Vignette &info,
    const std::vector<std::pair<TimeStamp, double>> &depth_samples,
    const std::vector<std::tuple<TimeStamp, GeoPoint, double>> &boundary_samples,
    const std::vector<EventTrailSample> &trail_samples) const
{
  boost::json::array trace;

  bool plausible = true;
  const bool filter_enabled = filter_type > 0;
  const bool use_updraft_filter = uses_updraft_gust_filter(filter_type);
  bool kf_valid = filter_enabled;
  Averager visibility_avg;
  std::vector<DetectMiss> misses;
  size_t reconstruction_warmup_samples = 0;

  if (use_updraft_filter)
  {
    FlightReconstruction::FilterWithUpdraftGust filter;
    populate_incursion_trace(info, depth_samples, boundary_samples, trail_samples,
                             filter, trace, plausible,
                             kf_valid, reconstruction_warmup_samples);

    if (kf_valid)
      append_smoothed_fields(trace, filter, reconstruction_warmup_samples,
                             filter_type, false, misses, visibility_avg);
  }
  else
  {
    FlightReconstruction::Filter filter;
    populate_incursion_trace(info, depth_samples, boundary_samples, trail_samples,
                             filter, trace, plausible,
                             kf_valid, reconstruction_warmup_samples);

    if (kf_valid)
      append_smoothed_fields(trace, filter, reconstruction_warmup_samples,
                             filter_type, false, misses, visibility_avg);
  }

  char date_buffer[32];
  FormatISO8601(date_buffer, flight_date_utc_start);

  double max_depth = 0;
  for (const auto &[_, depth] : depth_samples)
    max_depth = std::max(max_depth, depth);

  boost::json::object data = {
      {"id", id},
      {"fr_info", fr_info},
      {"fr_id", fr_id},
      {"in_flock", in_flock},
      {"plausible", plausible},
      {"date_start", date_buffer},
      {"max_depth", max_depth},
      {"trace", trace}};

  return data;
}

boost::json::object AircraftModel::write_terrain(
    const Vignette &info,
    const std::vector<std::pair<TimeStamp, double>> &distance_samples,
    const std::vector<EventTrailSample> &trail_samples) const
{
  boost::json::array trace;

  bool plausible = true;
  const bool filter_enabled = filter_type > 0;
  const bool use_updraft_filter = uses_updraft_gust_filter(filter_type);
  bool kf_valid = filter_enabled;
  Averager visibility_avg;
  std::vector<DetectMiss> misses;
  size_t reconstruction_warmup_samples = 0;

  if (use_updraft_filter)
  {
    FlightReconstruction::FilterWithUpdraftGust filter;
    populate_terrain_trace(info, distance_samples, trail_samples,
                           filter, trace, plausible,
                           kf_valid, reconstruction_warmup_samples);

    if (kf_valid)
      append_smoothed_fields(trace, filter, reconstruction_warmup_samples,
                             filter_type, false, misses, visibility_avg);
  }
  else
  {
    FlightReconstruction::Filter filter;
    populate_terrain_trace(info, distance_samples, trail_samples,
                           filter, trace, plausible,
                           kf_valid, reconstruction_warmup_samples);

    if (kf_valid)
      append_smoothed_fields(trace, filter, reconstruction_warmup_samples,
                             filter_type, false, misses, visibility_avg);
  }

  char date_buffer[32];
  FormatISO8601(date_buffer, flight_date_utc_start);

  double min_distance = 0;
  bool has_distance = false;
  for (const auto &[_, distance] : distance_samples)
  {
    if (!has_distance)
    {
      min_distance = distance;
      has_distance = true;
    }
    else
      min_distance = std::min(min_distance, distance);
  }

  boost::json::object data = {
      {"id", id},
      {"fr_info", fr_info},
      {"fr_id", fr_id},
      {"in_flock", in_flock},
      {"plausible", plausible},
      {"date_start", date_buffer},
      {"distance_min", has_distance ? boost::json::value(min_distance) : boost::json::value(nullptr)},
      {"trace", trace}};

  return data;
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
