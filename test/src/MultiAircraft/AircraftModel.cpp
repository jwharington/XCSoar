#include "AircraftModel.hpp"
#include "Formatter/TimeFormatter.hpp"
#include <fstream>
#include <iomanip>      // std::setprecision

using namespace MultiAircraft;

extern bool debug;

int AircraftModel::num_aircraft = 0;
double AircraftModel::first_launch = 0;

GlidePolar AircraftModel::glide_polar(0);
WindSettings AircraftModel::wind_settings;
CirclingSettings AircraftModel::circling_settings;

static const char* truefalse(const bool x) {
  if (x)
    return "true";
  else
    return "false";
}

bool AircraftModel::init(Args& args)
{
  if (num_aircraft == 0) {
    wind_settings.SetDefaults();
    wind_settings.zig_zag_wind = false;
  }

  const char* ptr = args.PeekNext();
  const char* ptr_end = ptr+strlen(ptr)-1;
  char buffer[80];

  while ((*ptr_end != '/') && (ptr_end > ptr)) {
    ptr_end--;
  }
  ptr = ptr_end;

  while (*ptr != '_') {
    ptr++;
  }
  ptr++;
  sscanf(ptr,"%[^.]", buffer);
  id = std::string(buffer);

  idi = num_aircraft++;
  fr_info.clear();
  fr_id.clear();

  replay = CreateDebugReplay(args);
  if (replay == NULL) {
    replay_ok = false;
    return false;
  }
  json_trace_file.open(get_trace_filename());
  json_trace_file << "{\n";

  reset();
  return true;
}

std::string AircraftModel::get_symbol() const
{
  if (!replay->Calculated().flight.flying) {
    return std::string(" ");
  } else if (!valid) {
    return std::string("~");
  } else if (!live) {
    return std::string("?");
  } else {
    return std::string(".");
  }
}

void AircraftModel::advance()
{
  if (replay->Next()) {
    const MoreData &basic = replay->Basic();
    const DerivedInfo &calculated = replay->Calculated();
    if (!basic.location_available || !basic.gps_altitude_available) {
      return;
    }
    h_acc = replay->GetHAccuracy();
    if (calculated.flight.flying) {
      if (first_launch == 0) first_launch = basic.time;

      if (flight_time_start==0) {
        fr_info = replay->GetTypeInfo();
        fr_id = replay->GetIdentifier();

        flight_loc_start = basic.location;
        flight_time_start = basic.time;

        json_trace_file
            << "\"id\": \"" << id << "\",\n"
            << "\"frinfo\": \"" << fr_info << "\",\n"
            << "\"frid\": \"" << fr_id << "\",\n"
            << "\"trace\": [\n";
        json_trace_first = true;

        if (alt_start.empty()) {
          alt_start.add(basic.gps_altitude);
        }
      } else if ((basic.ground_speed < 20) && (basic.location.Distance(flight_loc_start) < 2500.0)) {
          alt_end.add(basic.gps_altitude);
          flight_time_end = basic.time;
      }

      flight_num_records++;
    } else if (flight_time_start > 0) {
      alt_end.add(basic.gps_altitude);
    } else {
      baro_offset = basic.gps_altitude-basic.baro_altitude;
      alt_start.add(basic.gps_altitude);
    }

    double mix;
    const double baro_altitude = update_baro_altitude(mix);

    interpolator.Update(basic.time, basic.location, basic.gps_altitude, baro_altitude);

    if (debug) {
      fprintf(ftrace,"%d %.8g %.8g %g %g %g\n", (int)basic.time,
              basic.location.longitude.Degrees(),
              basic.location.latitude.Degrees(),
              baro_altitude,
              basic.gps_altitude,
              mix);
    }

    circling_computer.TurnRate(replay->SetCalculated(),
                               basic, calculated.flight);
    circling_computer.Turning(replay->SetCalculated(),
                              basic,
                              calculated.flight,
                              circling_settings);

    wind_computer.Compute(wind_settings, glide_polar, basic,
                          replay->SetCalculated());
  } else {
    if (replay_ok && debug) {
      fclose(ftrace);
    }
    replay_ok = false;
  }
}


void AircraftModel::Interpolate(const int t, const SpeedVector& wind)
{
  interp_loc_last = interp_loc;
  interp_loc = interpolator.Interpolate(t, wind);

  if (trail.empty()) {
    trail.emplace_back(interp_loc, interpolator.GetVector(t, wind), replay->Calculated().turn_mode, replay->GetHAccuracy(), interpolator.IsActual(t));
    return;
  }
  const TrailPoint& prev = trail.back();
  trail.emplace_back(interp_loc, interpolator.GetVector(t, wind), replay->Calculated().turn_mode, replay->GetHAccuracy(), interpolator.IsActual(t));
  TrailPoint& now = trail.back();
  now.update_reconstruction(prev, wind);
  euler = EulerAngles(now.bank_angle, now.pitch_angle, now.yaw_angle);

  while (trail.size() > EncounterMapStore::MAX_TRAIL+2) {
    trail.pop_front();
  }

  valid = true;

  if (debug) {
    fprintf(fitrace,"%d %.8g %.8g %g\n", (int)t,
            interp_loc.location.longitude.Degrees(),
            interp_loc.location.latitude.Degrees(),
            interp_loc.gps_altitude);
  }
  if (!json_trace_first) {
    json_trace_file
        << ",\n";
  }
  json_trace_file
      << std::setprecision(5) << std::fixed
      << "{"
      << "\"t\": " << t << ", "
      << "\"longitude\": " << interp_loc.location.longitude.Degrees() << ", "
      << "\"latitude\": " << interp_loc.location.latitude.Degrees() << ", "
      << std::setprecision(1) << std::fixed
      << "\"gps_altitude\": " << interp_loc.gps_altitude
      << "}";
  json_trace_first = false;
}


void AircraftModel::advance_to_start(int &t_start, int &t_end)
{
  replay->SetCalculated().estimated_wind = SpeedVector();

  while (replay_ok && (!interpolator.Ready() || !replay->Calculated().flight.flying || (flight_time_start==0)))
    advance();

  if (!replay_ok)
    return;

  const int t_this = replay->Basic().time;

  if ((t_start==0) || (t_this < t_start)) {
    t_start = t_this;
  }
  if (!interpolator.Ready()) {
    valid = false;
  } else {
    t_end = std::max(t_end, (int)interpolator.GetMaxTime());
  }
}


bool AircraftModel::advance_to_time(const int t, int &t_end)
{
  valid = false;
  if (!replay_ok || !interpolator.Ready())
    return false;
  if (t < interpolator.GetMinTime())
    return false;
  while (interpolator.NeedData(t) && replay_ok) {
    advance();
    t_end = std::max(t_end, (int)interpolator.GetMaxTime());
  }
  if (!replay->Calculated().flight.flying)
    return false;

  Interpolate(t, replay->Calculated().estimated_wind);

  if (interp_loc.baro_altitude < 400) { // TODO!
    valid = false;
  } else {
    live = true;
  }

  return valid && live;
}


void AircraftModel::write_diagnostics(FILE* fout, const double alt_start_av, const double geoid_sep,
                                      Averager& all_baro_error) const
{
  if (live) {
    const double alt_anomaly = geoid_sep!= 0? (alt_start.get_avg()-alt_start_av)/geoid_sep : 0;
    const double avg_timestep = flight_num_records>1? std::max(0.0,(flight_time_end-flight_time_start-1)*1.0/(flight_num_records-1)): 0;
    all_baro_error.add(baro_error);
    fprintf(fout, "\"%s\" %d %g %g %.1f %.1f %.1f \"%s\" \"%s\" %.1f %.1f\n",
            id.c_str(),
            idi, flight_time_start, flight_time_end,
            alt_start.get_avg(),
            alt_end.get_avg(),
            sqrt(baro_error.get_avg()),
            fr_info.c_str(),
            fr_id.c_str(),
            alt_anomaly,
            avg_timestep);
  }
}


void AircraftModel::finalise(Averager& all_alt_start)
{
  json_trace_file << "]}\n";
  json_trace_file.close();
  if (!live)
    return;
  alt_start.calculate();
  alt_end.calculate();
  all_alt_start.add(alt_start.get_avg());
  baro_error.calculate();
}

void AircraftModel::set_wind_if_not_available(const SpeedVector& wind_avg)
{
  assert(replay);
  if (!Calculated().estimated_wind_available) {
    replay->SetCalculated().estimated_wind = wind_avg;
  }
}


TurnModeList AircraftModel::gen_turnmodelist(const EncounterMapStore::EncounterInfo &info) const
{
  const int t0 = info.time_start-EncounterMapStore::TYP_TRAIL;
  const int t1 = info.time_start+1;
  return trail.gen_turnmodelist(t0, t1);
}


//////////////////////////////////////////////////////////////////////////////////////////////////

static int n_write = 0;

void AircraftModel::write_encounter_open(const EncounterMapStore::EncounterInfo& info, const double distance_threshold)
{
  std::ofstream json_encounter_file;
  json_encounter_file.open(info.json_filename);
  json_encounter_file << "{\n  "
        << std::setprecision(1) << std::fixed
        << "\"first_launch\": " << (int)first_launch << ", "
        << "\"time_start\": " << (int)info.time_start << ", "
        << "\"time_end\": " << (int)info.time_end << ", "
        << "\"time_close\": " << info.time_close << ", "
        << "\"d_min\": " << info.d_min << ", "
        << "\"d_threshold\": " << distance_threshold << ", "
        << "\"time_pred\": " << info.time_pred << ", "
        << "\"v_max\": " << info.v_max << ", "
        << std::setprecision(8)
        << "\"latitude\": " << info.origin.latitude.Degrees() << ", "
        << "\"longitude\": " << info.origin.longitude.Degrees() << ", "
        << std::setprecision(2)
        << "\"p_close\": " << 1-info.p_free << ", "
        << "\n  \"aircraft\": [\n";
  json_encounter_file.close();
  n_write = 0;
}

void AircraftModel::write_encounter_close(const EncounterMapStore::EncounterInfo& info)
{
  std::ofstream json_encounter_file;
  json_encounter_file.open(info.json_filename, std::ofstream::out | std::ofstream::app);
  json_encounter_file << "]}\n";
  json_encounter_file.close();
}

bool AircraftModel::other_visible(const EncounterMapStore::EncounterInfo& info,
                                  const unsigned id_target) const
{
  for (auto&& p: trail) {

    if (!p.within_time(info.time_start - EncounterMapStore::TYP_TRAIL,
                       info.time_end + EncounterMapStore::HYS_TRAIL)) {
      continue;
    }
    if (p.present(id_target))
      return true;
  }
  return false;
}

void AircraftModel::write_encounter(const EncounterMapStore::EncounterInfo& info,
                                    const unsigned id_target) const
{
  std::ofstream json_encounter_file;
  json_encounter_file.open(info.json_filename, std::ofstream::out | std::ofstream::app);

  if (n_write>0) {
    json_encounter_file << ",\n";
  }
  json_encounter_file << " {\n"
        << "   \"id\": \"" << id << "\",\n"
        << "   \"frinfo\": \"" << fr_info << "\",\n"
        << "   \"frid\": \"" << fr_id << "\",\n"
        << "   \"turn_mode_list\": \"" << gen_turnmodelist(info).string() << "\",\n"
        << "   \"in_flock\": " << truefalse(in_flock) << ",\n"
        << "   \"trace\": [\n";

  FILE* fenc;

  if (debug) {
    char buf[80];
    sprintf(buf, "encounter_%05d-%s.txt", info.id, id.c_str());
    fenc = fopen(buf,"w");
    fprintf(fenc, "# id time_start time_end time_close d_min time_pred v_max\n");
    fprintf(fenc, "# \"%s\" %g %g %d %g %g %g\n", id.c_str(), info.time_start, info.time_end,
            info.time_close, info.d_min, info.time_pred, info.v_max);
    fprintf(fenc, "#\n");
    fprintf(fenc, "# t x y alt_baro alt_gps v_ias v hdg turnrate bank pitch turn_mode lon lat actual range elevation_angle azimuth_angle inclination_angle ang_size occlusion focus plausible fix_acc\n");
  }

  bool cont = false;
  bool plausible = true;

  Averager visibility_avg;

  for (auto&& p: trail) {

    if (!p.within_time(info.time_start - EncounterMapStore::TYP_TRAIL,
                       info.time_end + EncounterMapStore::HYS_TRAIL)) {
      continue;
    }

    const FlatPoint fp = info.project_loc_wind(p);
    const Aspect aspect = p.lookup_aspect(id_target);
    const Visibility visibility(aspect);

    if (n_write<2) {
      plausible &= p.plausible;
      if (p.pos.time-info.time_start<= 0) {
        visibility_avg.add(visibility.focus_factor * (1-visibility.occlusion));
      }
    }

    if (debug) {
      fprintf(fenc, "%g %g %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %.1f %d %.8g %.8g %d %.0f %.1f %.1f %.1f %.1f %.1f %.1f %d %.1f\n",
              p.pos.time-info.time_start, fp.x, fp.y,
              p.pos.baro_altitude, p.pos.gps_altitude,
              p.v_ias,
              p.v_wind.norm, p.v_wind.bearing.Degrees(), p.turn_rate_wind.Degrees(),
              p.bank_angle.Degrees(),
              p.pitch_angle.Degrees(),
              p.yaw_angle.Degrees(),
              (int)p.turn_mode,
              p.pos.location.longitude.Degrees(), p.pos.location.latitude.Degrees(),
              p.actual,
              aspect.range,
              aspect.elevation_angle.Degrees(),
              aspect.azimuth_angle.Degrees(),
              aspect.inclination_angle.Degrees(),
              visibility.angular_size.Degrees(),
              visibility.occlusion,
              visibility.focus_factor,
              p.plausible,
              p.fix_acc
              );
    }

    if (cont) {
      json_encounter_file << ",\n";
    } else {
      cont = true;
    }

    json_encounter_file << std::setprecision(1) << std::fixed
          << "      {"
          << "\"t\": " << p.pos.time-info.time_start << ", "
          << "\"x\": " << fp.x << ", "
          << "\"y\": " << fp.y << ", "
          << "\"alt_baro\": " << p.pos.baro_altitude << ", "
          << "\"alt_gps\": " << p.pos.gps_altitude << ", "
          << "\"v_ias\": " << p.v_ias << ", "
          << "\"v\": " << p.v_wind.norm << ", "
          << "\"hdg\": " << p.v_wind.bearing.Degrees() << ", "
          << "\"bank\": " << p.bank_angle.Degrees() << ", "
          << "\"pitch\": " << p.pitch_angle.Degrees() << ", "
          << "\"yaw\": " << p.yaw_angle.Degrees();

    if (n_write<2) {
      json_encounter_file << ", "
            << "\"turnrate\": " << p.turn_rate_wind.Degrees() << ", "
            << "\"turn_mode\": \"" << TurnModeList::to_string(p.turn_mode) << "\", "
            << "\"actual\": " << truefalse(p.actual) << ", "
            << "\"plausible\": " << truefalse(p.plausible) << ", "
            << "\"fix_acc\": " << p.fix_acc << ", "
            << "\"range\": " << aspect.range << ", "
            << "\"elevation_angle\": " << aspect.elevation_angle.Degrees() << ", "
            << "\"azimuth_angle\": " << aspect.azimuth_angle.Degrees() << ", "
            << "\"inclination_angle\": " << aspect.inclination_angle.Degrees() << ", "
            << "\"ang_size\": " << visibility.angular_size.Degrees() << ", "
            << "\"occlusion\": " << visibility.occlusion << ", "
            << "\"focus_factor\": " << visibility.focus_factor;
    }
    json_encounter_file << "}";

  }
  if (debug) {
    fclose(fenc);
  }

  visibility_avg.calculate();

  json_encounter_file << "\n    ],\n    "
        << "\"plausible\": " << truefalse(plausible) << ", "
        << std::setprecision(2)
        << "\"visibility_avg\": " << visibility_avg.get_avg() << "}\n";

  json_encounter_file.close();
  n_write++;
}


double AircraftModel::update_baro_altitude(double& mix)
{
  const MoreData &basic = replay->Basic();
  const double baro_altitude = basic.baro_altitude + baro_offset;
  const double err = basic.gps_altitude-baro_altitude;
  mix = (1-MIX_BARO)*basic.gps_altitude + MIX_BARO*baro_altitude;
  baro_error.add((basic.gps_altitude-mix)*(basic.gps_altitude-mix)+(baro_altitude-mix)*(baro_altitude-mix));
  baro_offset += ALPHA_BARO*err;
  return baro_altitude;
}

const Aspect AircraftModel::get_aspect(const AircraftModel& target) const
{
  const GeoVector v(interp_loc.location, target.interp_loc.location);
  const TrigAngle ang(v.bearing);
  const double x_inertial[3] = {v.distance*ang.c, v.distance*ang.s, interp_loc.gps_altitude-target.interp_loc.gps_altitude};
  return euler.get_aspect(x_inertial);
}

void AircraftModel::calc_aspect(const AircraftModel& target)
{
  //printf("%d->%d  %.0f %.0f %.0f\n", a.idi, b.idi, a_aspect.range, 180/M_PI*a_aspect.elevation_angle, 180/M_PI*a_aspect.azimuth_angle);
  //printf("%d<-%d  %.0f %.0f %.0f\n", a.idi, b.idi, b_aspect.range, 180/M_PI*b_aspect.elevation_angle, 180/M_PI*b_aspect.azimuth_angle);
  trail.back().aspects[target.idi] = get_aspect(target);
}

void AircraftModel::reset()
{
  replay->Rewind();

  replay_ok = true;
  circling_computer.Reset();
  wind_computer.Reset();

  flight_time_start = 0;

  if (debug) {
    char buf[80];
    sprintf(buf,"trace-%s.txt", id.c_str());
    ftrace = fopen(buf,"w");
    fprintf(ftrace,"# time lon lat alt_baro alt_gps alt_mix\n");

    sprintf(buf,"tracei-%s.txt", id.c_str());
    fitrace = fopen(buf,"w");
    fprintf(fitrace,"# time lon lat alt_gps\n");
  }
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
