// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright The XCSoar Project

#include "FlightCollectionEncounter.hpp"
#include "Geo/GeoBounds.hpp"
#include "Geo/Geoid.hpp"
#include "Math/Vector.hpp"
#include <unordered_set>
#include <algorithm>
#include <fstream>
#include <sstream>

using namespace MultiAircraft;

bool FlightCollectionEncounter::skip_encounter_processing = false;

////////////////////////////////////////////////////////////////////////////
namespace
{

  bool AverageAircraftWind(const std::vector<const AircraftModel *> &aircraft,
                           const TimeStamp t_min,
                           const TimeStamp t_max,
                           SpeedVector &wind)
  {
    Vector wind_acc(0, 0);
    unsigned num_aircraft = 0;

    for (const auto *aircraft_model : aircraft)
    {
      SpeedVector aircraft_wind;
      if (!aircraft_model->get_average_wind(t_min, t_max, aircraft_wind))
      {
        continue;
      }

      wind_acc += Vector(aircraft_wind);
      ++num_aircraft;
    }

    if (num_aircraft == 0)
    {
      return false;
    }

    wind = SpeedVector(wind_acc.y / num_aircraft, wind_acc.x / num_aircraft);
    return true;
  }

  struct iterator_hash
  {
    size_t operator()(std::list<AircraftModel>::const_iterator it) const
    {
      return it->idi;
    }
  };

  typedef std::unordered_set<std::list<AircraftModel>::const_iterator, iterator_hash> CandidateFlockPoints;

  static SpeedVector average_wind(const AircraftModel &a, const AircraftModel &b)
  {
    const Vector wind_acc = Vector(a.Calculated().estimated_wind) + Vector(b.Calculated().estimated_wind);
    return SpeedVector(wind_acc.y / 2, wind_acc.x / 2);
  }

  static GeoPoint center(const AircraftModel &a, const AircraftModel &b)
  {
    return a.interp_loc.location.Interpolate(b.interp_loc.location, 0.5);
  }

  ////////////////////////////////////////////////////////////////////////////

  static double distance_horiz(const CatmullRomInterpolator::Record &a,
                               const CatmullRomInterpolator::Record &b)
  {
    return a.location.Distance(b.location);
  }

  static double distance_vert(const CatmullRomInterpolator::Record &a,
                              const CatmullRomInterpolator::Record &b)
  {
    return fabs((a.gps_altitude) - (b.gps_altitude));
  }

  static double distance(const CatmullRomInterpolator::Record &a,
                         const CatmullRomInterpolator::Record &b)
  {
    const double d_horiz = distance_horiz(a, b);
    const double d_vert = distance_vert(a, b);
    return sqrt(d_horiz * d_horiz + d_vert * d_vert);
  }

  ////////////////////////////////////////////////////////////////////////////

  static constexpr double sqr(const double x)
  {
    return x * x;
  }

  static constexpr double z_normal(const double x, const double mu, const double sigma)
  {
    return (x - mu) / sigma;
  }

  static double cdf_normal(const double z)
  {
    return 0.5 * (1 + erf(z - 0.707106781188));
  }

  static double pdf_normal(const double z, const double sigma)
  {
    return exp(-z * z) / (sqrt(2 * M_PI) * sigma);
  }

  static double expected_distance(const double D, const double d, const double sigma)
  {
    double h_acc = 0;
    double p_acc = 0;
    for (double x = -D; x <= D; x += 0.5)
    {
      const double z = z_normal(x, d, sigma);
      const double p = pdf_normal(z, sigma);
      const double h = abs(x);
      h_acc += h * p;
      p_acc += p;
    }
    h_acc /= p_acc;
    return h_acc;
  }

  ////////////////////////////////////////////////////////////////////////////

  static GeoPoint calc_ll_delta(const GeoPoint &loc, const double range)
  {
    const GeoVector v(range, Angle::Degrees(-45));
    return v.EndPoint(loc) - loc;
  }

} // namespace

bool FlightCollectionEncounter::process(const TimeStamp t)
{
  const bool was_first = first;
  const bool ok = FlightCollection::process(t);

  if (skip_encounter_processing)
  {
    return ok;
  }

  if (first)
  {
    // populate id list
    for (auto &i : group)
    {
      flock_algorithm.ids.push_back(i.id);
    }

    return ok;
  }
  else if (was_first)
  {
    delta_proximity = calc_ll_delta(loc_general, DISTANCE * 2);
    delta_visibility = calc_ll_delta(loc_general, DISTANCE_VISIBILITY);
    delta_flock = calc_ll_delta(loc_general, flock_algorithm.epsilon);
  }

  visibility_update();
  encounter_update(t);
  update_vignettes(t);
  update_airspace_incursions(t);
  update_terrain_events(t);
  time_close += encounter_store.erase_expired(t, group, DISTANCE, alt_start_av + HEIGHT_THRESHOLD_M);
  return ok;
}

void FlightCollectionEncounter::update_terrain_events(const TimeStamp t)
{
  if (terrain == nullptr || terrain->empty())
    return;

  for (auto &[_, active] : active_terrain_events)
    active.seen = false;

  for (const auto &a : group)
  {
    if (!a.live || !a.valid)
      continue;

    const auto terrain_height = terrain->GetHeight(a.interp_loc.location);
    if (!terrain_height.has_value())
      continue;

    const double terrain_distance = a.interp_loc.gps_altitude - *terrain_height;
    if (terrain_distance > TERRAIN_CLEARANCE_M)
      continue;

    const auto wind = a.Calculated().estimated_wind;
    auto it = active_terrain_events.find(a.idi);
    if (it == active_terrain_events.end())
    {
      ActiveTerrainEvent active{
          Vignette(a.idi, t, a.flight_date_utc_start,
                   a.interp_loc.location,
                   a.interp_loc.baro_altitude,
                   wind),
          {},
          terrain_distance,
          true};
      active.distance_samples.emplace_back(t, terrain_distance);
      active_terrain_events.emplace(a.idi, std::move(active));
      continue;
    }

    auto &active = it->second;
    active.vignette.time_end = t;
    active.vignette.wind_acc += Vector(wind);
    ++active.vignette.num_wind;
    active.vignette.wind = SpeedVector(active.vignette.wind_acc.y / active.vignette.num_wind,
                                       active.vignette.wind_acc.x / active.vignette.num_wind);
    active.distance_samples.emplace_back(t, terrain_distance);
    active.min_distance = std::min(active.min_distance, terrain_distance);
    active.seen = true;
  }

  for (auto it = active_terrain_events.begin(); it != active_terrain_events.end();)
  {
    if (it->second.seen)
    {
      ++it;
      continue;
    }

    completed_terrain_events[it->first].push_back(std::move(it->second));
    it = active_terrain_events.erase(it);
  }
}

void FlightCollectionEncounter::update_airspace_incursions(const TimeStamp t)
{
  if (!openaip_airspaces.IsEnabled())
    return;

  std::unordered_set<unsigned> current_airspace_aircraft;
  std::unordered_set<IncursionKey, IncursionKeyHash> current_incursion_hits;

  for (auto &[_, active] : active_incursions)
    active.seen = false;

  for (const auto &a : group)
  {
    if (!a.live || !a.valid)
      continue;

    current_airspace_aircraft.emplace(a.idi);

    auto ground_it = ground_reference_by_aircraft.find(a.idi);
    if (ground_it == ground_reference_by_aircraft.end())
      ground_it = ground_reference_by_aircraft.emplace(a.idi, a.interp_loc.baro_altitude).first;
    else
      ground_it->second = std::min(ground_it->second, a.interp_loc.baro_altitude);

    const auto hits = openaip_airspaces.Query(a.interp_loc.location,
                                              a.interp_loc.baro_altitude,
                                              ground_it->second);
    const auto wind = a.Calculated().estimated_wind;

    for (const auto &hit : hits)
    {
      const IncursionKey key{(unsigned)a.idi, hit.airspace_index};
      current_incursion_hits.emplace(key);

      auto it = active_incursions.find(key);
      if (it == active_incursions.end())
      {
        const bool seen_aircraft_prev_step = previous_airspace_aircraft.contains(a.idi);
        const bool was_incursion_prev_step = previous_incursion_hits.contains(key);
        if (!seen_aircraft_prev_step || was_incursion_prev_step)
          continue;

        ActiveIncursion active{
            hit.airspace_index,
            Vignette(a.idi, t, a.flight_date_utc_start,
                     a.interp_loc.location,
                     a.interp_loc.baro_altitude,
                     wind),
            {},
            hit.depth_m,
            true};
        active.depth_samples.emplace_back(t, hit.depth_m);
        active_incursions.emplace(key, std::move(active));
        continue;
      }

      auto &active = it->second;
      active.vignette.time_end = t;
      active.vignette.wind_acc += Vector(wind);
      ++active.vignette.num_wind;
      active.vignette.wind = SpeedVector(active.vignette.wind_acc.y / active.vignette.num_wind,
                                         active.vignette.wind_acc.x / active.vignette.num_wind);
      active.depth_samples.emplace_back(t, hit.depth_m);
      active.max_depth = std::max(active.max_depth, hit.depth_m);
      active.seen = true;
    }
  }

  for (auto it = active_incursions.begin(); it != active_incursions.end();)
  {
    if (it->second.seen)
    {
      ++it;
      continue;
    }

    completed_incursions[it->first.aircraft_id].push_back(std::move(it->second));
    it = active_incursions.erase(it);
  }

  previous_airspace_aircraft = std::move(current_airspace_aircraft);
  previous_incursion_hits = std::move(current_incursion_hits);
}

void FlightCollectionEncounter::update_vignettes(const TimeStamp t)
{
  if (!vignette_options.enabled)
  {
    return;
  }

  for (const auto &a : group)
  {
    if (!a.live || !a.valid)
      continue;

    const auto wind = a.Calculated().estimated_wind;
    auto it = vignette_map.find(a.idi);
    if (it == vignette_map.end())
    {
      vignette_map.emplace(a.idi,
                           Vignette(a.idi,
                                    t,
                                    a.flight_date_utc_start,
                                    a.interp_loc.location,
                                    a.interp_loc.baro_altitude,
                                    wind));
      continue;
    }

    auto &v = it->second;
    v.time_end = t;
    v.wind_acc += Vector(wind);
    ++v.num_wind;
    v.wind = SpeedVector(v.wind_acc.y / v.num_wind,
                         v.wind_acc.x / v.num_wind);
  }
}

double FlightCollectionEncounter::get_average_h_acc() const
{
  Averager h_acc;
  for (auto &&a : group)
  {
    if (!a.live || !a.valid)
      continue;
    if (a.h_acc > 0)
    {
      h_acc.add(a.h_acc);
    }
  }
  if (!h_acc.empty())
  {
    h_acc.calculate();
    return h_acc.get_avg();
  }
  else
  {
    return 5.0; // default?
  }
}

void FlightCollectionEncounter::visibility_update()
{
  // encounters detect/update
  for (auto i = group.begin(); i != group.end(); ++i)
  {
    AircraftModel &a = *i;
    if (!a.live || !a.valid)
      continue;

    const GeoBounds visibility_bounds(a.interp_loc.location + delta_visibility, a.interp_loc.location - delta_visibility);

    auto j = i;
    while (++j != group.end())
    {
      AircraftModel &b = *j;
      if (!b.live || !b.valid)
        continue;

      if (!visibility_bounds.IsInside(b.interp_loc.location))
      {
        continue; // fast exit, clearly out of bounds
      }
      const double d_vert = distance_vert(a.interp_loc, b.interp_loc);
      if (d_vert > DISTANCE_VISIBILITY)
        continue; // fast exit, clearly out of bounds

      a.calc_auxiliary(b);
      b.calc_auxiliary(a);
    }
  }
  // TODO: do something like erase_expired in map
}

void FlightCollectionEncounter::encounter_update(const TimeStamp t)
{
  const double sigma_limit = 2.5;

  const double h_acc_av = get_average_h_acc();
  const int tint = (int)t.ToDuration().count();

  const bool do_flock = (tint % 4 == 0);
  CandidateFlockPoints potential_flock_participants;

  // encounters detect/update
  for (auto i = group.begin(); i != group.end(); ++i)
  {
    const AircraftModel &a = *i;
    if (!a.live || !a.valid)
      continue;
    const BrokenDate dt = a.flight_date_utc_start;

    const double a_h_acc = a.h_acc > 0 ? a.h_acc : h_acc_av;
    const GeoBounds proximity_bounds(a.interp_loc.location + delta_proximity, a.interp_loc.location - delta_proximity);
    const GeoBounds flock_bounds(a.interp_loc.location + delta_flock, a.interp_loc.location - delta_flock);

    auto j = i;
    while (++j != group.end())
    {
      const AircraftModel &b = *j;
      if (!b.live || !b.valid || b.aliased(a))
        continue;

      const double d_vert = distance_vert(a.interp_loc, b.interp_loc);

      // XXXXX FLOCK
      //  if a or b within flock distance, add them to a set of iterators
      if (do_flock && flock_bounds.IsInside(b.interp_loc.location))
      {
        potential_flock_participants.insert(i);
        potential_flock_participants.insert(j);
      }

      if (d_vert > 3 * DISTANCE)
        continue; // fast exit, clearly out of bounds

      if (!proximity_bounds.IsInside(b.interp_loc.location))
      {
        continue; // fast exit, clearly out of bounds
      }

      // TODO JMW calculate actual effective distance
      const double effective_distance = get_effective_distance(a, b);

      const double v_acc = sqrt(sqr(a.v_acc) + sqr(b.v_acc));

      if (d_vert - sigma_limit * v_acc > effective_distance / 2)
        continue; // fast exit, too many sigma outside, out of bounds
                  //
      const double p_close_v = cdf_normal(z_normal(effective_distance / 2, d_vert, v_acc));

      const double d_horiz = distance_horiz(a.interp_loc, b.interp_loc);
      const double d_abs = sqrt(d_horiz * d_horiz + d_vert * d_vert);

      const double b_h_acc = b.h_acc > 0 ? b.h_acc : h_acc_av;
      const double h_acc = sqrt(sqr(a_h_acc) + sqr(b_h_acc));
      if (d_abs - sigma_limit * h_acc > effective_distance)
        continue;
      const double p_close_h = cdf_normal(z_normal(effective_distance, d_horiz, h_acc));

      const double p_close = p_close_v * p_close_h;

      if (p_close > P_THRESHOLD)
      {
        const double d_last = distance(a.interp_loc_last, b.interp_loc_last);
        const double v = (d_abs - d_last);
        const double d_exp = expected_distance(effective_distance, d_abs, h_acc); // approximate
        encounter_store.update(a.idi, b.idi, t, dt, center(a, b), a.interp_loc.baro_altitude, average_wind(a, b), d_exp, v, 1 - p_close);
      }
    }
  }

  ///////////////////////////
  // FLOCK process
  if (!do_flock)
  {
    return;
  }

  // for each item in the set, compile sorted list of points
  std::list<Flock::IndexPoint> P;
  for (auto i : potential_flock_participants)
  {
    const FlatPoint fp = project_loc(i->get_location());
    P.push_back(Flock::IndexPoint(Flock::Point(fp.x, fp.y), i->idi));
  }

  // sort in increasing x
  P.sort([](const Flock::IndexPoint &a, const Flock::IndexPoint &b)
         { return a.x <= b.x; });

  flock_algorithm.process_time(tint, P);
  flock_algorithm.mark_in_flock(group);
}

void FlightCollectionEncounter::finalise()
{
  flock_algorithm.finalise();
  time_close += encounter_store.erase_expired(TimeStamp::Undefined(), group, DISTANCE, alt_start_av + HEIGHT_THRESHOLD_M);
  write_vignette_file();
  for (auto &[key, active] : active_incursions)
    completed_incursions[key.aircraft_id].push_back(std::move(active));
  active_incursions.clear();
  for (auto &[idi, active] : active_terrain_events)
    completed_terrain_events[idi].push_back(std::move(active));
  active_terrain_events.clear();
  write_incursion_files();
  write_terrain_files();
  FlightCollection::finalise();
}

void FlightCollectionEncounter::write_vignette_file()
{
  if (!vignette_options.enabled)
  {
    return;
  }

  auto subject_it = std::find_if(group.begin(), group.end(),
                                 [this](const AircraftModel &a)
                                 {
                                   return a.id == vignette_options.subject;
                                 });
  if (subject_it == group.end())
  {
    std::cerr << "vignette subject not found: " << vignette_options.subject << "\n";
    return;
  }

  const TimeStamp t_start = TimeStamp(FloatDuration(vignette_options.start_time));
  const TimeStamp t_end = TimeStamp(FloatDuration(vignette_options.end_time));

  std::vector<unsigned> selected_ids;
  selected_ids.push_back(subject_it->idi);

  for (const auto &a : group)
  {
    if (a.idi == subject_it->idi)
      continue;

    auto it = vignette_map.find(a.idi);
    if (it == vignette_map.end())
      continue;

    const auto &v = it->second;
    if (v.time_end < t_start || v.time_start > t_end)
      continue;

    if (subject_it->within_horizontal_distance(a, t_start, t_end, FlightFlock::epsilon_distance_m))
    {
      selected_ids.push_back(a.idi);
    }
  }

  auto subject_vignette_it = vignette_map.find(subject_it->idi);
  if (subject_vignette_it == vignette_map.end())
  {
    std::cerr << "vignette subject has no collected trail: " << vignette_options.subject << "\n";
    return;
  }

  if (subject_vignette_it->second.time_end < t_start ||
      subject_vignette_it->second.time_start > t_end)
  {
    std::cerr << "vignette subject outside requested window: " << vignette_options.subject << "\n";
    return;
  }

  std::sort(selected_ids.begin() + 1, selected_ids.end());

  boost::json::array json_aircraft;
  std::vector<const AircraftModel *> included_aircraft;
  GeoPoint subject_origin = subject_vignette_it->second.origin;
  for (const auto idi : selected_ids)
  {
    auto aircraft_it = std::find_if(group.begin(), group.end(),
                                    [idi](const AircraftModel &a)
                                    {
                                      return a.idi == (int)idi;
                                    });
    auto v_it = vignette_map.find(idi);
    if (aircraft_it == group.end() || v_it == vignette_map.end())
      continue;

    Vignette clipped = v_it->second;
    clipped.time_start = std::max(clipped.time_start, t_start);
    clipped.time_end = std::min(clipped.time_end, t_end);
    GeoPoint clipped_origin;
    TimeStamp clipped_origin_time;
    if (aircraft_it->get_first_location(clipped.time_start, clipped.time_end,
                                        clipped_origin, clipped_origin_time))
    {
      clipped.origin = clipped_origin;
      clipped.origin_time = clipped_origin_time;
    }
    clipped.finalise();

    if (idi == (unsigned)subject_it->idi)
      subject_origin = clipped.origin;

    json_aircraft.emplace_back(aircraft_it->write_vignette(clipped));
    included_aircraft.push_back(&*aircraft_it);
  }

  if (json_aircraft.empty())
  {
    return;
  }

  std::ostringstream filename;
  filename << "vignette-"
           << vignette_options.subject
           << "-" << vignette_options.start_time
           << "-" << vignette_options.end_time
           << ".json";

  const double geoid_offset = EGM96::LookupSeparation(subject_origin);
  SpeedVector wind;
  const bool wind_available = AverageAircraftWind(included_aircraft, t_start, t_end, wind);

  boost::json::object json_info = {
      {"time_start", vignette_options.start_time},
      {"time_end", vignette_options.end_time},
      {"subject", vignette_options.subject},
      {"distance_threshold", FlightFlock::epsilon_distance_m},
      {"latitude", subject_origin.latitude.Degrees()},
      {"longitude", subject_origin.longitude.Degrees()},
      {"geoid_offset", geoid_offset},
      {"wind_speed", wind_available ? boost::json::value(wind.norm) : boost::json::value(nullptr)},
      {"wind_bearing", wind_available ? boost::json::value(wind.bearing.Degrees()) : boost::json::value(nullptr)},
      {"aircraft", json_aircraft}};

  std::ofstream file(filename.str());
  file << boost::json::serialize(json_info);
}

void FlightCollectionEncounter::write_incursion_files()
{
  if (!openaip_airspaces.IsEnabled())
    return;

  for (auto &[idi, incursions] : completed_incursions)
  {
    auto aircraft_it = std::find_if(group.begin(), group.end(),
                                    [idi](const AircraftModel &a)
                                    {
                                      return a.idi == (int)idi;
                                    });
    if (aircraft_it == group.end())
      continue;

    for (std::size_t index = 0; index < incursions.size(); ++index)
    {
      auto &incursion = incursions[index];
      incursion.vignette.finalise();
      const auto &metadata = openaip_airspaces.GetMetadata(incursion.airspace_index);
      const double geoid_offset = EGM96::LookupSeparation(incursion.vignette.origin);

      boost::json::array aircraft_json;
      aircraft_json.emplace_back(aircraft_it->write_incursion(incursion.vignette,
                                                              incursion.depth_samples));

      boost::json::object json_info = {
          {"time_start", (int)incursion.vignette.time_start.ToDuration().count()},
          {"time_end", (int)incursion.vignette.time_end.ToDuration().count()},
          {"subject", aircraft_it->id},
          {"airspace_name", metadata.name},
          {"airspace_type", metadata.type},
          {"airspace_class", metadata.icao_class_name},
          {"lower_limit", metadata.lower_label},
          {"upper_limit", metadata.upper_label},
          {"depth_max", incursion.max_depth},
          {"latitude", incursion.vignette.origin.latitude.Degrees()},
          {"longitude", incursion.vignette.origin.longitude.Degrees()},
          {"geoid_offset", geoid_offset},
          {"wind_speed", incursion.vignette.wind.norm},
          {"wind_bearing", incursion.vignette.wind.bearing.Degrees()},
          {"aircraft", aircraft_json}};

      std::ostringstream filename;
      filename << "incursion-" << aircraft_it->id << "-" << index << ".json";
      std::ofstream file(filename.str());
      file << boost::json::serialize(json_info);
    }
  }
}

void FlightCollectionEncounter::write_terrain_files()
{
  if (terrain == nullptr || terrain->empty())
    return;

  for (auto &[idi, events] : completed_terrain_events)
  {
    auto aircraft_it = std::find_if(group.begin(), group.end(),
                                    [idi](const AircraftModel &a)
                                    {
                                      return a.idi == (int)idi;
                                    });
    if (aircraft_it == group.end())
      continue;

    for (std::size_t index = 0; index < events.size(); ++index)
    {
      auto &event = events[index];
      event.vignette.finalise();
      const double geoid_offset = EGM96::LookupSeparation(event.vignette.origin);

      boost::json::array aircraft_json;
      aircraft_json.emplace_back(aircraft_it->write_terrain(event.vignette,
                                                            event.distance_samples));

      boost::json::object json_info = {
          {"time_start", (int)event.vignette.time_start.ToDuration().count()},
          {"time_end", (int)event.vignette.time_end.ToDuration().count()},
          {"subject", aircraft_it->id},
          {"distance_min", event.min_distance},
          {"latitude", event.vignette.origin.latitude.Degrees()},
          {"longitude", event.vignette.origin.longitude.Degrees()},
          {"geoid_offset", geoid_offset},
          {"wind_speed", event.vignette.wind.norm},
          {"wind_bearing", event.vignette.wind.bearing.Degrees()},
          {"aircraft", aircraft_json}};

      std::ostringstream filename;
      filename << "terrain-" << aircraft_it->id << "-" << index << ".json";
      std::ofstream file(filename.str());
      file << boost::json::serialize(json_info);
    }
  }
}

double FlightCollectionEncounter::get_effective_distance(const AircraftModel &a,
                                                         const AircraftModel &b) const
{
  const AuxiliaryPair &auxiliary = a.get_latest_auxiliary(b);
  return DISTANCE * auxiliary.second.distance_scale;
}

/////////////////////////////////////////////////////////////////////

std::string FlightCollectionEncounter::get_symbol(const AircraftModel &m) const
{
  if (m.mark)
  {
    return std::string("\033[7;31m#\033[0m");
  }
  else if (m.in_flock)
  {
    return std::string("|");
  }
  else
  {
    return m.get_symbol();
  }
}

boost::json::object FlightCollectionEncounter::record_summary() const
{
  boost::json::array penalty_summary;

  // Create a file of "penalties" for the day - this is the sum of the maximum distance of each incursion
  for (auto &&a : group)
  {
    if (a.live)
    {
      // penalty is the sum of the "penalties" minus the buffer
      int score = std::max(0, (int)a.penalty - SCORE_BUFFER);
      boost::json::object penalties = {
          {"id", a.id},
          {"idi", a.idi},
          {"penalty", a.penalty},
          {"n_encounters", a.n_encounters},
          {"score", score},
      };
      penalty_summary.emplace_back(penalties);
    }
  }

  int num_aircraft = 0;
  for (auto &&a : group)
  {
    if (a.live)
    {
      num_aircraft++;
    }
  }

  boost::json::object analysis_summary = {
      {"num_flightsecs", num_flightsecs},
      {"num_aircraft", num_aircraft},
      {"time_close", (int)time_close.count()},
      {"first_launch", (int)AircraftModel::first_launch.ToDuration().count()},
      {"alt_max", alt_max},
      {"d_threshold", DISTANCE},
      {"score_buffer", SCORE_BUFFER},
  };

  boost::json::object flight_summary = FlightCollection::record_summary();

  return {
      {"analysis", analysis_summary},
      {"flight", flight_summary},
      {"penalty", penalty_summary},
  };
}
