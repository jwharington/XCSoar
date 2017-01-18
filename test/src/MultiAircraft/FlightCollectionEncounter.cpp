#include "FlightCollectionEncounter.hpp"
#include "Geo/GeoBounds.hpp"
#include <unordered_set>

extern bool debug;

using namespace MultiAircraft;

////////////////////////////////////////////////////////////////////////////
struct iterator_hash
{
  size_t operator()(std::list<AircraftModel>::const_iterator it) const
  {
    return it->idi;
  }
};

typedef std::unordered_set<std::list<AircraftModel>::const_iterator, iterator_hash> CandidateFlockPoints;


static SpeedVector average_wind(const AircraftModel& a, const AircraftModel& b) {
  const Vector wind_acc = Vector(a.Calculated().estimated_wind) + Vector(b.Calculated().estimated_wind);
  return SpeedVector (wind_acc.y/2, wind_acc.x/2);
}

static GeoPoint center(const AircraftModel& a, const AircraftModel& b) {
  return a.interp_loc.location.Interpolate(b.interp_loc.location, 0.5);
}

////////////////////////////////////////////////////////////////////////////

static double distance_horiz(const CatmullRomInterpolator::Record& a,
                             const CatmullRomInterpolator::Record& b)
{
  return a.location.Distance(b.location);
}

static double distance_vert(const CatmullRomInterpolator::Record& a,
                            const CatmullRomInterpolator::Record& b)
{
  return fabs((a.gps_altitude) - (b.gps_altitude));
}

static double distance(const CatmullRomInterpolator::Record& a,
                       const CatmullRomInterpolator::Record& b)
{
  const double d_horiz = distance_horiz(a, b);
  const double d_vert = distance_vert(a, b);
  return sqrt(d_horiz*d_horiz+d_vert*d_vert);
}

////////////////////////////////////////////////////////////////////////////

static constexpr double sqr(const double x) {
  return x*x;
}

static constexpr double z_normal(const double x, const double mu, const double sigma)
{
  return (x-mu)/sigma;
}

static double cdf_normal(const double z)
{
  return 0.5*(1+erf(z-0.707106781188));
}

static double pdf_normal(const double z, const double sigma)
{
  return exp(-z*z)/(sqrt(2*M_PI)*sigma);
}

static double expected_distance(const double D, const double d, const double sigma)
{
  double h_acc = 0;
  double p_acc = 0;
  for (double x=-D; x<=D; x+= 0.5) {
    const double z = z_normal(x, d, sigma);
    const double p = pdf_normal(z, sigma);
    const double h = abs(x);
    h_acc += h*p;
    p_acc += p;
  }
  h_acc /= p_acc;
  return h_acc;
}

////////////////////////////////////////////////////////////////////////////

static GeoPoint calc_ll_delta(const GeoPoint& loc, const double range)
{
  const GeoVector v(range, Angle::Degrees(-45));
  return v.EndPoint(loc)-loc;
}

bool FlightCollectionEncounter::process(const int t)
{
  const bool was_first = first;
  const bool ok = FlightCollection::process(t);
  if (first) {
    // populate id list
    for (auto &i: group) {
      flock_algorithm.ids.push_back(i.id);
    }

    return ok;
  } else if (was_first) {
    delta_proximity = calc_ll_delta(loc_general, DISTANCE*2);
    delta_visibility = calc_ll_delta(loc_general, DISTANCE_VISIBILITY);
    delta_flock = calc_ll_delta(loc_general, flock_algorithm.epsilon);
  }

  visibility_update();
  encounter_update(t);
  time_close += encounter_store.erase_expired(t, group, DISTANCE, debug);
  return ok;
}


double FlightCollectionEncounter::get_average_h_acc() const
{
  Averager h_acc;
  for (auto&& a : group) {
    if (!a.live || !a.valid)
      continue;
    if (a.h_acc>0) {
      h_acc.add(a.h_acc);
    }
  }
  if (!h_acc.empty()) {
    h_acc.calculate();
    return h_acc.get_avg();
  } else {
    return 5.0; // default?
  }
}

void FlightCollectionEncounter::visibility_update()
{
  // encounters detect/update
  for (auto i = group.begin(); i!= group.end(); ++i) {
    AircraftModel& a = *i;
    if (!a.live || !a.valid)
      continue;

    const GeoBounds visibility_bounds(a.interp_loc.location+delta_visibility, a.interp_loc.location-delta_visibility);

    auto j = i;
    while (++j != group.end()) {
      AircraftModel& b = *j;
      if (!b.live || !b.valid)
        continue;

      if (!visibility_bounds.IsInside(b.interp_loc.location)) {
        continue; // fast exit, clearly out of bounds
      }
      const double d_vert = distance_vert(a.interp_loc, b.interp_loc);
      if (d_vert > DISTANCE_VISIBILITY)
        continue; // fast exit, clearly out of bounds

      a.calc_aspect(b);
      b.calc_aspect(a);
    }
  }
  // TODO: do something like erase_expired in map
}

void FlightCollectionEncounter::encounter_update(const int t)
{
  const double sigma_limit = 2.5;

  const double h_acc_av = get_average_h_acc();

  const bool do_flock = (t % 4 == 0);
  CandidateFlockPoints potential_flock_participants;

  // encounters detect/update
  for (auto i = group.begin(); i!= group.end(); ++i) {
    const AircraftModel& a = *i;
    if (!a.live || !a.valid)
      continue;

    const double a_h_acc = a.h_acc>0? a.h_acc : h_acc_av;
    const GeoBounds proximity_bounds(a.interp_loc.location+delta_proximity, a.interp_loc.location-delta_proximity);
    const GeoBounds flock_bounds(a.interp_loc.location+delta_flock, a.interp_loc.location-delta_flock);

    auto j = i;
    while (++j != group.end()) {
      const AircraftModel& b = *j;
      if (!b.live || !b.valid)
        continue;

      const double d_vert = distance_vert(a.interp_loc, b.interp_loc);

      //XXXXX FLOCK
      // if a or b within flock distance, add them to a set of iterators
      if (do_flock && flock_bounds.IsInside(b.interp_loc.location)) {
        potential_flock_participants.insert(i);
        potential_flock_participants.insert(j);
      }

      if (d_vert > 3*DISTANCE)
        continue; // fast exit, clearly out of bounds

      if (!proximity_bounds.IsInside(b.interp_loc.location)) {
        continue; // fast exit, clearly out of bounds
      }

      const double v_acc = sqrt(sqr(a.v_acc)+sqr(b.v_acc));
      if (d_vert - sigma_limit*v_acc > DISTANCE/2)
        continue; // fast exit, too many sigma outside, out of bounds
      const double p_close_v = cdf_normal(z_normal(DISTANCE/2, d_vert, v_acc));

      const double d_horiz = distance_horiz(a.interp_loc, b.interp_loc);
      const double d_abs = sqrt(d_horiz*d_horiz+d_vert*d_vert);

      const double b_h_acc = b.h_acc>0? b.h_acc : h_acc_av;
      const double h_acc = sqrt(sqr(a_h_acc)+sqr(b_h_acc));
      if (d_abs - sigma_limit*h_acc > DISTANCE)
        continue;
      const double p_close_h = cdf_normal(z_normal(DISTANCE, d_horiz, h_acc));

      const double p_close = p_close_v*p_close_h;

      if (p_close > P_THRESHOLD) {
        const double d_last = distance(a.interp_loc_last, b.interp_loc_last);
        const double v = (d_abs-d_last);
        const double d_exp = expected_distance(DISTANCE, d_abs, h_acc); // approximate
        //printf("%.1f %.2f\t%.1f %.2f\t%.3f\t%.1f\t%f\n", d_vert, p_close_v, d_horiz, p_close_h, p_close, d_exp, h_acc);
        encounter_store.update(a.idi, b.idi, t, center(a,b), a.interp_loc.baro_altitude, average_wind(a,b), d_exp, v, 1-p_close);
      }
    }
  }

  ///////////////////////////
  // FLOCK process
  if (!do_flock) {
    return;
  }

  // for each item in the set, compile sorted list of points
  std::list<Flock::IndexPoint> P;
  for (auto i: potential_flock_participants) {
    const FlatPoint fp = project_loc(i->get_location());
    P.push_back(Flock::IndexPoint(Flock::Point(fp.x,fp.y), i->idi));
  }

  // sort in increasing x
  P.sort([](const Flock::IndexPoint & a, const Flock::IndexPoint & b) { return a.x <= b.x; });

  flock_algorithm.process_time(t, P);
  flock_algorithm.mark_in_flock(group);
}

void FlightCollectionEncounter::finalise()
{
  flock_algorithm.finalise();
  FlightCollection::finalise();
}

/////////////////////////////////////////////////////////////////////

std::string FlightCollectionEncounter::get_symbol(const AircraftModel& m) const
{
  if (m.mark) {
    return std::string("\033[7;31m#\033[0m");
  } else if (m.in_flock) {
    return std::string("|");
  } else {
    return m.get_symbol();
  }
}


void FlightCollectionEncounter::write_diagnostics_penalties() const
{
  // Create a file of "penalties" for the day - this is the sum of the maximum distance of each incursion
  FILE* fout = fopen("penalty.txt","w");
  fprintf(fout, "# id idi penalty n_encounters\n");
  for (auto&& a : group) {
    if (a.live) {
      fprintf(fout, "\"%s\" %d %f %d\n", a.id.c_str(), a.idi, a.penalty, a.n_encounters);
    }
  }
  fclose(fout);

  // Create the penalty file for the scorer to apply
  fout = fopen("scoring_penalty.txt","w");
  fprintf(fout, "# comp_id penalty\n");
  int score;
  for (auto&& a : group) {
    if (a.live) {
      // penalty is the sum of the "penalties" minus the buffer
      score = (int)a.penalty - SCORE_BUFFER;
      // Only if we have a postitive penalty output it.
      if (score > 0) {
        fprintf(fout, "\"%s\" %d\n", a.id.c_str(), score);
      }
    }
  }
  fclose(fout);
}

void FlightCollectionEncounter::write_diagnostics() const
{
  FlightCollection::write_diagnostics();
  write_diagnostics_penalties();

  FILE* fout = fopen("summary.txt","w");

  int num_aircraft = 0;
  for (auto&& a : group) {
    if (a.live) {
      num_aircraft++;
    }
  }

  fprintf(fout, "# num_flightsecs num_aircraft time_close alt_max\n");
  fprintf(fout, "%d %d %d %d\n", num_flightsecs, num_aircraft, time_close, (int)alt_max);
  fclose(fout);
}


/////////////////

