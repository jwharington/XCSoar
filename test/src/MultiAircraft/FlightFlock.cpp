#include "FlightFlock.hpp"
#include "AircraftModel.hpp"
#include "Geo/Flat/FlatPoint.hpp"
#include <iomanip>      // std::setprecision

using namespace MultiAircraft;

void FlightFlock::process_time(const int t, std::list<Flock::IndexPoint> &P)
{
  auto candidates = find_candidate_disks(P);
  // sort BBs by x
  candidates.sort([](const Flock::BoundingBox & a, const Flock::BoundingBox & b) { return a.ll.x <= b.ll.x; });

  auto prev = disk_store.begin();
  const bool prev_valid = prev != disk_store.end();

  disk_store.push_front(filter_candidates(candidates, t));
  auto &disks = disk_store.front();

  if (prev_valid) {
    connect_disks(disks, *prev);
    finalise_disks(*prev);
  }

  // set prune to true; will set to false when used next time step
  mark_all_prune(disks);
}

bool FlightFlock::find_flock(const int index) const
{
  auto it = disk_store.begin();
  if (it == disk_store.end())
    return false;
  for (auto &disk : *it) {
    if (disk.duration < min_duration)
      continue;
    if (disk.present(index))
      return true;
  }
  return false;
}


void FlightFlock::finalise()
{
  jsonfile << "]\n";
  jsonfile.close();
  if (!disk_store.empty()) {
    finalise_disks(disk_store.front());
  }
}

void FlightFlock::finalise_disks(Flock::DiskStore &disks)
{
  // TODO
  for (auto &d: disks) {
    if (!d.prune)
      continue;

    if (!d.duration)
      continue;

    std::list<Flock::DiskStore::const_iterator> chain;
    auto it = d.it_prev;
    double av = d.points.size();
    int n = 1;

    while (it->duration) {
      av += it->points.size();
      n++;
      chain.push_front(it);
      it = it->it_prev;
    };

    if (d.duration >= min_duration) {
      if (n_flock>0) {
        jsonfile << ",\n";
      }

      jsonfile << "{\n"
               << std::fixed
               << "\"duration\": " << d.duration << ", "
               << std::setprecision(1)
               << "\"av_size\": " << av/n << ",\n"
               << "\"trace\": [\n";
      first_out = true;

      if (debug)
        outfile << "# duration " << d.duration << " size " << (int)(av/n+0.5) << "\n";

      int i = 0;
      for (auto &it: chain) {
        if (i++ % 15 == 0) {
          report_disk(*it);
        }
      };
      report_disk(d);

      if (debug)
        outfile << "\n";

      jsonfile << "],\n"
               << std::fixed
               << std::setprecision(5)
               << "\"bounds\": {"
               << "\"west\": " << bounds.GetWest().Degrees() << ", "
               << "\"east\": " << bounds.GetEast().Degrees() << ", "
               << "\"north\": " << bounds.GetNorth().Degrees() << ", "
               << "\"south\": " << bounds.GetSouth().Degrees()
               << "}}\n";

      n_flock++;
    }
  }
  // disks.prune_disks();
}


void FlightFlock::report_disk(const Flock::DiskNode& d)
{
  const Flock::Point c = d.calc_center();
  const GeoPoint p = unproject_loc(c);
  if (debug)
    outfile << d.t
            << std::fixed
            << std::setprecision(5)
            << " " << p.longitude.Degrees()
            << " " << p.latitude.Degrees()
            << std::fixed
            << std::setprecision(1)
            << " " << c.x
            << " " << c.y
            << " " << d.points.size()
            << "\n";

  if (!first_out) {
    jsonfile << ",\n";
    bounds.Extend(p);
  } else {
    bounds = GeoBounds(p);
  }
  jsonfile << "{"
           << "\"t\": " << d.t << ", "
           << std::fixed
           << std::setprecision(5)
           << "\"lng\": " << p.longitude.Degrees() << ", "
           << "\"lat\": " << p.latitude.Degrees() << ", "
           << "\"members\": [";
  bool first_id = true;
  for (auto &i: d.points) {
    if (!first_id) {
      jsonfile << ", ";
    }
    jsonfile << "\"" << ids[i.index] << "\"";
    first_id = false;
  }
  jsonfile << "]";
  jsonfile << "}";

  first_out = false;
}


const GeoPoint FlightFlock::unproject_loc(const Flock::Point& p) const
{
  return proj.Unproject(FlatPoint(p.x, p.y)/proj.GetApproximateScale());
}

void FlightFlock::mark_in_flock(std::list<AircraftModel>& aircraft) const
{
  for (auto &m: aircraft) {
    m.in_flock = find_flock(m.idi);
  }
}


